#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <signal.h>
#include <errno.h>
#include <stdbool.h>
#include "cJSON/cJSON.h"

#define MAX_JSON_BUFFER 512
#define log_debug(fmt, ...) fprintf(stderr, "[DEBUG] " fmt "\n", ##__VA_ARGS__)

typedef enum { STATE_STOPPED, STATE_RUNNING } ServerState;
volatile ServerState server_state = STATE_RUNNING;

// Global running flag: when false the program will exit (acts like Ctrl-C)
volatile bool running = true;

modbus_mapping_t *mb_mapping;
modbus_t *ctx_tcp = NULL;
modbus_t *ctx_rtu = NULL;
int tcp_listen_sock = -1;

// Support multiple TCP clients
#define MAX_TCP_CLIENTS 10
int tcp_conn_socks[MAX_TCP_CLIENTS];
int tcp_conn_count = 0;

bool enable_tcp = false;
bool enable_rtu = false;

int coils_start=0, nb_coils=0;
int input_bits_start=0, nb_input_bits=0;
int holding_regs_start=0, nb_holding_regs=0;
int input_regs_start=0, nb_input_regs=0;
int unit_id=1, tcp_port=1502;
char mode[16]="tcp";
char serial_device[64]="/dev/ttyUSB0";
int baudrate=9600;
char parity='N';
int data_bits=8;
int stop_bits=1;

// Byte order for multi-register values
typedef enum { ORDER_LE, ORDER_BE, ORDER_SWAP } ByteOrder;

static int load_config(const char *file) {
    FILE *fp = fopen(file, "r"); if (!fp) return -1;
    char buf[2048]; size_t len = fread(buf,1,sizeof(buf)-1,fp); buf[len]='\0'; fclose(fp);
    cJSON *root = cJSON_Parse(buf); if(!root) return -1;
    cJSON *j;
    if((j=cJSON_GetObjectItem(root,"mode"))&&cJSON_IsString(j)) {
        strncpy(mode,j->valuestring,sizeof(mode)-1);
        // Parse mode to support "tcp", "rtu", or "tcp_rtu"
        enable_tcp = (strstr(mode,"tcp") != NULL);
        enable_rtu = (strstr(mode,"rtu") != NULL);
    }
    if((j=cJSON_GetObjectItem(root,"tcp_port"))&&cJSON_IsNumber(j)) tcp_port = j->valueint;
    if((j=cJSON_GetObjectItem(root,"unit_id"))&&cJSON_IsNumber(j)) unit_id = j->valueint;
    if((j=cJSON_GetObjectItem(root,"serial"))&&cJSON_IsObject(j)){
        cJSON *d;
        if((d=cJSON_GetObjectItem(j,"device"))&&cJSON_IsString(d)) strncpy(serial_device,d->valuestring,sizeof(serial_device)-1);
        if((d=cJSON_GetObjectItem(j,"baudrate"))&&cJSON_IsNumber(d)) baudrate = d->valueint;
        if((d=cJSON_GetObjectItem(j,"parity"))&&cJSON_IsString(d)) parity = d->valuestring[0];
        if((d=cJSON_GetObjectItem(j,"data_bits"))&&cJSON_IsNumber(d)) data_bits = d->valueint;
        if((d=cJSON_GetObjectItem(j,"stop_bits"))&&cJSON_IsNumber(d)) stop_bits = d->valueint;
    }
    // Parse coils (support both array and object format)
    if((j=cJSON_GetObjectItem(root,"coils"))){
        if(cJSON_IsObject(j)){
            cJSON *s = cJSON_GetObjectItem(j,"start_address");
            cJSON *c = cJSON_GetObjectItem(j,"count");
            if(s&&c&&cJSON_IsNumber(s)&&cJSON_IsNumber(c)){
                coils_start = s->valueint;
                nb_coils = c->valueint;
            }
        } else if(cJSON_IsArray(j)&&j->child&&j->child->next){
            coils_start = j->child->valueint;
            nb_coils = j->child->next->valueint;
        }
    }
    // Parse input_bits
    if((j=cJSON_GetObjectItem(root,"input_bits"))){
        if(cJSON_IsObject(j)){
            cJSON *s = cJSON_GetObjectItem(j,"start_address");
            cJSON *c = cJSON_GetObjectItem(j,"count");
            if(s&&c&&cJSON_IsNumber(s)&&cJSON_IsNumber(c)){
                input_bits_start = s->valueint;
                nb_input_bits = c->valueint;
            }
        } else if(cJSON_IsArray(j)&&j->child&&j->child->next){
            input_bits_start = j->child->valueint;
            nb_input_bits = j->child->next->valueint;
        }
    }
    // Parse holding_registers
    if((j=cJSON_GetObjectItem(root,"holding_registers"))){
        if(cJSON_IsObject(j)){
            cJSON *s = cJSON_GetObjectItem(j,"start_address");
            cJSON *c = cJSON_GetObjectItem(j,"count");
            if(s&&c&&cJSON_IsNumber(s)&&cJSON_IsNumber(c)){
                holding_regs_start = s->valueint;
                nb_holding_regs = c->valueint;
            }
        } else if(cJSON_IsArray(j)&&j->child&&j->child->next){
            holding_regs_start = j->child->valueint;
            nb_holding_regs = j->child->next->valueint;
        }
    }
    // Parse input_registers
    if((j=cJSON_GetObjectItem(root,"input_registers"))){
        if(cJSON_IsObject(j)){
            cJSON *s = cJSON_GetObjectItem(j,"start_address");
            cJSON *c = cJSON_GetObjectItem(j,"count");
            if(s&&c&&cJSON_IsNumber(s)&&cJSON_IsNumber(c)){
                input_regs_start = s->valueint;
                nb_input_regs = c->valueint;
            }
        } else if(cJSON_IsArray(j)&&j->child&&j->child->next){
            input_regs_start = j->child->valueint;
            nb_input_regs = j->child->next->valueint;
        }
    }
    cJSON_Delete(root);
    return 0;
}

// Parse byte order string
static ByteOrder parse_byte_order(const char *s) {
    if (!s || strcasecmp(s, "LE") == 0) return ORDER_LE;
    if (strcasecmp(s, "BE") == 0) return ORDER_BE;
    if (strcasecmp(s, "SWAP") == 0) return ORDER_SWAP;
    return ORDER_LE;
}

// Helper: Set file descriptor to non-blocking mode
static void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if(flags != -1) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Helper: Find free slot in TCP client array
static int find_free_tcp_client_slot(void) {
    for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if(tcp_conn_socks[i] == -1) return i;
    }
    return -1;  // No free slot
}

// Signal handler for graceful shutdown
static void signal_handler(int sig) {
    if(sig == SIGTERM || sig == SIGINT) {
        server_state = STATE_STOPPED;
        running = false;
        log_debug("Received signal %d, shutting down", sig);
    }
}

// Write multi-word data into registers with correct byte order
static void write_registers(int start_idx, uint16_t *words, int count, ByteOrder order) {
    // Swap bytes within words if SWAP
    if(order == ORDER_SWAP) {
        for(int i=0;i<count;i++) {
            words[i] = (words[i] >> 8) | (words[i] << 8);
        }
    }
    // Swap word order if BE (for 2-word data)
    if(order == ORDER_BE && count == 2) {
        uint16_t tmp = words[0]; words[0] = words[1]; words[1] = tmp;
    }
    for(int i=0;i<count;i++) {
        mb_mapping->tab_registers[start_idx + i] = words[i];
    }
}

static void process_json(const char *json_str) {
    // log_debug("Received JSON: %s", json_str);
    cJSON *root = cJSON_Parse(json_str); if(!root) return;
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root,"cmd");
    if(cJSON_IsString(cmd)){
        if(strcmp(cmd->valuestring,"stop")==0){
            server_state = STATE_STOPPED;
            running = false;
            printf("{\"status\":\"stopping\"}\n");
            cJSON_Delete(root);
            return;
        }
        if(strcmp(cmd->valuestring,"start")==0){
            if(server_state == STATE_RUNNING) {
                printf("{\"error\":\"already_running\"}\n");
                cJSON_Delete(root);
                return;
            }
            server_state = STATE_RUNNING;
            printf("{\"status\":\"starting\"}\n");
            cJSON_Delete(root);
            return;
        }
        if(strcmp(cmd->valuestring,"status")==0){
            printf("{\"status\":\"%s\"}\n", server_state == STATE_RUNNING ? "running" : "stopped");
            cJSON_Delete(root);
            return;
        }
    }
    // Data update
    cJSON *type     = cJSON_GetObjectItemCaseSensitive(root,"type");
    cJSON *addr     = cJSON_GetObjectItemCaseSensitive(root,"address");
    cJSON *datatype = cJSON_GetObjectItemCaseSensitive(root,"datatype");
    cJSON *order_it = cJSON_GetObjectItemCaseSensitive(root,"byte_order");
    cJSON *val      = cJSON_GetObjectItemCaseSensitive(root,"value");
    if(!cJSON_IsString(type)||!cJSON_IsNumber(addr)||!cJSON_IsString(datatype)||!val) { cJSON_Delete(root); return; }
    
    // Validate address
    int addr_val = addr->valueint;
    if(addr_val < holding_regs_start || addr_val >= holding_regs_start + nb_holding_regs) {
        printf("{\"error\":\"address_out_of_range\",\"address\":%d}\n", addr_val);
        cJSON_Delete(root);
        return;
    }
    
    int idx = addr_val - holding_regs_start;
    if(idx < 0 || idx >= mb_mapping->nb_registers) {
        printf("{\"error\":\"index_out_of_bounds\"}\n");
        cJSON_Delete(root);
        return;
    }
    
    ByteOrder bo = parse_byte_order(order_it ? order_it->valuestring : "LE");
    // Handle types
    if(strcmp(datatype->valuestring,"uint16")==0 && cJSON_IsNumber(val)) {
        uint16_t v = (uint16_t)val->valueint;
        if(bo == ORDER_SWAP) v = (v>>8)|(v<<8);
        mb_mapping->tab_registers[idx] = v;
    }
    else if(strcmp(datatype->valuestring,"int16")==0 && cJSON_IsNumber(val)) {
        int16_t iv = (int16_t)val->valueint;
        uint16_t v = *(uint16_t *)&iv;
        if(bo == ORDER_SWAP) v = (v>>8)|(v<<8);
        mb_mapping->tab_registers[idx] = v;
    }
    else if((strcmp(datatype->valuestring,"uint32")==0 || strcmp(datatype->valuestring,"int32")==0) && cJSON_IsNumber(val)) {
        uint32_t u = strcmp(datatype->valuestring,"int32")==0 ? (uint32_t)(int32_t)val->valueint : (uint32_t)val->valueint;
        uint16_t words[2] = { (uint16_t)(u & 0xFFFF), (uint16_t)(u >> 16) };
        write_registers(idx, words, 2, bo);
    }
    else if(strcmp(datatype->valuestring,"float")==0 && cJSON_IsNumber(val)) {
        union { float f; uint32_t u32; } fu; fu.f = (float)val->valuedouble;
        uint16_t words[2] = { (uint16_t)(fu.u32 & 0xFFFF), (uint16_t)(fu.u32 >> 16) };
        write_registers(idx, words, 2, bo);
    }
    printf("{\"status\":\"updated\",\"address\":%d,\"datatype\":\"%s\"}\n", addr_val, datatype->valuestring);
    cJSON_Delete(root);
}

// Server main loop
static void run_server(void) {
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    
    // Initialize TCP client array
    for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
        tcp_conn_socks[i] = -1;
    }
    tcp_conn_count = 0;
    
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    
    // Initialize TCP if enabled
    if(enable_tcp) {
        ctx_tcp = modbus_new_tcp(NULL, tcp_port);
        if(!ctx_tcp) { fprintf(stderr,"TCP context failed\n"); exit(EXIT_FAILURE); }
        modbus_set_slave(ctx_tcp, unit_id);
        modbus_set_debug(ctx_tcp, FALSE);
        tcp_listen_sock = modbus_tcp_listen(ctx_tcp, 1);
        if(tcp_listen_sock == -1) { fprintf(stderr,"TCP listen failed\n"); exit(EXIT_FAILURE); }
        log_debug("TCP Server listening on port %d", tcp_port);
    }
    
    // Initialize RTU if enabled
    if(enable_rtu) {
        ctx_rtu = modbus_new_rtu(serial_device, baudrate, parity, data_bits, stop_bits);
        if(!ctx_rtu) { fprintf(stderr,"RTU context failed\n"); exit(EXIT_FAILURE); }
        modbus_set_slave(ctx_rtu, unit_id);
        modbus_set_debug(ctx_rtu, FALSE);
        if(modbus_connect(ctx_rtu) == -1) { 
            fprintf(stderr,"RTU connect failed: %s\n", modbus_strerror(errno)); 
            exit(EXIT_FAILURE);
        }
        log_debug("RTU Slave connected on %s", serial_device);
    }
    
    // Initialize shared memory mapping (used by both TCP and RTU)
    mb_mapping = modbus_mapping_new_start_address(
        coils_start, nb_coils,
        input_bits_start, nb_input_bits,
        holding_regs_start, nb_holding_regs,
        input_regs_start, nb_input_regs);
    if(!mb_mapping) { 
        fprintf(stderr,"Mapping alloc failed\n"); 
        if(ctx_tcp) modbus_free(ctx_tcp);
        if(ctx_rtu) { modbus_close(ctx_rtu); modbus_free(ctx_rtu); }
        exit(EXIT_FAILURE);
    }
    
    printf("{\"status\":\"server_ready\",\"tcp\":%s,\"rtu\":%s,\"unit_id\":%d}\n",
           enable_tcp?"true":"false", enable_rtu?"true":"false", unit_id);
    
    while(server_state == STATE_RUNNING) {
        fd_set fds; FD_ZERO(&fds);
        
        // Add TCP listen socket
        if(enable_tcp && tcp_listen_sock != -1) FD_SET(tcp_listen_sock, &fds);
        
        // Add all active TCP client sockets
        if(enable_tcp) {
            for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                if(tcp_conn_socks[i] != -1) {
                    FD_SET(tcp_conn_socks[i], &fds);
                }
            }
        }
        
        // Add RTU serial port
        if(enable_rtu && ctx_rtu) {
            int rtu_fd = modbus_get_socket(ctx_rtu);
            if(rtu_fd != -1) FD_SET(rtu_fd, &fds);
        }
        
        int maxfd = 0;
        if(enable_tcp && tcp_listen_sock > maxfd) maxfd = tcp_listen_sock;
        // Find max fd among TCP clients
        if(enable_tcp) {
            for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                if(tcp_conn_socks[i] > maxfd) maxfd = tcp_conn_socks[i];
            }
        }
        if(enable_rtu && ctx_rtu) {
            int rtu_fd = modbus_get_socket(ctx_rtu);
            if(rtu_fd > maxfd) maxfd = rtu_fd;
        }
        
        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
        if(select(maxfd+1, &fds, NULL, NULL, &tv) > 0) {
            // Handle TCP listen socket - accept new connections
            if(enable_tcp && tcp_listen_sock != -1 && FD_ISSET(tcp_listen_sock, &fds)) {
                int slot = find_free_tcp_client_slot();
                if(slot != -1) {
                    int new_conn = modbus_tcp_accept(ctx_tcp, &tcp_listen_sock);
                    if(new_conn != -1) {
                        tcp_conn_socks[slot] = new_conn;
                        tcp_conn_count++;
                        log_debug("TCP client connected (slot %d, fd %d), total clients: %d", slot, new_conn, tcp_conn_count);
                    }
                } else {
                    log_debug("Max TCP clients reached (%d), rejecting new connection", MAX_TCP_CLIENTS);
                    int temp_sock = modbus_tcp_accept(ctx_tcp, &tcp_listen_sock);
                    if(temp_sock != -1) close(temp_sock);
                }
            }
            
            // Handle data from all TCP clients
            if(enable_tcp) {
                for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                    if(tcp_conn_socks[i] != -1 && FD_ISSET(tcp_conn_socks[i], &fds)) {
                        modbus_set_socket(ctx_tcp, tcp_conn_socks[i]);
                        int rc = modbus_receive(ctx_tcp, query);
                        if(rc > 0) {
                            if(modbus_reply(ctx_tcp, query, rc, mb_mapping) == -1) {
                                log_debug("TCP reply failed for client %d: %s", i, modbus_strerror(errno));
                            }
                        } else if(rc == -1) {
                            log_debug("TCP client disconnect (slot %d)", i);
                            close(tcp_conn_socks[i]);
                            tcp_conn_socks[i] = -1;
                            tcp_conn_count--;
                        }
                    }
                }
            }
            
            // Handle RTU
            if(enable_rtu && ctx_rtu) {
                int rtu_fd = modbus_get_socket(ctx_rtu);
                if(rtu_fd != -1 && FD_ISSET(rtu_fd, &fds)) {
                    int rc = modbus_receive(ctx_rtu, query);
                    if(rc > 0) {
                        if(modbus_reply(ctx_rtu, query, rc, mb_mapping) == -1) {
                            log_debug("RTU reply failed: %s", modbus_strerror(errno));
                        }
                    } else if(rc == -1) {
                        // RTU error - in run_server context, just log
                        if(errno != EAGAIN && errno != EWOULDBLOCK) {
                            log_debug("RTU error: %s", modbus_strerror(errno));
                        }
                    }
                }
            }
        }
    }
    
    // Cleanup
    if(enable_tcp) {
        // Close all client connections
        for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if(tcp_conn_socks[i] != -1) {
                close(tcp_conn_socks[i]);
                tcp_conn_socks[i] = -1;
            }
        }
        tcp_conn_count = 0;
        if(tcp_listen_sock != -1) close(tcp_listen_sock);
        modbus_free(ctx_tcp);
    }
    if(enable_rtu) {
        modbus_close(ctx_rtu);
        modbus_free(ctx_rtu);
    }
    modbus_mapping_free(mb_mapping);
}

int main(void) {
    setvbuf(stdout,NULL,_IOLBF,0);
    if(load_config("/home/pi/modbus-server/modbus_config.json")!=0) {
        fprintf(stderr,"Config load failed\n"); exit(EXIT_FAILURE);
    }
    
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    
    printf("{\"ready\":true}\n");
    fflush(stdout);
    
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    char buf[MAX_JSON_BUFFER];
    
    while(running) {
        // Check stdin for commands
        fd_set fds; FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        
        // Add server sockets only if running
        if(server_state == STATE_RUNNING) {
            if(enable_tcp && tcp_listen_sock != -1) FD_SET(tcp_listen_sock, &fds);
            // Add all active TCP client sockets
            if(enable_tcp) {
                for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                    if(tcp_conn_socks[i] != -1) {
                        FD_SET(tcp_conn_socks[i], &fds);
                    }
                }
            }
            if(enable_rtu && ctx_rtu) {
                int rtu_fd = modbus_get_socket(ctx_rtu);
                if(rtu_fd != -1) FD_SET(rtu_fd, &fds);
            }
        }
        
        int maxfd = STDIN_FILENO;
        if(server_state == STATE_RUNNING) {
            if(enable_tcp && tcp_listen_sock > maxfd) maxfd = tcp_listen_sock;
            // Find max fd among TCP clients
            if(enable_tcp) {
                for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                    if(tcp_conn_socks[i] > maxfd) maxfd = tcp_conn_socks[i];
                }
            }
            if(enable_rtu && ctx_rtu) {
                int rtu_fd = modbus_get_socket(ctx_rtu);
                if(rtu_fd > maxfd) maxfd = rtu_fd;
            }
        }
        
        struct timeval tv = {.tv_sec = 0, .tv_usec = 100000};
        int ret = select(maxfd+1, &fds, NULL, NULL, &tv);
        
        if(ret < 0) {
            if(errno == EINTR) continue;
            break;
        }
        
        // Process stdin commands
        if(FD_ISSET(STDIN_FILENO, &fds)) {
            if(fgets(buf, sizeof(buf), stdin)) {
                process_json(buf);
            } else {
                // EOF on stdin -> treat as stop (exit like Ctrl-C)
                running = false;
                server_state = STATE_STOPPED;
            }
        }
        
        // Initialize/start server on first STATE_RUNNING
        if(server_state == STATE_RUNNING && !mb_mapping) {
            // First time running
            if(!mb_mapping) {
                // Initialize TCP if enabled
                if(enable_tcp) {
                    ctx_tcp = modbus_new_tcp(NULL, tcp_port);
                    if(!ctx_tcp) { fprintf(stderr,"TCP context failed\n"); continue; }
                    modbus_set_slave(ctx_tcp, unit_id);
                    modbus_set_debug(ctx_tcp, FALSE);
                    tcp_listen_sock = modbus_tcp_listen(ctx_tcp, 1);
                    if(tcp_listen_sock == -1) { fprintf(stderr,"TCP listen failed\n"); continue; }
                    // Initialize TCP client array
                    for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                        tcp_conn_socks[i] = -1;
                    }
                    tcp_conn_count = 0;
                    log_debug("TCP Server listening on port %d", tcp_port);
                }
                
                // Initialize RTU if enabled
                if(enable_rtu) {
                    ctx_rtu = modbus_new_rtu(serial_device, baudrate, parity, data_bits, stop_bits);
                    if(!ctx_rtu) { fprintf(stderr,"RTU context failed\n"); continue; }
                    modbus_set_slave(ctx_rtu, unit_id);
                    modbus_set_debug(ctx_rtu, FALSE);
                    // Set RTU to non-blocking + short timeout so disconnects don't block stdin
                    int rtu_fd_temp = modbus_get_socket(ctx_rtu);
                    if(rtu_fd_temp != -1) {
                        set_nonblocking(rtu_fd_temp);
                        modbus_set_response_timeout(ctx_rtu, 0, 100000); // 100ms timeout
                    }
                    if(modbus_connect(ctx_rtu) == -1) { 
                        fprintf(stderr,"RTU connect failed: %s\n", modbus_strerror(errno)); 
                        modbus_free(ctx_rtu);
                        ctx_rtu = NULL;
                        continue;
                    }
                    log_debug("RTU Slave connected on %s", serial_device);
                }
                
                // Initialize shared memory mapping
                mb_mapping = modbus_mapping_new_start_address(
                    coils_start, nb_coils,
                    input_bits_start, nb_input_bits,
                    holding_regs_start, nb_holding_regs,
                    input_regs_start, nb_input_regs);
                if(!mb_mapping) { 
                    fprintf(stderr,"Mapping alloc failed\n"); 
                    continue;
                }
                
                printf("{\"status\":\"server_ready\",\"tcp\":%s,\"rtu\":%s,\"unit_id\":%d}\n",
                       enable_tcp?"true":"false", enable_rtu?"true":"false", unit_id);
            }
        }
        
        // Cleanup when stopping
        if(server_state == STATE_STOPPED && mb_mapping) {
            if(enable_tcp) {
                // Close all TCP client connections
                for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                    if(tcp_conn_socks[i] != -1) {
                        close(tcp_conn_socks[i]);
                        tcp_conn_socks[i] = -1;
                    }
                }
                tcp_conn_count = 0;
                if(tcp_listen_sock != -1) close(tcp_listen_sock);
                modbus_free(ctx_tcp);
                tcp_listen_sock = -1;
                ctx_tcp = NULL;
            }
            if(enable_rtu) {
                modbus_close(ctx_rtu);
                modbus_free(ctx_rtu);
                ctx_rtu = NULL;
            }
            modbus_mapping_free(mb_mapping);
            mb_mapping = NULL;
            printf("{\"status\":\"server_stopped\"}\n");
        }
        
        // Handle connections only when running
        if(server_state == STATE_RUNNING && ret > 0) {
            // Handle TCP listen socket - accept new connections
            if(enable_tcp && tcp_listen_sock != -1 && FD_ISSET(tcp_listen_sock, &fds)) {
                int slot = find_free_tcp_client_slot();
                if(slot != -1) {
                    int new_conn = modbus_tcp_accept(ctx_tcp, &tcp_listen_sock);
                    if(new_conn != -1) {
                        tcp_conn_socks[slot] = new_conn;
                        tcp_conn_count++;
                        log_debug("TCP client connected (slot %d, fd %d), total clients: %d", slot, new_conn, tcp_conn_count);
                    }
                } else {
                    log_debug("Max TCP clients reached (%d), rejecting new connection", MAX_TCP_CLIENTS);
                    int temp_sock = modbus_tcp_accept(ctx_tcp, &tcp_listen_sock);
                    if(temp_sock != -1) close(temp_sock);
                }
            }
            
            // Handle data from all TCP clients
            if(enable_tcp) {
                for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                    if(tcp_conn_socks[i] != -1 && FD_ISSET(tcp_conn_socks[i], &fds)) {
                        modbus_set_socket(ctx_tcp, tcp_conn_socks[i]);
                        int rc = modbus_receive(ctx_tcp, query);
                        if(rc > 0) {
                            if(modbus_reply(ctx_tcp, query, rc, mb_mapping) == -1) {
                                log_debug("TCP reply failed for client %d: %s", i, modbus_strerror(errno));
                            }
                        } else if(rc == -1) {
                            log_debug("TCP client disconnect (slot %d)", i);
                            close(tcp_conn_socks[i]);
                            tcp_conn_socks[i] = -1;
                            tcp_conn_count--;
                        }
                    }
                }
            }
            
            // Handle RTU
            if(enable_rtu && ctx_rtu) {
                int rtu_fd = modbus_get_socket(ctx_rtu);
                if(rtu_fd != -1 && FD_ISSET(rtu_fd, &fds)) {
                    int rc = modbus_receive(ctx_rtu, query);
                    if(rc > 0) {
                        if(modbus_reply(ctx_rtu, query, rc, mb_mapping) == -1) {
                            log_debug("RTU reply failed: %s", modbus_strerror(errno));
                        }
                    } else if(rc == -1) {
                        // RTU error (e.g., device disconnected) - don't exit, just log and keep trying
                        if(errno != EAGAIN && errno != EWOULDBLOCK) {
                            log_debug("RTU error: %s (device may be disconnected)", modbus_strerror(errno));
                            // Attempt to close and free RTU gracefully
                            modbus_close(ctx_rtu);
                            modbus_free(ctx_rtu);
                            ctx_rtu = NULL;
                        }
                    }
                }
            }
            
            // AUTO-RECONNECT: If RTU is NULL and server is running, try to reconnect
            if(enable_rtu && ctx_rtu == NULL && server_state == STATE_RUNNING) {
                log_debug("Attempting to reconnect RTU on %s", serial_device);
                ctx_rtu = modbus_new_rtu(serial_device, baudrate, parity, data_bits, stop_bits);
                if(ctx_rtu) {
                    modbus_set_slave(ctx_rtu, unit_id);
                    modbus_set_debug(ctx_rtu, FALSE);
                    int rtu_fd_temp = modbus_get_socket(ctx_rtu);
                    if(rtu_fd_temp != -1) {
                        set_nonblocking(rtu_fd_temp);
                        modbus_set_response_timeout(ctx_rtu, 0, 100000);
                    }
                    if(modbus_connect(ctx_rtu) == -1) {
                        log_debug("RTU reconnect failed: %s", modbus_strerror(errno));
                        modbus_free(ctx_rtu);
                        ctx_rtu = NULL;
                    } else {
                        log_debug("RTU reconnected successfully on %s", serial_device);
                    }
                }
            }
        }
    }
    
    // Final cleanup before exit
    if(mb_mapping) {
        if(enable_tcp) {
            // Close all TCP client connections
            for(int i = 0; i < MAX_TCP_CLIENTS; i++) {
                if(tcp_conn_socks[i] != -1) {
                    close(tcp_conn_socks[i]);
                    tcp_conn_socks[i] = -1;
                }
            }
            tcp_conn_count = 0;
            if(tcp_listen_sock != -1) close(tcp_listen_sock);
            if(ctx_tcp) modbus_free(ctx_tcp);
            tcp_listen_sock = -1; ctx_tcp = NULL;
        }
        if(enable_rtu) {
            if(ctx_rtu) { modbus_close(ctx_rtu); modbus_free(ctx_rtu); ctx_rtu = NULL; }
        }
        modbus_mapping_free(mb_mapping);
        mb_mapping = NULL;
    }
    printf("{\"status\":\"exited\"}\n");
    
    return 0;
}
