CC = gcc
CFLAGS = -Wall -O2
LIBS = -lmodbus

TARGET = json-modbus-server
SRCS = json-modbus-server.c
OBJS = $(SRCS:.c=.o)
LIBS = -L/usr/lib/arm-linux-gnueabihf -lcjson -lmodbus

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lcjson $(LIBS)

clean:
	rm -f $(TARGET) $(OBJS)
