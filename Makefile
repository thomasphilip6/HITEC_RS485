CC = gcc
CFLAGS = -Wall -Werror
LDFLAGS = -lwiringPi

all: test_control_servo

test_control_servo: uart_comms.o servo_control.o
	$(CC) $(CFLAGS) -o test_control_servo uart_comms.o servo_control.o $(LDFLAGS)

uart_comms.o: uart_comms.c uart_comms.h
	$(CC) $(CFLAGS) -c uart_comms.c -o uart_comms.o

servo_control.o: servo_control.c uart_comms.h servo_control.h
	$(CC) $(CFLAGS) -c servo_control.c -o servo_control.o

clean:
	rm -f *.o test_control_servo

