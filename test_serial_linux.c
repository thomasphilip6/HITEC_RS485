#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#define BUFFER_READ_SIZE 1024

int hSerial; 

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

void serial_init() {
    hSerial = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (hSerial == -1) {
        error_exit("Error opening serial port");
    }

    struct termios options;
    tcgetattr(hSerial, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Mask data size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cc[VMIN] = 1;      // Minimum number of characters to read
    options.c_cc[VTIME] = 0;     // non_blocking

    tcsetattr(hSerial, TCSANOW, &options);  // Apply settings
    tcflush(hSerial, TCIOFLUSH);
}

void send_data(const char* message) {
    ssize_t bytes_written = write(hSerial, message, strlen(message));
    if (bytes_written == -1) {
        error_exit("Error writing to serial port");
    }

    printf("Data written to serial port successfully\n");
}

char* read_serial() {
    static char readBuffer[BUFFER_READ_SIZE];
    ssize_t bytesRead;

    while (1) {
        bytesRead = read(hSerial, readBuffer, sizeof(readBuffer) - 1);
        printf("Bytes read: %zd\n", bytesRead);
        if (bytesRead == -1) {
            if (errno == EAGAIN) {
                // No data available, sleep briefly and retry
                usleep(100000); 
                printf("if EAGAIN\n");
                continue;
            } else {
                // Other read error
                error_exit("Error reading from serial port");
            }
        } else {
            break; 
        }
    }

    readBuffer[bytesRead] = '\0';
    return readBuffer;
}

int main() {
    serial_init();
    send_data("hello world\n");
    usleep(100000);  // 100ms delay
    while (1) {
        printf("looping\n");
        char* data = read_serial();
        printf("Received: %s", data);
    }
    close(hSerial); 
    return 0;
}

