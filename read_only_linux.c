#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#define BUFFER_SIZE 1024

void error_exit(const char* message) {
    perror(message);
    exit(EXIT_FAILURE);
}

int main() {
    int serial_fd;
    char buffer[BUFFER_SIZE];
    ssize_t bytesRead;

    serial_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        error_exit("Error opening serial port");
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);  // Set baud rate to 9600
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Mask data size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cc[VMIN] = 1;      // Minimum number of characters to read
    options.c_cc[VTIME] = 0;     // Timeout in deciseconds for read (0 for non-blocking)

    tcsetattr(serial_fd, TCSANOW, &options);  // Apply settings

    while (1) {
        bytesRead = read(serial_fd, buffer, sizeof(buffer)-1);
        if (bytesRead == -1) {
            error_exit("Error reading from serial port");
        } else if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; 
            printf("Received: %s", buffer);
        }
    }

    close(serial_fd);

    return 0;
}
