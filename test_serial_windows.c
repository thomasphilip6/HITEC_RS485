#include <stdio.h>
#include <windows.h>
#include <stdint.h>

HANDLE hSerial;
DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};
#define BUFFER_READ_SIZE 128
void error_exit(const char* message) {
    fprintf(stderr, "%s. Error code: %d\n", message, GetLastError());
    exit(1);
}

void serial_init(){
    hSerial = CreateFile("\\\\.\\COM17", GENERIC_READ | GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        error_exit("Error in opening serial port");
    }
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        error_exit("Error getting device state");
    }
    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        error_exit("Error setting device parameters");
    }

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        error_exit("Error setting timeouts");
    }

}

void send_data(char* message){
    DWORD bytes_written;
    if (!WriteFile(hSerial, message, strlen(message), &bytes_written, NULL)) {
        error_exit("Error writing to serial port");
    }

    printf("Data written to serial port successfully\n");
}
char* read_serial() {
    static char readBuffer[BUFFER_READ_SIZE];
    DWORD bytesRead;
    
    if (!ReadFile(hSerial, readBuffer, sizeof(readBuffer) - 1, &bytesRead, NULL)) {
        error_exit("Error reading from serial port");
    }

    readBuffer[bytesRead] = '\0';
    return readBuffer;
}
int main() {
    serial_init();
    send_data("hello world\n");
    while (1) {
        char* data = read_serial();
        printf("Received: %s\n", data);
    }
    CloseHandle(hSerial);
    return 0;
}
