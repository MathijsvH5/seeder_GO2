#include <iostream>
#include <fcntl.h>    // O_RDWR, O_NOCTTY, O_NDELAY
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cstring>    // strerror
#include <cerrno>     // errno

// ALSO TEST THIS ONE!
void readResponses(int port) {
    char buffer[256];
    int bytes_read;
    bool got_reply = false;
    
    std::cout << "ESP32 replied: ";
    // Try to read all available bytes. Since we use O_NDELAY, it returns -1 when empty.
    while ((bytes_read = read(port, buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytes_read] = '\0'; // Make it a safe string
        std::cout << buffer;
        got_reply = true;
    }
    
    if (got_reply) {
        std::cout << std::endl;
    } else {
        std::cout << "(no reply)" << std::endl;
    }
}

void sendCommand(int port, const std::string& command) {
    // 1. Clear out any pending replies BEFORE sending
    readResponses(port);
    
    std::string payload = command + "\n";
    std::cout << "Sending: " << command << " ... " << std::flush;
    
    int bytes_written = write(port, payload.c_str(), payload.length());
    
    if (bytes_written < 0) {
        std::cerr << "FAILED! Error: " << strerror(errno) << std::endl;
    } else {
        std::cout << "Success (" << bytes_written << " bytes sent)." << std::endl;
    }
    
    // 2. Give the ESP32 a little more time to process, then read its reply
    usleep(200000); // Wait 0.2 seconds
    readResponses(port);
}

int main() {
    std::cout << "--- ESP32 Bluetooth Test ---" << std::endl;
    
    int fd = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd < 0) {
        std::cerr << "\nCRITICAL ERROR: Could not open /dev/rfcomm0!" << std::endl;
        std::cerr << "Linux says: " << strerror(errno) << std::endl;
        std::cerr << "Did you run ./connect_dog.sh first?" << std::endl;
        return 1;
    }
    
    // fcntl(fd, F_SETFL, 0); // Keep non-blocking for readResponses

    std::cout << "Port /dev/rfcomm0 opened successfully!\n" << std::endl;

    struct termios tty;
    tcgetattr(fd, &tty);

    cfmakeraw(&tty); 

    tcsetattr(fd, TCSADRAIN, &tty);

    tcflush(fd, TCIOFLUSH); 
    std::cout << "Port buffers flushed and ready." << std::endl;


    sendCommand(fd, "OPEN");
    
    std::cout << "Waiting 2 seconds..." << std::endl;
    sleep(2);
    
    sendCommand(fd, "CLOSE");
    
    std::cout << "Waiting 2 seconds..." << std::endl;
    sleep(2);
    
    sendCommand(fd, "OPEN");
    
    std::cout << "Waiting 2 seconds..." << std::endl;
    sleep(2);
    
    sendCommand(fd, "CLOSE");
    
    std::cout << "Waiting 2 seconds..." << std::endl;
    sleep(2);

    sendCommand(fd, "OPEN");
    
    std::cout << "Waiting 2 seconds..." << std::endl;
    sleep(2);
    
    sendCommand(fd, "CLOSE");

    std::cout << "Waiting for final transmission to clear the airwaves..." << std::endl;
    sleep(1);

    close(fd);
    std::cout << "\nTest complete. Port closed." << std::endl;
    
    return 0;
}