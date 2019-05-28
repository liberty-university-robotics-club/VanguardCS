#include <iostream>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions

int main(int argc, char *argv[]){
    std::string message = "";
    std::string path;
    int messageLen;

    // gets the path to serial device e.g. "//dev//ttyACM0"
    path.append(argv[1]);
    int pathLen = path.length();
    char p[pathLen];
    path.copy(p, pathLen);
    std::cout << p << std::endl;
    std::cout << pathLen << std::endl;

//------------------------------- Opening the Serial Port -------------------------------
    int fd;                             //device file id
    fd = open(p, O_RDWR | O_NOCTTY);
    if(fd == -1)                        // Error Checking 
        printf("Error while opening the device: %s\n", p);

//---------- Setting the Attributes of the serial port using termios structure ---------
    struct termios SerialPortSettings;  // Create the structure                          
    tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port

// Setting the Baud rate
    cfsetispeed(&SerialPortSettings, B9600); // Set Read  Speed as 9600                       
    cfsetospeed(&SerialPortSettings, B9600); // Set Write Speed as 9600                       

    SerialPortSettings.c_cflag &= ~PARENB;                          // Disables the Parity Enable bit(PARENB),So No Parity   
    SerialPortSettings.c_cflag &= ~CSTOPB;                          // CSTOPB = 2 Stop bits, here it is cleared so 1 Stop bit 
    SerialPortSettings.c_cflag &= ~CSIZE;                           // Clears the mask for setting the data size             
    SerialPortSettings.c_cflag |=  CS8;                             // Set the data bits = 8                                 
    SerialPortSettings.c_cflag &= ~CRTSCTS;                         // No Hardware flow Control                         
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;                   // Enable receiver,Ignore Modem Control lines        
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p 
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode 
    SerialPortSettings.c_oflag &= ~OPOST;                           //No Output Processing

// Setting Time outs 
    SerialPortSettings.c_cc[VMIN] = 1;  // Read at least 1 characters 
    SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly  

    if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
    printf("Error while setting attributes \n");

    /* Flush Port */
    tcflush(fd, TCIFLUSH );

    // TEST CODE
    // asks user for text then sends it to arduino over serial and loops
    while(true){
        message = "";
        std::cout << "Enter Text: ";
        std::getline(std::cin, message);
        if(message == "-d")
            break;
        messageLen = message.length();

        std::cout << message << std::endl;
        std::cout << messageLen << std::endl;
        char str[messageLen];
        message.copy(str, messageLen);
        
        write(fd, &str, messageLen);
    }
    close(fd);
 
	return 0;
}