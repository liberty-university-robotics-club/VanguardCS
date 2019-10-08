#include "serialUtil.h"

serialUtil::serialUtil(const char *device){
    m_dev = device;
    m_fd = -1;
}

bool serialUtil::open(speed_t baud){

    if (m_fd >= 0){
        close();
    }

    //------------------------------- Opening the Serial Port -------------------------------
    m_fd = ::open(m_dev, O_RDWR | O_NOCTTY);

    if(m_fd < 0) {                       // Error Checking 
        printf("Error while opening the device: %s\n", m_dev);
        return false;
    }

    //---------- Setting the Attributes of the serial port using termios structure ---------
    struct termios SerialPortSettings;  // Create the structure                          
    tcgetattr(m_fd, &SerialPortSettings); // Get the current attributes of the Serial port

    // Setting the Baud rate
    cfsetispeed(&SerialPortSettings, baud); // Set Read  Speed as 9600                       
    cfsetospeed(&SerialPortSettings, baud); // Set Write Speed as 9600                       

    SerialPortSettings.c_cflag &= ~PARENB;                          // Disables the Parity Enable bit(PARENB),So No Parity   
    SerialPortSettings.c_cflag &= ~CSTOPB;                          // CSTOPB = 2 Stop bits, here it is cleared so 1 Stop bit 
    SerialPortSettings.c_cflag &= ~CSIZE;                           // Clears the mask for setting the data size             
    SerialPortSettings.c_cflag |=  CS8;                             // Set the data bits = 8                                 
    SerialPortSettings.c_cflag &= ~CRTSCTS;                         // No Hardware flow Control                         
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;                   // Enable receiver,Ignore Modem Control lines        
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p 
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode 
    SerialPortSettings.c_oflag &= ~OPOST;                           // No Output Processing

    // Setting Time outs 
    SerialPortSettings.c_cc[VMIN] = 1;  // Read at least 1 characters 
    SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly  

    if((tcsetattr(m_fd, TCSANOW, &SerialPortSettings)) != 0){ // Set the attributes to the termios structure
        printf("Error while setting attributes \n");
    }

    /* Flush Port */
    tcflush(m_fd, TCIFLUSH );
    return true;
}

void serialUtil::close(){
    tcdrain(m_fd);
    ::close(m_fd);
    m_fd = -1;
}

void serialUtil::write(std::string message){
    int messageLen = message.length();
    char str[messageLen];
    message.copy(str, messageLen);
    ::write(m_fd, &str, messageLen);
}

int serialUtil::read(int n) {
    //fcntl(m_fd, F_SETFL, FNDELAY);
    char c;
    ::read(m_fd, &c, n);
    return c;
}