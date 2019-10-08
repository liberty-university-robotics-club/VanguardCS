#pragma once

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string>       // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions

class serialUtil{
    private:
        const char *m_dev;
        int m_fd;

    public:
        serialUtil(const char *dev);
        bool open(speed_t);
        void close();
        void write(std::string);
        int read(int);
};