#include <iostream>
#include <string>           // string function definitions
#include "serialUtil.h"

bool verifyDevice(serialUtil&);

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

    //passes device file and opens it
    serialUtil s1(p);
    s1.open(B9600);

    if(verifyDevice(s1)){
        std::cout << "Correct Device Connected" << std::endl;
    } else {
        std::cout << "Incorrect Device Connected" << std::endl;
    }

    // TEST CODE
    // asks user for text then sends it to arduino over serial and loops
    while(true){
        message = "";
        std::cout << "Enter Text: ";
        std::getline(std::cin, message);
        if(message == "-d")
            break;
        
        s1.write(message);
    }
    //closes com port
    s1.close();
    

	return 0;
}

bool verifyDevice(serialUtil& s){

     char c = s.read(1);
     std::cout << c << std::endl;
     if(c == 'A'){
        s.write("a");
        return true;
    } else {
        return false;
    }
    
}