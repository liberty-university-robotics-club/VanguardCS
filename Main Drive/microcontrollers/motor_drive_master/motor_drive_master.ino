#include <Wire.h>

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

// for incoming serial data
int right = 0;
int left = 0;	
int prevRight = 0;
int prevLeft = 0;
int rightN;
int leftN;

boolean newData = false;

//============

void setup() {
    // join i2c bus
    Wire.begin();

    Serial.begin(115200);
    Serial.println("Motor control - Master");
}

//============

void loop() {
  
    recvWithStartEndMarkers();

    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();

        newData = false;
    }
    if ((right != prevRight) || (left != prevLeft)) {
        Wire.beginTransmission(0xA);    // transmit to device 0xA
        Wire.write(right);              // sends speed
        Wire.write(rightN);             // sends 1 if negative and 0 if positive
        Wire.write(left);               // sends speed
        Wire.write(leftN);              // sends 1 if negative and 0 if positive
        Wire.endTransmission();
        Wire.beginTransmission(0xB);    // transmit to device 0xB
        Wire.write(right);              // sends speed
        Wire.write(rightN);             // sends 1 if negative and 0 if positive
        Wire.write(left);               // sends speed
        Wire.write(leftN);              // sends 1 if negative and 0 if positive
        Wire.endTransmission();
        Wire.beginTransmission(0xC);    // transmit to device 0xC
        Wire.write(right);              // sends speed
        Wire.write(rightN);             // sends 1 if negative and 0 if positive
        Wire.write(left);               // sends speed
        Wire.write(leftN);              // sends 1 if negative and 0 if positive
        Wire.endTransmission();
        
        prevRight = right;
        prevLeft = left; 
    }
    delay(100);
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                // terminate the string
                receivedChars[ndx] = '\0'; 
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    // get the first part - the string
    strtokIndx = strtok(tempChars,","); 

    // convert this part to an integer
    right = atoi(strtokIndx); 
    Serial.print("right: ");
    Serial.println(right);
    if (right < 0) {
        rightN = 1;
        right = right * -1;
    }
    else {
        rightN = 0;
    }
    
    // this continues where the previous call left off
    strtokIndx = strtok(NULL, ","); 

    // convert this part to an integer
    left = atoi(strtokIndx); 
    Serial.print("left: ");
    Serial.println(left); 
    if (left < 0) {
        leftN = 1;
        left = left * -1;
    }
    else {
        leftN = 0;
    }   

}
