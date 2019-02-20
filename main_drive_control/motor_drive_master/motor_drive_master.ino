#include <SPI.h>

const int SS1 = 10;
const int SS2 = 9;
const int SS3 = 8;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

// for incoming serial data
int right = 0;
int left = 0;	
int prevRight = 0;
int prevLeft = 0;

boolean newData = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Motor control - Master");

  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);
  digitalWrite(SS3, HIGH);

  SPI.begin();

}

void loop() {
  
    recvWithStartEndMarkers();

    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();

        //showParsedData();

        newData = false;
    }
    if (right != prevRight || left != prevLeft) {
        // enable Slave Select and send speed values
        digitalWrite(SS1, LOW);
        SPI.transfer(right, left);
        digitalWrite(SS1, HIGH);
        digitalWrite(SS2, LOW);
        SPI.transfer(right, left);
        digitalWrite(SS2, HIGH);
        digitalWrite(SS3, LOW);
        SPI.transfer(right, left);
        digitalWrite(SS3, HIGH);   
    }

    prevRight = right;
    prevLeft = left;
}

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
                receivedChars[ndx] = '\0'; // terminate the string
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

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    right = atoi(strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    left = atoi(strtokIndx);     // convert this part to an integer

}

//============

void showParsedData() {
    Serial.println();
    Serial.print("Right ");
    Serial.println(right);
    Serial.println();
    Serial.print("Left ");
    Serial.println(left);
    Serial.println();
}
