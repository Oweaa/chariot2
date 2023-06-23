#include <SoftwareSerial.h>

#define MESSAGE_LENGTH 20

static char message[MESSAGE_LENGTH];

SoftwareSerial commSerial(3, 2); // RX, TX

void setup() {
  Serial.begin(9600);
  commSerial.begin(115200);
  pinMode(3, INPUT);
  pinMode(2, OUTPUT);
  while(!Serial){}
}

void loop() {
  if (commSerial.available() > 0) {
    int myFloat = int(commSerial.parseFloat(SKIP_ALL, '\n'));

    // prints the received float number
    Serial.print("I received: ");
    Serial.println(int(myFloat));
    if (int(myFloat == 1)) {
      Serial.println("yesss");
    }
  }
//  while (commSerial.available() > 0) {
//    static unsigned int message_pos = 0;
//
//    char inByte = commSerial.read();
//
//    if (inByte != '\n' && (message_pos < MESSAGE_LENGTH - 1) ) {
//      message[message_pos] = inByte;
//      message_pos++;
//    } else {
//      message[message_pos] = '\0';
//      Serial.print("Message received: ");
//      Serial.println(message);
//      if (message == "1") {
//        Serial.println("yesss");
//      }
//      message_pos = 0;
//    }
//    delay(10);
//  }
////  commSerial.println("To ESP");
  delay(1000);
}
