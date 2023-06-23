#include <SoftwareSerial.h>

#define MESSAGE_LENGTH 25

static char message[MESSAGE_LENGTH];

int a[] = {1, 2, 3};

SoftwareSerial commSerial(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  commSerial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
}

void loop() {
//  Serial.println(commSerial.available());
//  while (commSerial.available() > 0) {
//    Serial.println("test");
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
//      message_pos = 0;    
//    }
//    delay(1000);
//  }
  float i = 1;
  commSerial.println(i);
  delay(1000);
}
