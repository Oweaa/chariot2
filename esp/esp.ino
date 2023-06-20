#include <SoftwareSerial.h>

#define MESSAGE_LENGTH 20

static char message[MESSAGE_LENGTH];

SoftwareSerial commSerial(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  Serial.println("begin");
  commSerial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
}

void loop() {
  while (commSerial.available() > 0) {
    Serial.println("begin while loop");
    static unsigned int message_pos = 0;

    char inByte = commSerial.read();

    if (inByte != '\n' && (message_pos < MESSAGE_LENGTH - 1) ) {
      Serial.println("if statement");
      message[message_pos] = inByte;
      message_pos++;
    } else {
      message[message_pos] = '\0';
      Serial.print("Message received: ");
      Serial.println(message);
      message_pos = 0;    
    }
    delay(1000);
  }
  String i = "TO Arduino";
  commSerial.println(i);
  delay(1500);
}
