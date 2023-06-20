#include <SoftwareSerial.h>

#define MESSAGE_LENGTH 20

static char message[MESSAGE_LENGTH];

SoftwareSerial commSerial(3, 2); // RX, TX

void setup() {
  Serial.begin(115200);
  commSerial.begin(115200);
}

void loop() {
  while (commSerial.available() > 0) {
    static unsigned int message_pos = 0;

    char inByte = commSerial.read();

    if (inByte != '\n' && (message_pos < MESSAGE_LENGTH - 1) ) {
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
  commSerial.println("To ESP");
  delay(1000);
}
