// include libraries
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// define
#define MESSAGE_LENGTH 15

// servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ToF sensor
VL53L0X lox;

// define servo data
#define SERVOMIN 94
#define SERVOMIN 500
#define SERVO_FREQ 50

// set servo pins
uint8_t servonum_0 = 0;
uint8_t servonum_1 = 1;
uint8_t servonum_2 = 2;

// variables distance data
int distanceData[] = {0,0,0}; // [distance left, distance forward, distance right]
int array_length = sizeof(distanceData) / sizeof(int);
String distanceSensorData = "";

// array for received messages from server
char message[MESSAGE_LENGTH];

void setup() {
  // Setup serial
  Serial.begin(9600);
  
  // Initialize the PWM and servo shield
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); // Set the PWM frequency (usually 50 Hz for servos)

  delay(10);

  // Setup ToF sensor
  lox.init();
  lox.setTimeout(500);
  lox.startContinuous();

  // center sensor servo, ToF sensor looking forward
  pwm.setPWM(servonum_0, 0, angle_to_pulse(90));

}

uint16_t angle_to_pulse(int angle) {
  float pulse_len;
  // 0 degree leads to SERVOMIN (should be 1000 uS)
  // 180 degrees leads to SERVOMAX (should be 2000 uS)
  // anything in the middle is linearly integrated 
  angle = constrain(angle,0,180) ; // to be safe (memory 32632)
  pulse_len = SERVOMIN +(SERVOMIN - SERVOMIN) / 180.0 * float(angle);
//  Serial.println(pulse_len);
  return(pulse_len);
}

void get_distance_array(){
  // set sensor servo to 0, 90 and 180 degrees, scans the distance and put them into a array
  for(int angle = 0; angle <= 180; angle += 90) {
//    Serial.println(angle);
    pwm.setPWM(servonum_0, 0, angle_to_pulse(angle));
    distanceData[angle / 90] = lox.readRangeContinuousMillimeters();
    delay(1000);
  }
}

void print_sensor_data() {
  Serial.println("sensor data: ");
  for (int i = 0; i < array_length; i++) {
     distanceSensorData += String(distanceData[i]);

     // Add separator
     if(i < array_length - 1) {
       distanceSensorData += ", ";
     }
  }
  Serial.println(distanceSensorData);
  distanceSensorData = "";
}

void drive_forward() {
  // servo 1 angle_to_pulse(120) rotate forward, servo 2 angle_to_pulse(75) rotate forward
  pwm.setPWM(servonum_1, 0, angle_to_pulse(120));
  pwm.setPWM(servonum_2, 0, angle_to_pulse(75));
}

void drive_backward() {
  // servo 1 angle_to_pulse(75) rotate backward, servo 2 angle_to_pulse(120) rotate backward
  pwm.setPWM(servonum_1, 0, angle_to_pulse(75));
  pwm.setPWM(servonum_2, 0, angle_to_pulse(120));
}

void drive_stop() {
  // servo 1 angle_to_pulse(100) stop rotating, servo 2 angle_to_pulse(100) stop rotating
  pwm.setPWM(servonum_1, 0, angle_to_pulse(100));
  pwm.setPWM(servonum_2, 0, angle_to_pulse(100));
}

void loop() {
  while (Serial.available() > 0) {
    int message_pos = 0;

    char inByte = Serial.read();

    if (inByte != '\n' && (message_pos < MESSAGE_LENGTH - 1)) {
      message[message_pos] = inByte;
      message_pos++;
    } else {
      message[message_pos] = '\0';
      Serial.print("Message received: ");
      Serial.println(message);
      message_pos = 0;    
    }
  }
  delay(1000);
}
