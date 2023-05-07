#include <Arduino.h>
#include <Servo.h>
#include "Adafruit_VL53L1X.h"


#define IRQ_PIN 25
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

  

int P;
int I;
int D;
int lastError = 0;
double Kp=0.15, Ki=0.001, Kd=0.001;

  

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to



#define RXD2 16
#define TXD2 17

// Motor A


char read1;
// Setting PWM properties

int cx;

int MotSpeed=105;
// float kp = 0.02, kd = 0.001, ki=0.003, j1old;



static const int ServoPin = 2;
static const int MotorPin = 15;


Servo servo1;
Servo servo2;



void setup() {
  // sets the pins as outputs:


  Serial.begin(115200);
Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);


while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(20);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());




  
  servo1.attach(ServoPin);
  servo2.attach(MotorPin);






  servo2.write(89);
  servo1.write(94);
  delay(3000);

  servo2.write(100);
  delay(200);
  servo2.write(110);
  delay(200);
  servo2.write(MotSpeed);




}

void loop() {


int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      distance = 4000;
     

    }

    //Serial.println(distance);

    vl53.clearInterrupt();

    int j1;
      

  // Move the DC motor forward at maximum speed

    j1 = distance;
    if (j1 > 1000){
      j1 = 1000;
    }
     
  int error = j1-300; //3500 is the ideal position (the centre)

  P = error;
  //I = I + error;
  D = error - lastError;
  lastError = error;
  int u = P * Kp + D * Kd; // + I*Ki;
  if (u>50){
    u = 50;
  }
    if (u<-50){
    u = -50;
  }

   servo1.write(98-u);


  }

  }






  //cx = 90;
/*
  if (Serial2.available() > 0)
  { // если есть доступные данные

    String rd = Serial2.readStringUntil('\n');
    cx = rd.toInt();
   

  }
     */
  

/*
servo1.write(98);
delay(500);
servo1.write(150);
delay(500);
servo1.write(98);
delay(500);

servo2.write(89);
delay(50000000);

*/

