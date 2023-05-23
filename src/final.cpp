// Добавление всех необходимых библиотек //
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>

// Создание переменных //
#define RXD2 16
#define TXD2 17

int P;
int I;
int D;
int white = 3500;
int blue = 3270;
int timing = 700;
int lastError = 0, j, startflag, error;

int cx, millisold = 0;
int speed = 98;

int oldmil;
char color;
float Kp=0.1, Ki=0, Kd=1;
int16_t Fread, Lread, Rread, LigR, ButR = 0, count = 0;
const char* ssid = "DIR-615-effb";
const char* password = "18050523";
constexpr size_t size = 1000;
double numbers[size] {0}; 
int flag = 0, u, turn, porog;
int distance, sum  = 0, i = 0, sum1 = 0, Min = 9999;
int timMilis;
int timOLD;

int tim1 = 1000, tim2 = 1000, tim3 = 1000, ang1 = 65, ang2 = 65, ang3 = 65;







String Kp_text = String(Kp, 5);
String Ki_text = String(Ki, 5);
String Kd_text = String(Kd, 5);
String white_text = String(white);
String blue_text = String(blue);
String speed_text = String(speed);
String timing_text = String(timing);

String tim1_text = String(tim1);
String tim2_text = String(tim2);
String tim3_text = String(tim3);

String ang1_text = String(ang1);
String ang2_text = String(ang2);
String ang3_text = String(ang3);





// Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

Servo myservo;
Servo servo1;

void start(){  // Функция определения направления и положения ТС //

  servo1.write(84);

  while (i<10) {
    if (lox2.isRangeComplete()) {
    distance=lox2.readRange();
    if (distance < 999){
      Rread = distance;
      i=i+1;
      sum1 = sum1 + Rread;
    }
  }
  }

  Rread = sum1 / i;



if (Rread<550 && Rread>380){
turn=1;
Serial.print("Center1: ");
}
//Serial.println(Rread);


if (Rread>=550){
turn=2;
Serial.print("Left1: ");
}
//Serial.println(Rread);

if (Rread<=380){
  turn=3;
Serial.print("Right1: ");
}
//Serial.println(Rread);

myservo.write(103);
Serial.print("Turn: ");
Serial.println(turn);
Serial.print("dist: ");
Serial.println(Rread);






while (flag == 0){



LigR=analogRead(34);
//Serial.print(LigR);
//Serial.print("                        ");
sum=0;

if (LigR<white){
  for (i = 0; i < 10; i++) {
    LigR=analogRead(34);
    sum = sum + LigR;
  }

porog = sum/i;
Serial.println(porog);
if (porog<white){
flag = 1;
while (porog<white){


LigR=analogRead(34);
//Serial.print(LigR);
//Serial.print("                        ");
sum=0;

  for (i = 0; i < 10; i++) {
    LigR=analogRead(34);
    sum = sum + LigR;
  }

porog = sum/i;


  if (porog<Min){
    Min = porog;
  } 
}   
}
}
}





Serial.println(Min);

if (Min > blue){
  color = 'O';
Serial.print("ORANGE");

if (turn == 3){     // left
servo1.write(ang3);
delay(tim3);
servo1.write(84);
delay(200);

}
if (turn == 1){     // left
servo1.write(ang1);
delay(tim1);
servo1.write(84);
delay(200);

}
if (turn == 2){     // left
servo1.write(ang2);
delay(tim2);
servo1.write(84);
delay(200);

}

}
else{
  color = 'B';
Serial.print("BLUE");

if (turn == 2){     // left
servo1.write(104);
delay(1000);
servo1.write(84);
delay(200);
}
if (turn == 1){     // center
servo1.write(104);
delay(1000);
servo1.write(84);
delay(200);
}
if (turn == 3){     // right
servo1.write(104);
delay(1000);
servo1.write(84);
delay(200);

}

}




}

void RedTurn(){  // Функция объезда красного знака //
digitalWrite(13,1); // red
int redflag = 0;
int errold = 0;
int kp1 = 0.5;
int kd1 = 0;
cx = 60;
Serial.println("RED");
myservo.write(90);
delay(1000);
digitalWrite(13,0); // red

myservo.write(97);

int starttim = millis();
while (millis() - starttim < 1000){


  if (Serial2.available() > 0)
  { // если есть доступные данные

    String rd = Serial2.readStringUntil('\n');

    String rc = rd.substring(1,4);
    cx = rc.toInt();
    Serial.print(cx);
    
   
  }

  error = cx - 60;
  u = error * 0.5;// + kd1 * (error - errold);
  errold = error;

    if (u>30){
       u = 30;
    }
    if (u<-30){
       u = -30;
    }
   servo1.write(80-u);

}

myservo.write(100);
servo1.write(60);
delay(400);
servo1.write(105);
delay(450);

myservo.write(90);
//delay(5000);
millisold = millis();
}

void GreenTurn(){  // Функция объезда зеленого знака //
digitalWrite(12,1); // green

int redflag = 0;
int errold = 0;
int kp1 = 0.5;
int kd1 = 0;
cx = 60;
Serial.println("GREEN");
myservo.write(90);
delay(1000);
digitalWrite(12,0); // green

myservo.write(97);
int starttim = millis();
while (millis() - starttim < 1000){


  if (Serial2.available() > 0)
  { // если есть доступные данные

    String rd = Serial2.readStringUntil('\n');

    String rc = rd.substring(1,4);
    cx = rc.toInt();
    Serial.print(cx);
    
   
  }

  error = cx - 120;
 u = error * 0.5;// + kd1 * (error - errold);
  errold = error;

    if (u>30){
       u = 30;
    }
    if (u<-30){
       u = -30;
    }
   servo1.write(80-u);

}
myservo.write(100);
servo1.write(100);
delay(400);
servo1.write(55);
delay(450);

myservo.write(90);
//delay(5000);



millisold = millis();
}

void SetID() {  // Функция инициализации дальномеров //


digitalWrite(18,LOW);
digitalWrite(5,LOW);
delay(10);
digitalWrite(18,HIGH);
digitalWrite(5,HIGH);
delay(10);

digitalWrite(18,LOW);






digitalWrite(5,HIGH);

Serial.println("Adafruit VL53L0X test.");
  if (!lox1.begin(0x30)) {
    Serial.println(F("Failed to boot VL53L0X"));
   // while(1);
  }
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
  lox1.setMeasurementTimingBudgetMicroSeconds(20000);
  lox1.startRangeContinuous();




digitalWrite(18,HIGH);
Serial.println("Adafruit VL53L0X test.");
  if (!lox2.begin(0x32)) {
    Serial.println(F("Failed to boot VL53L0X"));
    //while(1);
  }
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.startRangeContinuous();



}

void setup() {

pinMode(18,OUTPUT);
pinMode(5,OUTPUT);
pinMode(12,OUTPUT);
pinMode(13,OUTPUT);
pinMode(14,OUTPUT);


Serial.begin(115200);
Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  SetID();






  myservo.attach(15); 
  servo1.attach(2);
  servo1.write(84);
  myservo.write(90);
int iiiii = 0;

while (iiiii<7){
digitalWrite(13,1); // red
digitalWrite(12,0); // green
digitalWrite(14,0); // blue
delay(200);
digitalWrite(13,0); // red
digitalWrite(12,1); // green
digitalWrite(14,0); // blue
delay(200);
digitalWrite(13,0); // red
digitalWrite(12,0); // green
digitalWrite(14,1); // blue
delay(200);
digitalWrite(13,0); // red
digitalWrite(12,1); // green
digitalWrite(14,1); // blue
delay(200);
digitalWrite(13,0); // red
digitalWrite(12,0); // green
digitalWrite(14,0); // blue
delay(200);
iiiii = iiiii + 1;

}

  while (digitalRead(4) == 0){

  }

  //RedTurn();

digitalWrite(14,1); // blue
delay(500);
digitalWrite(14,0); // blue
 start();

}




void loop() {  // Основной цикл программы //


timMilis = millis();






int distance, sum, i = 0;

/*
  if (lox.isRangeComplete()) {
    Fread=lox.readRange();
    //Serial.print("FORWARD: ");
    //Serial.println(Fread);
  }
*/


if (color == 'B'){


  if (lox2.isRangeComplete()) {
    Rread = lox2.readRange();
    
    if (Rread > 1000){
      Rread = 1000;
    }
     error = 500 - Rread;

  }

      
}


else {


  if (lox1.isRangeComplete()) {
    Lread=lox1.readRange();

    if (Lread > 1000){
      Lread = 1000;
    }
      error = Lread - 500;

  }


}
      P = error;
      I = I + error;
      D = error - lastError;
      lastError = error;
      int u = P * Kp + D * Kd + I*Ki;
      //Serial.println(u);
      if (u>25){
        u = 25;
      }
      if (u<-25){
        u = -25;
      }
      myservo.write(speed);
      servo1.write(82 + u);


  LigR=analogRead(34);


if (color == 'B')
{


sum=0;
if (LigR<blue){
  for (i = 0; i < 10; i++) {
    LigR=analogRead(34);
    sum = sum + LigR;
  }

if (sum/i<white){
timOLD = millis();

servo1.write(109);

delay(1300);
servo1.write(82);
delay(200);

count = count + 1;
}
    
  }

}
else{


sum=0;
if (LigR>blue && LigR<white){
  for (i = 0; i < 5; i++) {
    LigR=analogRead(34);
    sum = sum + LigR;
  }

if (sum/i<white){
timOLD = millis();

servo1.write(59);

delay(1300);

servo1.write(82);
delay(200);



count = count + 1;
}
    
  }




}



if (count == 111){
//servo1.write(82);
////delay(500);
  //myservo.write(90);
//delay(10000000);
}




  if (Serial2.available() > 0)
  { // если есть доступные данные

    String rd = Serial2.readStringUntil('\n');

    String rc = rd.substring(1,4);
    cx = rc.toInt();
    Serial.print(cx);
    Serial.print("       ");
    Serial.println(rd);
    if (timMilis - millisold > 300 && cx<998){

    

    if (rd[0] == 'g'){
    //Serial.println("g");
    GreenTurn();
    }


    if (rd[0] == 'r'){
    //Serial.println("r");
    RedTurn();
    }


    }
   
  
  
  }  
   
  


  





}
