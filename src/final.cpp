// 192.168.4.1

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>


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
///*  //  1111111111111111111111111111111111111111111111111111111111111111111111111111
 AsyncWebServer server(80);


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

//*/   // 11111111111111111111111111111111111111111111111111111111111111111111111111111


void start(){

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



void RedTurn(){
digitalWrite(13,1); // red
int redflag = 0;
int errold = 0;
int kp1 = 0.5;
int kd1 = 0;
cx = 60;
Serial.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
myservo.write(90);
delay(1000);
myservo.write(97);
int starttim = millis();
while (millis() - starttim < 500){


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
digitalWrite(13,0); // red
}





void GreenTurn(){
digitalWrite(12,1); // green
int redflag = 0;
int errold = 0;
int kp1 = 0.5;
int kd1 = 0;
cx = 60;
Serial.println("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
myservo.write(90);
delay(1000);
myservo.write(97);
int starttim = millis();
while (millis() - starttim < 500){


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


digitalWrite(12,0); // green


}




void SetID() {


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


///*  //  22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222

WiFi.mode(WIFI_AP);
WiFi.softAP("ESP32AP","123321456");


  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

String mess = "<!DOCTYPE HTML><html><head><title>ESP Input \
    Form</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body><form action='/get'> \
    kP: <input type='text' name='Kp' value='"+Kp_text+"'><br> \
    kI: <input type='text' name='Ki' value='"+Ki_text+"'> \
    <br>kD: <input type='text' name='Kd' value='"+Kd_text+"'>\
    <br>speed: <input type='text' name='speed' value='"+speed_text+"'>\
    <br>white: <input type='text' name='white' value='"+white_text+"'>\
    <br>blue: <input type='blue' name='blue' value='"+blue_text+"'>\
    <br>timing: <input type='timing' name='timing' value='"+timing_text+"'>\
    <br>tim1: <input type='tim1' name='tim1' value='"+tim1_text+"'>\
    <br>ang1: <input type='ang1' name='ang1' value='"+ang1_text+"'>\
    <br>tim2: <input type='tim2' name='tim2' value='"+tim2_text+"'>\
    <br>ang2: <input type='ang2' name='ang2' value='"+ang2_text+"'>\
    <br>tim3: <input type='tim3' name='tim3' value='"+tim3_text+"'>\
    <br>ang3: <input type='ang3' name='ang3' value='"+ang3_text+"'>\
    <input type='submit' value='Submit'>\
  </form>\
</body></html>";
    
    request->send(200, "text/html", mess);

});

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("Kp")) {
      Kp_text = request->getParam("Kp")->value();
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    if (request->hasParam("Ki")) {
      Ki_text = request->getParam("Ki")->value();
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    if (request->hasParam("Kd")) {
      Kd_text = request->getParam("Kd")->value();
    }

    if (request->hasParam("speed")) {
      speed_text = request->getParam("speed")->value();
    }
    
    if (request->hasParam("white")) {
      white_text = request->getParam("white")->value();
    }

    if (request->hasParam("blue")) {
      blue_text = request->getParam("blue")->value();
    }

    if (request->hasParam("timing")) {
      timing_text = request->getParam("timing")->value();
    }
    if (request->hasParam("tim1")) {
      tim1_text = request->getParam("tim1")->value();
    }
    if (request->hasParam("tim2")) {
      tim2_text = request->getParam("tim2")->value();
    }
    if (request->hasParam("tim3")) {
      tim3_text = request->getParam("tim3")->value();
    }
    if (request->hasParam("ang1")) {
      ang1_text = request->getParam("ang1")->value();
    }
    if (request->hasParam("ang2")) {
      ang2_text = request->getParam("ang2")->value();
    }
    if (request->hasParam("ang3")) {
      ang3_text = request->getParam("ang3")->value();
    }



    Kp = Kp_text.toFloat();
    Ki = Ki_text.toFloat();
    Kd = Kd_text.toFloat();
    speed = speed_text.toInt();
    white = white_text.toInt();
    blue = blue_text.toInt();
    timing = timing_text.toInt();
    tim1 = tim1_text.toInt();
    tim3 = tim2_text.toInt();
    tim1 = tim3_text.toInt();
    ang1 = ang1_text.toInt();
    ang2 = ang2_text.toInt();
    ang3 = ang3_text.toInt();

    Serial.println(String(Kp,5));
    Serial.println(String(Ki,5));
    Serial.println(String(Kd,5));


int light = 0;
 sum = 0;
for (i = 0; i < 10; i++) {
    LigR=analogRead(34);
    sum = sum + LigR;
  }
light = sum/i;

String light_text = String(light);
String right_dist,left_dist;

  if (lox2.isRangeComplete()) {
  right_dist=String(lox2.readRange());
  }
    
  if (lox1.isRangeComplete()) {
  left_dist=String(lox1.readRange());
  }
String count_text=String(count);
String turn_text = String(turn);

String mess = "<!DOCTYPE HTML><html><head><title>ESP Input \
    Form</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body> \
    Kp="+Kp_text+"  Ki="+Ki_text+"  Kd="+Kd_text+"  Light="+light_text+"  Right_dist="+right_dist+"  Left_dist="+left_dist+"  Count="+count_text+"  turn="+turn_text+" \
    <form action='/get'> \
    kP: <input type='text' name='Kp' value='"+Kp_text+"'><br> \
    kI: <input type='text' name='Ki' value='"+Ki_text+"'> \
    <br>kD: <input type='text' name='Kd' value='"+Kd_text+"'>\
    <br>speed: <input type='text' name='speed' value='"+speed_text+"'>\
    <br>white: <input type='text' name='white' value='"+white_text+"'>\
    <br>blue: <input type='blue' name='blue' value='"+blue_text+"'>\
    <br>timing: <input type='timing' name='timing' value='"+timing_text+"'>\
    <br>tim1: <input type='tim1' name='tim1' value='"+tim1_text+"'>\
    <br>ang1: <input type='ang1' name='ang1' value='"+ang1_text+"'>\
    <br>tim2: <input type='tim2' name='tim2' value='"+tim2_text+"'>\
    <br>ang2: <input type='ang2' name='ang2' value='"+ang2_text+"'>\
    <br>tim3: <input type='tim3' name='tim3' value='"+tim3_text+"'>\
    <br>ang3: <input type='ang3' name='ang3' value='"+ang3_text+"'>\
    <input type='submit' value='Submit'>\
  </form>\
</body></html>";

    request->send(200, "text/html", mess);
  });
  server.onNotFound(notFound);
  server.begin();

//*/  //  222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222




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

void loop() {


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

    if (cx > -1 && timMilis - millisold > 300){

    millisold = millis();

    if (rd[0] == 'g'){
    Serial.println("g");
    GreenTurn();
    }


    if (rd[0] == 'r'){
    Serial.println("r");
    RedTurn();
    }


    
   
  }
  
  }  
   
  


  





}