
/*  CODE WRITTEN BY JOSANU RARES IONUT for the project DriveJR - The Smart Car 
 *  VERSION: 1.0
 *  LAST MODIFIED: 08.06.2017
 */
#include <dht.h>
#include <Servo.h>

#include <SoftwareSerial.h>  

int bluetoothTx = 4;
int bluetoothRx = 2;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

//----------FRONT UltraSonic------
#define trigPinSL 53 //DIGITAL
#define echoPinSL 52 //DIGITAL
#define trigPinSR 48 //DIGITAL
#define echoPinSR 49 //DIGITAL
#define trigPinFRONT 50 //DIGITAL
#define echoPinFRONT 51 //DIGITAL
#define frontTimeout 20000 //timeout if out of range
//----------BACK UltraSonic-------
#define echoPinBACK 0 //DIGITAL
#define trigPinBACK 0//DIGITAL
#define echoPinBR 0//DIGITAL
#define trigPinBR 0//DIGITAL
#define echoPinBL 0//DIGITAL
#define trigPinBL 0//DIGITAL
//-----------PWM------------------
#define servoPin 45 //PWM
#define buzzerPin 44 //PWM
//-----------ACC & GYRO-----------
#define INT_Acc 47 //DIGITAL
#define SCL_Acc A8//ANALOG
#define SDA_Acc A9//ANALOG
//--------ADDITIONAL SENSORS-----
#define tempSensor 46 //DIGITAL
#define encoderPin A0 //DIGITAL


//--------MOTOR DRIVER-----------
#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 9
#define MOTOR1_PIN1 3 
#define MOTOR1_PIN2 5 
//--------------------SENSOR ENABLE------------------
bool SideSensor_R_ENABLED = true;
bool SideSensor_L_ENABLED = true;
bool FrontSensor_ENABLED = true;
//---------------------------------------------------

//--------------------PARKING VARIABLES--------------

float distanceBetweenObjects;
float carWidth = 16.0;
float carLenght = 26.0;
float widthError = 3.0;
float lenghtError = 5.0;
bool searchingStarted = false;
bool parkAvailable = true; //true for debugging

//---------------------------------------------------

unsigned long timerServo;
int servoCounter;

int frontLastObstaclePos = 0; //-1 left | 0 unknown | 1right
int avoidDistance = 30; // in CM

Servo frontServo;

int driveSpeed = 200;
String speedLevel = "MID";
int frontServoPos = 90; // LEFT = 15 / MID = 90 / RIGHT = 165 - HAS TO BE CALIBRATED
long duration, distance, SideSensor_R, SideSensor_L, FrontSensor; //SonarSensor
String distanceLevel = "HIGH"; //HIGH - 30+cm | MID - 15cm-29cm | LOW - 5cm-14cm

float RPM; //encoder RPM
float ROT;
bool rotCounted = false;
float countedHoles;
float oldCountedHoles;
int encoderCounter = 2;

float wheelDiameter = 0.065; //metric
float traveledDistance;

float temperature;
float humidity;
dht DHT;

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  
  pinMode(trigPinSR, OUTPUT);
  pinMode(echoPinSR, INPUT);
  pinMode(trigPinSL, OUTPUT);
  pinMode(echoPinSL, INPUT);
  pinMode(trigPinFRONT,OUTPUT);
  pinMode(echoPinFRONT,INPUT);
  pinMode(43,INPUT);
  pinMode(bluetoothTx,OUTPUT);
  pinMode(bluetoothRx,INPUT);
    
  frontServo.attach(servoPin);
  frontServo.write(frontServoPos); //Sets the servo's pos at 90 degrees | Seteaza pozitia servoului la 90 de grade
  Serial.begin(9600);
  tone(buzzerPin,2500,300);
  delay(500);
  bluetooth.begin(115200);
}

void loop() {
  //obstacleAvoid(driveSpeed);
  //followFrontObject();
  //servoPosition();
  //calculateAproxDistance();
  //readTempHum();

  
  //****DEOARECE MODULUI BLUETOOTH FOLOSIT S-A DEFECTAT,
  //****PEZENTAREA FUNCTIILOR SE VA FACE MANUAL, 
  //****SCOTAND FUNCTIA CE TREBUIE PREZENTATA DIN COMENTARII
}

void followFrontObject(){
  frontServo.write(90); //set the servo to 90degrees
  SonarSensor(trigPinFRONT, echoPinFRONT); 
  if(distance >= 100){
  FrontSensor = 100;
  }
  FrontSensor = distance;

  if(FrontSensor < avoidDistance*2){
    go(0,0);
    //tone(buzzerPin,3000,100);
  }
  if(FrontSensor > avoidDistance*2 && FrontSensor < avoidDistance){
    go(driveSpeed,driveSpeed);
    //tone(buzzerPin,3000,300);
  }
  if(FrontSensor > avoidDistance*2 && FrontSensor > avoidDistance && driveSpeed<=205){
    go(driveSpeed+50,driveSpeed+50);
    //tone(buzzerPin,3000,500);
  }
  if(FrontSensor > avoidDistance*2 && FrontSensor > avoidDistance && driveSpeed>205){
    go(driveSpeed,driveSpeed);
    //tone(buzzerPin,3000,500);
  }
}

void calculateAproxDistance(){
  go(100,100);
  if(digitalRead(A0) == 0){
  encoderCounter++;
  }
  if(digitalRead(A0) == 1){
  encoderCounter = 0;
  }
  if(encoderCounter == 1){
    countedHoles++;
  }
  traveledDistance = countedHoles / 20 * wheelDiameter * 3.14;
   Serial.println(traveledDistance);
   if(traveledDistance > 1.00){
   tone(buzzerPin,3000,500);
   }
}

void readTempHum(){

  int chk = DHT.read11(tempSensor);
  temperature = DHT.temperature;
  humidity = DHT.humidity;
}

void speedController(){ //Speed controller
  if(speedLevel == "HIGH"){
    driveSpeed = 255;
  }
  else if(speedLevel == "MID"){
    driveSpeed = 150;
  }
  else if(speedLevel == "LOW"){
    driveSpeed = 80;
  }
}

void remoteController(int carSpeed){ // Remote controller
  //use carSpeed;
}

void findParkingPlace(){ // Find Parking Place 
  if(speedLevel == "LOW"){
    //...
  }
}

void park(int parkSpeed){ // Auto-Park mode
  if(parkAvailable){
    //...
  }
}


void servoPosition(){ //Returns servoPosition / controlls servo's rotation | Returneaza pozitia servomotorului / controleaza servomotorul
  if(millis() - timerServo > 330){
    servoCounter++; 
    timerServo = millis();
  }
  if(servoCounter == 4){
    servoCounter = 0;
  }
  switch (servoCounter){
        case 0:
        frontServo.write(frontServoPos);
        frontServoPos = 45;
        break;
        
        case 1:
        frontServo.write(frontServoPos);
        frontServoPos = 90;
        break;
        
        case 2:
        frontServo.write(frontServoPos);
         frontServoPos = 135;
        break;

        case 3:
        frontServoPos = 90;
        frontServo.write(frontServoPos);
        break;
  }
}

void USSensors(){ //Using SonarSensor() + trigPin and echoPin arguments, we can read each sensor's value | Folosind functia SonarSensor() + argumentele trigPin si echoPin, putem citi valorile fiecarui senzor separat
  SonarSensor(trigPinSR, echoPinSR);
  if(distance >= 100){
  SideSensor_R = 100;
  }
  SideSensor_R = distance;
  
  SonarSensor(trigPinSL, echoPinSL); 
  if(distance >= 100){
  SideSensor_L = 100;
  }
  SideSensor_L = distance;

  SonarSensor(trigPinFRONT, echoPinFRONT); 
  if(distance >= 100){
  FrontSensor = 100;
  }
  FrontSensor = distance;
}

void obstacleAvoid(int avoidSpeed){ //Drive Assist mode function | Functie utilizata pentru modul Drive Assist
  USSensors(); //READ DATA FROM UltraSonic sensors | Citeste datele de la senzorii de distanta cu ultrasunete
 // servoPosition();
  if(SideSensor_R < avoidDistance && SideSensor_R_ENABLED == true){ //Conditie senzor lateral, fata
    frontLastObstaclePos = 1;
    tone(buzzerPin,2500,300);
    go(-avoidSpeed,avoidSpeed);
    delay(150);
  }
  else if(SideSensor_L < avoidDistance && SideSensor_L_ENABLED == true){ //Conditie senzor lateral, stanga
    frontLastObstaclePos = -1;
    tone(buzzerPin,2500,300);
    go(avoidSpeed,-avoidSpeed);
    delay(150);
  }
  else if(FrontSensor < avoidDistance && FrontSensor_ENABLED == true){ //Conditie senzor frontal + servo TO FIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
      tone(buzzerPin,2500,300);
      switch (frontServoPos) { //Cazuri pozitie servo
        case 45:
        go(-avoidSpeed,-avoidSpeed);
        delay(300);
        go(-avoidSpeed,avoidSpeed);
        delay(450);
        break;
        case 90:
        go(-avoidSpeed,-avoidSpeed);
        delay(500);
        go(-avoidSpeed,avoidSpeed);
        delay(500);
        break;
        case 135:
         go(-avoidSpeed,-avoidSpeed);
        delay(300);
        go(-avoidSpeed,avoidSpeed);
        delay(450);
        break;
      }
    
  }
  else{
    go(avoidSpeed,avoidSpeed);
    remoteController(driveSpeed);
  }
}

void SonarSensor(int trigPin,int echoPin) //Used to read the distance sensors | Functia este folosita pentru a citi valorile senzorilor de distanta
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH); //set timeout to 20000 *optional
if(duration != 0){
distance = (duration/2) / 29.1;
}
else{
  distance = 100;
}
Serial.println(distance);
}

void go(int speedLeft, int speedRight) { // go(speed1,speed2), motor driver control
  if (speedLeft > 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  } 
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }
 
  if (speedRight > 0) {
    analogWrite(MOTOR2_PIN1, speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  }else {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, -speedRight);
  }
}
