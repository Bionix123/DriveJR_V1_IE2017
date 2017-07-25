/*------REMOTE CONTROL COMMANDS-------
 * -----------FROM APP TO CAR---------
 * TD = Transmission DRIVE
 * TN = Transmission NEUTRAL
 * TR = Transmission REVERSE
 * S1 = Speed level LOW
 * S2 = Speed level MID
 * S3 = Speed level HIGH
 * FF = Move forward
 * FR = Move forward to right
 * FL = Move forward to left
 * BR = Move backwards to right
 * BL = Move backwards to left
 * BB = Move backwords
 * DAO = Drive Assist ON
 * DAF = Drive Assist OFF
 * AP = Park mode
 * -----------FROM CAR TO APP---------
 * TEMP,HUM,ACC,GYRO,DISTANCE - Under development
 */
String BT_Data;
String currentFunction;

float outsideTemp;
int outsideHmd;

int frontServoPos;

float accX;
float accY;
float accZ;

int backSensorsDistanceLevel = 5; //5 = default - 1 = very close

//SENSORS & DEVICES
int BUZZER = 9;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  pinMode(13,OUTPUT);
  pinMode(BUZZER,OUTPUT);
}

void loop() {
recieveData();


//  remoteControl(); 
}

void recieveData(){
   if(Serial.available() > 0){
   Serial.setTimeout(10);
   BT_Data = Serial.readString();
  }
}

void sendData(){
  /*-------TO SEND---------
   * 1.Temperature - "TEMP" + outsideTemp - ex: TEMP23
   * 2.Humidity - "HMD" + outsideHmd - ex: HMD56
   * 3.Accelerometer - "accX" / "accY" / "accZ" + accX/Y/Z - ex: accX1.5
   * 4.GYROSCOPE - same as Acc but with "gyro"
   * 5.Distance sensors BACK - "DISTB" + "LEFT/MID/RIGHT" + level (G / Y / R)
   * 6.Distance sensors SIDES - "DISTS" + "LEFT/RIGHT" + level (G / Y / R)
   * 7.Distance sensor FRONT - "DISTF" + level + frontServoPos 
   */
}


void remoteControl(){
  if(BT_Data.charAt(0) == 'T'){
    if(BT_Data.charAt(1) == 'D'){
      currentFunction = "DRIVE"; //SWITCH TO DRIVE MODE
      Serial.println(currentFunction);
    }
    if(BT_Data.charAt(1) == 'N'){
      currentFunction = "NEUTRAL"; //SWITCH TO NEUTRAL MODE
      Serial.println(currentFunction);
      tone(BUZZER, 200, 200*backSensorsDistanceLevel);
       
    }
    if(BT_Data.charAt(1) == 'R'){
      currentFunction = "REVERSE"; //SWITCH TO REVERSE MODE
      Serial.println(currentFunction);
    }
  }
  if(BT_Data.charAt(0) == 'S'){
    if(BT_Data.charAt(1) == '1'){
      driveSpeed = 80;
      speedLevel = "LOW"; // SPEED LEVEL = LOW
    }
    if(BT_Data.charAt(1) == '2'){
      driveSpeed = 175;
      speedLevel = "MID"; // SPEED LEVEL = MID
    }
    if(BT_Data.charAt(1) == '3'){
      driveSpeed = 255;
      speedLevel = "HIGH"; // SPEED LEVEL = HIGH
    }
  }
  
}

