#include<PID_v1.h>

//Settings
double Kp = 1, Ki = 0.05, Kd = 0.01;
// propoprtional gain, differential gain and integral gain for PID

int LturnTime = 500;
int RturnTime = 560;
int UturnTime = 1050;
int FATTime =575;
int FBRTTime = 960;
int FBLTTime =790;
// FORWARD AFTER TURN TIME
int goFowdTime = 25;
int lExtspeed = 10;

double threshold = 60; //side threshold value for deciding turn
double fThreshold = 40;  //forward threshold value for deciding turn
int outLowLim = 100;
int outHigLim = 225;


int rled = 2;
int lled = 8;
int lTrig = A4;
int lEcho = A5;
int fTrig = A2;
int fEcho = A3;
int rTrig = A0;
int rEcho = A1;

const int lFowd = 6; //Left Forward pwm
const int lBack = 3;  //left backward PWM
const int rBack = 10;  //Right Backward PWM
const int rFowd = 5; //Right Forward pwm
const int En_A = 9;
const int En_B = 11;

bool hascounteddist = false;


void readSensors();
void lTurn();
void rTurn();
void stopAll();
void stabilize();
void goFowd();
void decide();
void uTurn();
void Print();
void lledblink();
void rledblink();


double lDist, rDist, fDist;
double mean;

double lms = 150, rms = 150;
long lastPrintTime = 0;
int printInterval = 500;

PID lPid(&lDist, &lms, &mean, Kp, Ki, Kd, DIRECT);
PID rPid(&rDist, &rms, &mean, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(En_A, OUTPUT);
  pinMode(En_B, OUTPUT);
  pinMode(rBack, OUTPUT);
  pinMode(rFowd, OUTPUT);
  pinMode(lBack, OUTPUT);
  pinMode(lFowd, OUTPUT);

  pinMode(lTrig, OUTPUT);
  pinMode(rTrig, OUTPUT);
  pinMode(fTrig, OUTPUT);
  pinMode(lEcho, INPUT);
  pinMode(rEcho, INPUT);
  pinMode(fEcho, INPUT);

  lPid.SetMode(AUTOMATIC);
  rPid.SetMode(AUTOMATIC);
  lPid.SetOutputLimits(outLowLim, outHigLim);
  rPid.SetOutputLimits(outLowLim, outHigLim);
  analogWrite(En_A, 180);
  analogWrite(En_B, 180);
  pinMode(rled, OUTPUT);
  pinMode(lled, OUTPUT);

}



void loop() {

  readSensors();  // used if condition for serial print taking delay between consecutive serial output such that loop wont get any delay
  if (millis() - lastPrintTime >= printInterval) {
    Print();
    lastPrintTime = millis();}
  
  stabilize();
  if (hascounteddist == false){  //to decide fthreshold and side threshold value by calculating size of maze
      fThreshold = lDist + rDist + 10;
      threshold = mean*4;
      
      hascounteddist = true;  // will run initially once   
      }
  decide();
}

void readSensors() {

  lDist = getDistance(lTrig, lEcho);
  fDist = getDistance(fTrig, fEcho);
  rDist = getDistance(rTrig, rEcho);
}

double getDistance(int triggerPin, int echoPin) {
  
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  double distance = duration * 0.034 / 2;

  return distance;
}


void lTurn() {
  
  digitalWrite(lFowd, LOW);
  digitalWrite(lBack, HIGH);
  digitalWrite(rFowd, HIGH);
  digitalWrite(rBack, LOW);
  delay(LturnTime);
}

void rTurn() {
  
  digitalWrite(rFowd, LOW);
  digitalWrite(rBack, HIGH);
  digitalWrite(lFowd, HIGH);
  digitalWrite(lBack, LOW);
  delay(RturnTime);
}

void uTurn() {
  
  digitalWrite(rFowd, LOW);
  digitalWrite(rBack, HIGH);
  digitalWrite(lFowd, HIGH);
  digitalWrite(lBack, LOW);
  delay(UturnTime);
}


void stabilize() {

  mean = (lDist + rDist)/2;
  lPid.Compute();
  rPid.Compute();

}
void goFowd() {

  analogWrite(lFowd, lms + lExtspeed);
  digitalWrite(lBack, LOW);
  digitalWrite(rBack, LOW);
  analogWrite(rFowd, rms);
  delay(goFowdTime);
}

void FAT() { //to go some steps forward after turn
  analogWrite(lFowd, 150);
  digitalWrite(lBack, LOW);
  digitalWrite(rBack, LOW);
  analogWrite(rFowd, 150);
  delay(FATTime);
  
  }

void FBT() { //to go forward untill forward dist is 13 cm in case of turning
  analogWrite(lFowd, 180);
  digitalWrite(lBack, LOW);
  digitalWrite(rBack, LOW);
  analogWrite(rFowd, 180);
  
  
  }
  
void decide() {

  if (rDist > threshold){
      FBT();
        delay(FBRTTime);
      rTurn();
      FAT();
    }

 
  else if (fDist > fThreshold){
    if (fDist > fThreshold + 7){
    goFowd();
  }
    else {
      FBT();
    }
  }
  
  else if (lDist > threshold){
      FBT();
      delay(FBLTTime);
      lTurn();
      FAT();
    }
 

  else
  {
  uTurn();
   // taking uturn
}
}
void Print() {
  Serial.print("Leftdistance = ");
  Serial.println(lDist);
  Serial.print("Rightdistance = ");
  Serial.println(rDist);
  Serial.print("Frontdistance = ");
  Serial.println(fDist);
  Serial.print("Mean = ");
  Serial.println(mean);
  Serial.print("Lms = ");
  Serial.println(lms);
  Serial.print("Rms = ");
  Serial.println(rms);
  Serial.print("thershold = ");
  Serial.println(threshold);
  Serial.print("fThershold = ");
  Serial.println(fThreshold);
  Serial.print("\n");
}
