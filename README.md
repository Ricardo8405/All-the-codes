#include<AFMotor.h>

AF_DCMotor Motor1D(3);
AF_DCMotor Motor2D(2);
AF_DCMotor Motor1I(4);
AF_DCMotor Motor2I(1);

const int sensorPins[8] = {A8, A9, A10, A11, A12, A13, A14, A15};
int sensorValues[8];
const int led = 31;

const int THRESHOLD = 930;

double Kp = 0.72;
double Ki = 0.3; 
double Kd = 0.24;

int motorSpeed = 70; 

int lastError = 0;
long integral = 0;
bool isBlackLine = true; 

void setup() {
 Serial.begin(9600);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW); 

  Motor1D.setSpeed(0);
  Motor2D.setSpeed(0);
  Motor1I.setSpeed(0);
  Motor2I.setSpeed(0);
  
  Motor1D.run(RELEASE);
  Motor2D.run(RELEASE);
  Motor1I.run(RELEASE);
  Motor2I.run(RELEASE);

  Serial.println("Iniciando PID (Logica reparada)...");
  delay(1000); 
  digitalWrite(led, HIGH);
}

void loop() {
  
  int OverThreshold = 0;
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    if (sensorValues[i] > THRESHOLD) {
      OverThreshold++;
    }
  }

  if (OverThreshold >= 6) { 
    isBlackLine = false; // Modo: Línea BLANCA
  }
  else if (OverThreshold <= 3) { 
    isBlackLine = true; // Modo: Línea NEGRA
  }

  long sum = 0;
  int sensorsOnLine = 0;
  int position = 3500;

  for (int i = 0; i < 8; i++) {
    bool onLine = (isBlackLine) ? (sensorValues[i] > THRESHOLD) : (sensorValues[i] < THRESHOLD);
    if (onLine) {
      sum += (long)i * 1000;
      sensorsOnLine++;
    }
  }

  if (sensorsOnLine > 0) {
    position = sum / sensorsOnLine;
  } else {
    if (lastError < -2000) { position = 0; }
    else if (lastError > 2000) { position = 7000; }
  }
  
  float error = position - 3500;
  
  float P = error;
  
  integral = integral + error;
  
  float D = error - lastError;
  lastError = error;

  double PID = (Kp * P) + (Ki * integral) + (Kd * D);


  float speedI = motorSpeed - PID;
  float speedD = motorSpeed + PID;

  speedD = constrain(speedD, 0, 90);
  speedI = constrain(speedI, 0, 90);

  Motor1D.setSpeed(speedD);
  Motor2D.setSpeed(speedD);
  Motor1I.setSpeed(speedI);
  Motor2I.setSpeed(speedI);

  Motor1D.run(FORWARD);
  Motor2D.run(FORWARD);
  Motor1I.run(FORWARD);
  Motor2I.run(FORWARD);





jfEWONFOIWNVOISNVOSOVDSBVBOSDVIOSNDVIOSNOVDSN
