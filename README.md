# All-the-codes
Roborregos

###Seguidor de línea

#include <AFMotor.h>

AF_DCMotor motorD1(3);
AF_DCMotor motorD2(2);
AF_DCMotor motorI1(4);
AF_DCMotor motorI2(1);

int values[] = {A0, A1, A2, A3, A4, A5};
int THRESHOLD = 800;


void setup() {
  Serial.begin(9600);
for (int i =0; i<6; i++){
  pinMode(A0+i,INPUT);
}

}

void loop() {
  int max = 0;
  int line;
  int error;
  int I;
  int D;
  int P;
  float KP = 0.7;
  float KI = 0.3;
  float KD = 0.2;
  float PID;
  int previouserror;
  int speedD;
  int speedI;
  int PIDD;
  int PIDI;

for (int i = 0; i<6; i++){
  values[i] = analogRead(A0+i);
  if (values[i]>max){
    max = values[i];
    line = i+1;
  }
}
if((line)==1){
  error = -2;
}
if((line)==2){
  error = -1;
}
if((line)==3){
  error = 0;
}
if((line)==4){
  error = 0;
}
if((line)==5){
  error = 1;
}
if((line)==6){
  error = 2;
}
I = error + I;
D = error-previouserror;
P = error;
PID = map((KP*P)+(KI*I)+(KD*D),0,2.4,0,255);
previouserror = error;
speedD=255+PID;
speedI=255-PID;

motorD1.setSpeed(speedD);
motorD2.setSpeed(speedD);
motorI1.setSpeed(speedI);
motorI2.setSpeed(speedI);
motorD1.run(FORWARD);
motorD2.run(FORWARD);
motorI1.run(FORWARD);
motorI2.run(FORWARD);
Serial.print("Speed derecha es: ");
Serial.println(speedD);
Serial.print("Speed izquierda es: ");
Serial.print(speedI);
Serial.print("    ");
}


## MAZE

#include <NewPing.h>
#include <AFMotor.h>

#define Trigger_sensor_1
#define Echo_sensor_1

#define Trigger_sensor_2
#define Echo_sensor_2

#define Max 200

NewPing sensor_1(Trigger_sensor_1, Echo_sensor_1, Max);
NewPing sensor_2(Trigger_sensor_2, Echo_sensor_2, Max);

AF_DCMotor motorD1(3);
AF_DCMotor motorD2(2);
AF_DCMotor motorI1(4);
AF_DCMotor motorI2(1);

void setup(){
  Serial.begin(9600);
  Serial.write(12);
}
void loop(){
  delay(2000);
  Serial.print("Distancia enfrente: ");
  Serial.print(sensor_1.ping_cm());
  Serial.println("cm");

  delay(2000);
  Serial.print("Distancia izquierda: ");
  Serial.print(sensor_2.ping_cm());
  Serial.println("cm");

 if sensor_1.pingcm()>2 and sensor_2.pin_cm()<=4 {
  motorD1.run(FORWARD);
  motorD2.run(FORWARD);
  motorI1.run(FORWARD);
  motorI2.run(FORWARD);
 }

}
