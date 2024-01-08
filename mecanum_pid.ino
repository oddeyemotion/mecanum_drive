#include <util/atomic.h>

// Pins
#define ENCA_L A5
#define ENCA_R A4
#define PWM_A 9
#define IN1_A 8

#define ENCB_L A3
#define ENCB_R A2
#define PWM_B 6
#define IN1_B 7

#define ENCC_L A1
#define ENCC_R A0
#define PWM_C 5
#define IN1_C 4

#define ENCD_L 16
#define ENCD_R 17
#define PWM_D 3
#define IN1_D 2


// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA_L,INPUT);
  pinMode(ENCA_R,INPUT);
  pinMode(PWM_A,OUTPUT);
  pinMode(IN1_A,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_L),readEncoder,RISING);

  pinMode(ENCB_L,INPUT);
  pinMode(ENCB_R,INPUT);
  pinMode(PWM_B,OUTPUT);
  pinMode(IN1_B,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCB_L),readEncoder,RISING);

  pinMode(ENCC_L,INPUT);
  pinMode(ENCC_R,INPUT);
  pinMode(PWM_C,OUTPUT);
  pinMode(IN1_C,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCC_L),readEncoder,RISING);

  pinMode(ENCD_L,INPUT);
  pinMode(ENCD_R,INPUT);
  pinMode(PWM_D,OUTPUT);
  pinMode(IN1_D,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCD_L),readEncoder,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 100*(sin(currT/1e6)>0);

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
uint8_t pwr = (int) fabs(u);
  /*if(pwr > 255){
    pwr = 255;
  }*/
  setMotor(dir,pwr,PWM_A,IN1_A);
  setMotor(dir,pwr,PWM_B,IN1_B);
  setMotor(dir,pwr,PWM_C,IN1_C);
  setMotor(dir,pwr,PWM_D,IN1_D);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,LOW);
    //digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,HIGH);
    //digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,HIGH);
    //digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCA_R);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}