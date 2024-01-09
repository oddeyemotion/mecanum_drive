#include <util/atomic.h> 
#include <math.h>
#include <Encoder.h>

/*------------ CLASS ------------*/
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public:
    // Default initialization list
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,uint8_t &pwr, uint8_t &dir){
        
      // error
      int e = target - value;
      
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power
      pwr = (int) fabs(u);
      /*if( pwr > umax ){
        pwr = umax;
      }*/
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
            
      // store previous error
      eprev = e;
    }
    
};

/*------------ GLOBALS AND DEFINITIONS ------------*/
// Define the motors
#define NMOTORS 4
#define M0 0
#define M1 1
#define M2 2
#define M3 3
const int enca[] = {A1,16,A4,A3};
const int encb[]= {A0,17,A5,A2};
const int pwm[] = {5,3,9,6};
const int in1[] = {4,2,8,7};

// Global variables
long prevT = 0;
int posPrev[] = {0,0};

Encoder myEnc(A0, A1);

// positions
volatile int posi[] = {0,0};
volatile long pos[] = {0,0};

// PID classes
SimplePID pid[NMOTORS];


/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
  }
  else{
    analogWrite(pwm,0);
  }  
}
long oldPosition  = 0;
template <int j>
void readEncoder(){
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
}

// targets
float target_f[] = {0,0,0,0};
int target[] = {0,0,0,0};

void setTarget(float t, float deltat){

  float positionChange[4] = {0.0,0.0,0.0,0.0};
  float pulsesPerTurn = 16*18.75; 
  float pulsesPerMeter = pulsesPerTurn*3.5368;

  t = fmod(t,12); // time is in seconds
  float velocity = 0.25; // m/s

  if(t < 4){
  }
  else if(t < 8){
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = velocity*deltat*pulsesPerMeter;
    }
  }
  else{
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = -velocity*deltat*pulsesPerMeter; 
    } 
  }  

  for(int k = 0; k < 4; k++){
    target_f[k] = target_f[k] + positionChange[k];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
  target[2] = (long) -target_f[2];
  target[3] = (long) -target_f[3];
}

/*------------ SETUP ------------*/
void setup() {
  Serial.begin(9600);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pid[k].setParams(1,0.1,0,255);
  }
  attachInterrupt(digitalPinToInterrupt(enca[M0]),readEncoder<M0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]),readEncoder<M1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M2]),readEncoder<M2>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M3]),readEncoder<M3>,RISING);
}

/*------------ LOOP ------------*/
void loop() {

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
    
  // set target position
  setTarget(currT/1.0e6,deltaT);
  
  // Read the position in an atomic block to avoid a potential misread 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    uint8_t pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,pwm[k],in1[k]); // signal the motor
  }
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    }
  Serial.println(newPosition);
  /*for(int i = 0; i<4; i++){
    Serial.print(target[i]);
    Serial.print(" ");
  }
  for(int i = 0; i<4; i++){
    Serial.print(pos[i]);
    Serial.print(" ");
  }
  Serial.println();*/
  
}