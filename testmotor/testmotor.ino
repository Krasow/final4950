#define OFF 0x00
#define ON  0xFF

// for the DC motor
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;


const short int solenoid  = 8;


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);        

  pinMode(solenoid, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {  
  // waits for new serial comm
  while (!Serial.available());
  float motor_change  = Serial.parseFloat();
  float actuator_change = Serial.parseFloat();
  // Serial.Read will read a byte. We just want a bit
  bool  solendoid_sig =  bool(Serial.Read());

  Serial.println(motor_change);
  Serial.println(actuator_change);
  Serial.println(solendoid_sig);



  if (motor_change) {
    // set target position
    int target = motor_change;
    // PID constants
    float kp = 1.2;
    float kd = 0.3;
    float ki = 0.001;


    while(1) {
      // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
      prevT = currT;

      // error
      int e = pos - target;
      // if the errors hasn't changed, break out of loop
      if (e == e_prev) break;

      // derivative
      float dedt = (e - eprev) / (deltaT);

      // integral
      eintegral = eintegral + e * deltaT;

      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor power
      float pwr = fabs(u);
      if ( pwr > 255 ) {
        pwr = 255;
      }

      // motor direction
      int dir = 1;
      if (u < 0) {
        dir = -1;
      }

      // signal the motor
      setMotor(dir, pwr, PWM, IN1, IN2);

      // store previous error
      eprev = e;
    }
    
    //motor finished moving reset
    delay(3);
    motor_change = 0;

  }

  if (actuator_change)
  {
    // not sure what type of motor yet so can't code it yet
    actuator_change = 0;
  }


  if (solendoid_sig) {
    digitalWrite(solenoid, HIGH)
    delay(3);
    digitalWrite(solenoid, LOW)
    solendoid_sig = 0;
  }

}




void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  }
  else {
    pos--;
  }
}
