////////////////////////////////////////////////////////////////////////////
//
//  Original Authors: Tom Libby, Evan Chang-Siu
//  Updated: Matt Estrada 4/28/2015
//  Control and initialization for two motors
//  Motor 1: Climbing rack/pinion
//            Sensor feedback given from whisker switches
//  Motor 2: Intertial tail
//            Sensor feedback given from motor encoder
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////

// VARIABLE DECLARATIONS
////////////////////////////////////////////////////////////////////////////

//CONTROL
double error_tail = 0;
double error_body = 0;
double th_des = -90;
int tail_angle_des = 512; //XBEE
int pwm_wheel = 0;
int pwm_wheel_max = 200; //maximum wheel pwm
//int k_pt = -4; // p gain for relative tail control
//int k_g = -45; // grav comp gain

unsigned long time_wheelstart = 0; //initialized in WAITBEGIN
unsigned long timer_loop = 0; //
unsigned long dt_m = 0; // measured dt
unsigned long dt_l = 8; // measured dt
int time_final = 2500;

int th_des_switch = 90; // value for th_des after switch
//int th_des_time_start = 1275; // time when th_des switches

// Pinout Definitions
#define MOTOR3DIR 7          // Motor 3 = Climbing rack/pinion 7/6
#define MOTOR3PWM 6
#define MOTOR4DIR 8          // Motor 4 = Tail motor 8/9
#define MOTOR4PWM 9
#define MOTORSLEEPPIN A3     // Motor sleep to analog pin 3
#define encoder4PinA  3      // 'Encoder4' attached to Motor 4, external interrupt 1
#define encoder4PinB  4      // Direction 

#define MOTORPOT     A1    // Rotary potentiometer

// Encoder constants
const int gearRatio = 150;    // between output shaft and motor shaft
                              // 298 or 150
const int encoderTicks = 6; // ticks per motor shaft revolution
const float ticksPerDeg = gearRatio * encoderTicks / 360;
float tailPosition = 0;
volatile signed int encoder4Pos = 0;

// Tail Parameters
int slowSweep = 80; // PWM speed for tail motor 
int startingTailAngle = 90; 

// Potentiometer variables
int analogReading; 
int analogReadingPrev = 0; 


// Safety boundaries
const double MAX_TAIL_ANGLE = 115;
const double MIN_TAIL_ANGLE = -100;

const double MAX_POT_ANGLE = 140; //137 absolute max
const double MIN_POT_ANGLE = 835;//843 absolute min
const double MID_POT_ANGLE = 420;


////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
///////////////////////////////////////////////////////////////////////////

// init_mot()
// Initialize outputs for motors
void init_mot() {
  // Motors
  pinMode(MOTOR3DIR, OUTPUT); pinMode(MOTOR3PWM, OUTPUT); // Motor 1
  pinMode(MOTOR4DIR, OUTPUT); pinMode(MOTOR4PWM, OUTPUT); // Motor 2
  // Motor Driver sleep pin
  digitalWrite(MOTORSLEEPPIN  , HIGH);                     // sets the sleep mode to be off

  // Motor Encoder for tail
//  pinMode(encoder4PinA, INPUT);
//  digitalWrite(encoder4PinA, HIGH);       // turn on pullup resistor
//  pinMode(encoder4PinB, INPUT);
//  digitalWrite(encoder4PinB, HIGH);       // turn on pullup resistor
//  attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2


  // Tail potentiometer
  pinMode(MOTORPOT, INPUT);
  tail_pos_pot();  // read in pot to get initial value

  // Initialize Switches
  pinMode(tailUpSwitch, INPUT);
  pinMode(tailDownSwitch, INPUT);

}

// write_tail()
// Takes in int, deduces direction, writes motor appropriately
// analogWrite takes duty cycle betwee 0 (always off) and 255 (always on)
void write_tail(int PWM_VALt) {
  //PWM_VALt=map(PWM_VALt,-32,768, 32767,-255,255);
  //PWM_VALt=constrain(PWM_VALt,-255,255);

  if (PWM_VALt < 0) {
    digitalWrite(MOTOR4DIR, HIGH);
    analogWrite(MOTOR4PWM, PWM_VALt);
  }
  if (PWM_VALt > 0) {
    digitalWrite(MOTOR4DIR, LOW);
    analogWrite(MOTOR4PWM, PWM_VALt); // Took a negative out here
  }
  if (PWM_VALt == 0) {
    digitalWrite(MOTOR4DIR, LOW);
    analogWrite(MOTOR4PWM, 0);
  }
}


// calc_control_tail()
// inputs: tail_angle, tail_angle_des
// outputs: pwm_tail
void calc_control_tail() {

  // ABSOLUTE BODY CONTROL
  error_body = th_des - th_sf;
  //Serial.print("Body Angle: "); Serial.println(th_sf);
  //Serial.print("Body Error: "); Serial.println(error_body);

  pwm_tail = -double(k_pb) * error_body - double(k_db) * (0 - w_gyro); // control law, P control and Feedforward,
  pwm_tail = constrain(pwm_tail, -32768, 32767);

  // Safety limits
  //  if (tail_angle<75 ) {
  //    pwm_tail = constrain(pwm_tail,0,32767); // allows pwm_tail to only be positive, move down when up
  //  }
  //  if(tail_angle>315){
  //    pwm_tail = constrain(pwm_tail,-32768,0); // allows pwm_tail to only be positive, move up when down
  //  }

  pwm_tail = map(pwm_tail, -32768, 32767, -255, 255);

}

void calc_tune_pid(double tail_des) {

  // ABSOLUTE BODY CONTROL
  error_tail = tail_des - tail_angle;
  //Serial.print("Body Angle: "); Serial.println(th_sf);
  //Serial.print("Tail Error: "); Serial.println(error_tail);

  pwm_pid_tune = -double(k_pb) * error_tail - double(k_db) * (0 - tail_vel); // control law, P control and Feedforward,
  double proportional = -double(k_pb) * error_tail;
  double differential = - double(k_db) * (0 - tail_vel);
  //Serial.print("Proportional: "); Serial.println(proportional); 
  //Serial.print("Differential: "); Serial.println(differential); 
  pwm_pid_tune = constrain(pwm_pid_tune, -32768, 32767);

  // Safety limits
  //  if (tail_angle<75 ) {
  //    pwm_tail = constrain(pwm_tail,0,32767); // allows pwm_tail to only be positive, move down when up
  //  }
  //  if(tail_angle>315){
  //    pwm_tail = constrain(pwm_tail,-32768,0); // allows pwm_tail to only be positive, move up when down
  //  }

  pwm_pid_tune = map(pwm_pid_tune, -32768, 32767, -255, 255);

}

//void tail_pos_encoder() {
//
//  now = millis();
//  tail_angle = encoder4Pos / ticksPerDeg; //*float(360/(gearRatio*encoderTicks)); //degrees
//  tail_vel = 1000*(tail_angle-last_tail_angle)/(now-last_t)/57; //  [rad/sec]
//  
//  last_t = now;
//  last_tail_angle = tail_angle;
//  //Serial.print("Tail Position: "); Serial.println(tailPosition);
//  //Serial.print("Encoder count: "); Serial.println(encoder4Pos);
//
//}

//// calibrate absolute position of tail against bump switch when using encoder
//void calibrate_tail() {
//
//  write_tail(-slowSweep);
//  while (digitalRead(tailUpSwitch) == LOW) {
//    //Serial.print("Tail Up Switch: "); Serial.println(digitalRead(tailUpSwitch));
//    //Serial.println("Waiting for tail calibration");
//  }
//  write_tail(0);
//  encoder4Pos = MAX_TAIL_ANGLE*ticksPerDeg;
//  move_tail(0);
//}

void move_tail(double angle) {
  double err = 10;

  // Accuracy in degrees
  while (abs(err) > 2) {
    tail_pos_pot();
    err = tail_angle - angle;
//    Serial.print("Tail encoder: "); Serial.println(encoder4Pos);
    Serial.print("Tail position: "); Serial.println(tail_angle);
    Serial.print("Tail error: "); Serial.println(err);
    write_tail(slowSweep*( (err>0)-(err<0) ) );
  }
  write_tail(0);
}

// tail_limits()
// Checks pot analog reading to make sure it hasn't reached any values outside desired range
bool tail_limits_pot(){
  if (analogReading <= MAX_POT_ANGLE || analogReading >= MIN_POT_ANGLE){
    write_tail(0);
    if(analogReading <= MAX_POT_ANGLE){
      TAILUP = 1;
    }
    if(analogReading >= MIN_POT_ANGLE){
      TAILDOWN = 1;  
    }
    return true; 
  }
 return false; 
}

//// tail_limits_switches()
//// Checks the tail limits to make sure it hasn't reached any bump switches
//bool tail_limits_switch(){
//  if (digitalRead(tailUpSwitch) || digitalRead(tailDownSwitch)){
//    write_tail(0);
//    if(digitalRead(tailUpSwitch)){
//      TAILUP = 1;
//    }
//    if(digitalRead(tailDownSwitch)){
//      TAILDOWN = 1;  
//    }
//    return true; 
//  }
// return false; 
//}

//// doEncoder()
//// Interrupt for handling tail encoder
//void doEncoder() {
//  //Serial.println("Mototr Interrupt");
//  if (digitalRead(encoder4PinA) == digitalRead(encoder4PinB)) {
//    encoder4Pos--;
//  } else {
//    encoder4Pos++;
//  }
//}

void tail_pos_pot() {
  //float degPerAnalog = 90deg /(MID_POT_ANGLE-MAX_POT_ANGLE);
  float degPerAnalog = -.31; 
  analogReading = analogRead(MOTORPOT);
  //Serial.print("Analog read: "); Serial.println(analogReading);

  // Make sure we read in a valid, first value
//  while(analogReading == 0){
//    //Serial.println("Bad pot");
//    analogReading = analogRead(MOTORPOT);
//  }
  // Catching errors since pot sometimes give erroneous values
  if( analogReading < (MAX_POT_ANGLE -50) || analogReading > (MIN_POT_ANGLE +50) ){ 
    analogReading = analogReadingPrev;
  }
  else{
    analogReadingPrev = analogReading;
  }
  
  // Convert to angle
  tail_angle = double(degPerAnalog*(analogReading-MID_POT_ANGLE)); //*float(360/(gearRatio*encoderTicks)); //degrees
  tail_vel = 1000*(tail_angle-last_tail_angle)/(now-last_t)/57; //  [rad/sec]
  
  last_t = now;
  last_tail_angle = tail_angle;
  //Serial.print("Tail Position: "); Serial.println(tail_angle);

}
