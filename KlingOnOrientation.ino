////////////////////////////////////////////////////////////////////////////
//
//  KlingOn orientation

#include <EEPROM.h>
#include <Wire.h>
#include<MPU9150.h>
//#include "I2Cdev.h"
//#include "RTIMUSettings.h"
//#include "RTIMU.h"
//#include "CalLib.h"


RTIMU *imu;                                           // the IMU object
//RTFusionRTQF fusion;                                // the fusion object (TVCF used instead)
RTIMUSettings settings;                               // the settings object

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  9600
// TinyDuino Bluetooth can go up to 57600

unsigned long lastDisplay;
unsigned long lastRate;
int led1 = 12;

/////////////////////////////////////////////////////////////
// Global Variables
/////////////////////////////////////////////////////////////

float pi = 3.14159;

//SENSORFUSION
double dt = 0.01; //approximate time loop, used to be .008
double th_acc = 0;
double th_accp = 0;
double y_gyro = 0;
double y_gyrop = 0;
double y_acc = 0;
double y_accp = 0;
double wc = 2;
double w_gyro = 0;
double w_gyrop = 0;
double th_sf = 0;
double g_accx = 0;
double g_accy = 0;
double g_accz = 0;

//TVCF in SENSORFUSION
double s1 = 41;
double s2 = 0.18;
double s3 = 0.62;
double s4 =  0.06;
double xo1 = 0.05; //gamma = 3
double xo2 = 11.7;
double xo3 = 3.3;
double xo4 = 40;
double xb1 = 0;
double xb2 = 0;
double xb3 = 0;
double xb4 = 0;
double accmag = 0;
double accmag3D = 0;
double ddtg_accx = 0;
double ddtg_accy = 0;
double ddtaccmag = 0;
double mu = 1;
double whigh = 20;
double wlow = 0.1;
double g_accxp = 0;
double g_accyp = 0;
double ddtw_gyro = 0;


//TAIL_POT
double tail_angle;
double tail_vel;
double last_tail_angle = 0;
double last_t = 0;

// CONTROL
double pwm_tail = 0;
double pwm_pid_tune = 0;
int k_pb = 3 * 128; // p gain for absolute body control
int k_db = 20 ; //2 * 128;//-4*128*57; // d gain for absolute body control
signed int serialInt = 0;

// Tail switch flags
bool TAILUP = 0;
bool TAILDOWN = 0;
#define tailUpSwitch 13
#define tailDownSwitch 11


//Data collection
#define MAX_SAMPLES 25
float xAccel[MAX_SAMPLES];
float yAccel[MAX_SAMPLES];
unsigned long time[MAX_SAMPLES];
float magAccel[MAX_SAMPLES];
int buttonRelease[MAX_SAMPLES];
float pwmRecord[MAX_SAMPLES];
float wRecord[MAX_SAMPLES];
float angleRecord[MAX_SAMPLES];
float muRecord[MAX_SAMPLES];
float yGyroRecord[MAX_SAMPLES];
float yAccRecord[MAX_SAMPLES];



unsigned long now;
int entry = 0;


// State Machine
#define CALIBRATION 1
#define PRELAUNCH   2
#define BALLISTIC   3
#define COMPLETED   4
#define TUNE_PID    5
#define ACCEL_TROUBLESHOOT 6
char state = 0;
int sampleCount = 0;
int lastNow = 0;      //For debugging time delays



/////////////////////////////////////////////////////////////
// Main Functions
/////////////////////////////////////////////////////////////

void setup()
{
  int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  // Standard Initialization Prinouts
  Serial.println("KlingOn orientation code for active tail control");
  Serial.println("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  // Initialize Outputs
  init_mot();
  pinMode(led1, OUTPUT);
  digitalWrite(led1, LOW);

  state = CALIBRATION;

}

void loop()
{
  now = millis();

  //  if(imu->IMURead()){
  //    sensorfusion();
  //  }
  //  else{
  //    Serial.print("Messed up at: "); Serial.println(now);
  //  }

  //Check each round for tail limits being hit
  tail_pos_pot();
  if (tail_limits_pot()) {
    if ( TAILUP && TAILDOWN) {
      state = PRELAUNCH;
      Serial.println("Manual Reset");
      TAILUP = 0;
      TAILDOWN = 0;
      entry = 0;
      digitalWrite(led1, HIGH);
      delay(2000);
      digitalWrite(led1, LOW);
    }
    else {
      //Serial.println("Tail limit hit!");
      if (TAILUP) {
        Serial.println("Tail up");
      }
      if (TAILDOWN) {
        Serial.println("Tail down");
      }
      state = COMPLETED;
      TAILUP = 0;
      TAILDOWN = 0;
    }
  }


  switch (state) {

      // Calibration state waits for IMU to calculate offset
    case CALIBRATION:
      if (imu->IMUGyroBiasValid()) {
        Serial.println("Gyro bias valid!");
        //Serial.println("Calibrating Tail");
        //calibrate_tail();
        state = PRELAUNCH;
        digitalWrite(led1, HIGH);
        delay(500);
        digitalWrite(led1, LOW);
      }
      else {
        imu->IMURead();
      }
      break;

      // Used to find appropriate values for the PID controller by doing control on tail position
    case TUNE_PID:
      sensorfusion();

      tail_pos_pot();
      sensorfusion();
      calc_tune_pid(45);
      //Serial.print("Control PWM: "); Serial.println(pwm_pid_tune);
      //Serial.print("Angular Vel: "); Serial.println(tail_vel);

      write_tail(int(pwm_pid_tune));

      //Serial.print("Prelaunch accmag: "); Serial.println(accmag);
      break;

    case ACCEL_TROUBLESHOOT:
      {
        log_data();

        sampleCount++;

        if (sampleCount > MAX_SAMPLES) {
          dump_data();
          state = COMPLETED;
        }
      }
      break;

      // PRELAUNCH STATE is when the robot is readu to be launched
    case PRELAUNCH:
      if (imu->IMURead() ) {
        sensorfusion();
        //log_data();

        if (Serial.available()) {
          serialInt = Serial.parseInt();
          if (serialInt > 0) {
            k_pb = serialInt;
            Serial.print("New k_PRO: "); Serial.println(k_pb);
          }
          else if (serialInt < 0) {
            k_db = -serialInt;
            Serial.print("New k_DIF: "); Serial.println(k_db);
          }

          //         else{
          //            k_db =0;
          //            k_pb = 0;
          //            Serial.println("BOTH GAINS SET TO ZERO");
          //          }

        }
        //Serial.print("Button: "); Serial.println(digitalRead(tailUpSwitch));

        // Transition: linear accel drops in freeflight
        if (accmag3D < 0.7) {
          // dump_data();
          digitalWrite(led1, HIGH);

          state = BALLISTIC;
        }
      }
      //Serial.print("Prelaunch accmag: "); Serial.println(accmag);
      break;

      // BALLISTIC STATE: Implementing active tail control
    case BALLISTIC:
      if (imu->IMURead() ) {

        sensorfusion();
        calc_control_tail();
        log_data_front();

        //Serial.print("Attitude: "); Serial.println(th_sf);
        //print_everything();
        //Serial.print("Control PWM: "); Serial.println(pwm_tail);

        write_tail(int(-pwm_tail));
        //      //Serial.print("Ballistic accmag: "); Serial.println(accmag);


        // Transition: Accel spike
        if (accmag3D > 4) {
          digitalWrite(led1, LOW);
          write_tail(0); // stop the motor
          state = COMPLETED;
          dump_data_front();
        }
      }
      //Serial.print("Ballistic accmag: "); Serial.println(accmag);
      break;

      // COMPLETED STATE: Everything is done and we're through
    case COMPLETED:
      digitalWrite(led1, HIGH);
      delay(500);
      digitalWrite(led1, LOW);
      delay(500);


      break;

      // UNDEFINED STATE: An attempt to catch errors in the state machine...
    default:
      Serial.println("UNDEFINED STATE SOMETHING WENT WRONG");
      state = COMPLETED;
  }

}



