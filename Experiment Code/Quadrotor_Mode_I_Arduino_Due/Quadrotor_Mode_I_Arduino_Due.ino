// Terms of use
/****************************************************************************/
/* 
 *  The Sofrware is provided "as is", without warranty of any kind, express or
 *  implied, including but not limited to the warranties or merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall the
 *  authors or copyright holders be liable for any claim, damages or other liability,
 *  whether in an action of contract, tort or otherwise, arising from, out of
 *  or in connection with the software or the use of other dealings in the software.
*/
/****************************************************************************/
// Safety note ---- Important!!!
/****************************************************************************/
/*
 * Please remove the propellers and stay away from the motors unless you are 
 * 100% certain of what you are doing.
 */
/****************************************************************************/

/****************************************************************************/
/*
   This project is maintained by Wenchao Lei, email: wcleithinking@gmail.com.
   Mode I: The Single STR Structure
*/
/****************************************************************************/
/*
                    (CW)          |                 (CW)      (CCW)
   QuadP :           0            |   QuadX :         0         1
           (CCW)3   CoM   1(CCW)  |                       CoM
                     2            |                   3         2
                    (CW)          |                 (CCW)      (CW)
*/
/****************************************************************************/
#include "Config.h"
#include <Servo.h>
#include <IMU.h>
#include <SdFat.h>
Servo     motor[4];
IMU       myIMU;
/****************************************************************************/
// States of Vehicle, from Sensors and Filters
float Gyro_measure_old[3]   = {0, 0, 0};
float Gyro_measure_new[3]   = {0, 0, 0};
float Gyro_measure[3]       = {0, 0, 0};
float Gyro_bias_estimate[3] = {0, 0, 0};
float Angle_estimate[3]     = {0, 0, 0};
float Angle_measure[3]      = {0, 0, 0};
float Angle_bias[3]         = {0, 0, 0};
float Angle_desire[3]       = {0, 0, 0};
float Rate_measure[3]       = {0, 0, 0};
float Rate_bias[3]          = {0, 0, 0};
float Rate_desire[3]        = {0, 0, 0};
// Initial Values for STR-Estimator
#ifdef STR_v1
float a1[3] = { -1.55, -1.55, -1.55};
float a2[3] = {  0.55,  0.55,  0.55};
float a3[3] = { -0.18, -0.18, -0.18};
float b0[3] = {0.02, 0.02, 0.02};
float b1[3] = {0.02, 0.02, 0.02};
float b2[3] = {0.02, 0.02, 0.02};
#endif
/****************************************************************************/
// Motor Variables
bool ARM_flag = 0;
uint16_t RC[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_ref[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
int PWM_out[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
// Main Loop Setting
float dt        = 0.01;
int loop_index  = 0;
int time_diff   = 0;
unsigned long time_previous, time_current;
#ifdef STR_v1
int time_STR_diff;
volatile unsigned long time_STR_previous, time_STR_current;
#endif
// Debug
#ifdef DEBUG
volatile unsigned long time_start, time_end;
#ifdef STR_v1
unsigned long time_STR_start, time_STR_end;
#endif
#endif
// Log
#ifdef LOG
unsigned long errorCount = 0;
#endif
/****************************************************************************/
/****************************************************************************/
void setup() {
#ifdef DEBUG
  Serial.begin(BaudRate);
#endif
  Led_init();
  Led_low();
  Motor_init();
  Motor_update();
  Receiver_init();
  myIMU.init(3, 4, 3, 4, 1, dt);
#ifdef LOG
  Log_init();
#endif
  Led_blink(3);
  ARM_flag = 0;
  Led_armstate();
}

void loop() {
  while (ARM_flag == 0) {
    Led_armstate();
    Receiver_copy();
    while (RC[IndexRoll] <= (PWM_MAX - 10) || RC[IndexPitch] <= (PWM_MAX - 10)) {
      ARM_flag = 0;
      Led_armstate();
      Receiver_copy();
    }
    ARM_flag = 1;
    Led_armstate();
#ifdef STR_v1
    time_STR_previous = millis();
#endif
    time_previous = millis();
  }
  time_current = millis();
  if (time_current - time_previous >= dt * 1000 ) {
#ifdef DEBUG
    time_diff = time_current - time_previous;
#endif
    time_previous = time_current;
    myIMU.sample();
    myIMU.copygyro(Gyro_measure_old);
    myIMU.attitude_filter(Angle_estimate, Gyro_bias_estimate);
    Angle_calibrate();
    Receiver_copy();
    Motor_control();
    Motor_update();
    Controller_updatedata();
#ifdef LOG
#ifdef DEBUG
    time_start = millis();
    Log_savedata();
    time_end = millis();
#else
    Log_savedata();
#endif
#endif
    if (loop_index < 350) loop_index++;
  }
#ifdef DEBUG
  Serial_debug();
#endif
}
/****************************************************************************/
/****************************************************************************/
