#ifndef N20_H
#define N20_H

// struct MotorControlParams {
//   volatile bool currentDirection;
//   volatile int timerCounter;
//   volatile int timerDuty;
// };

// External declaration of global instance
// extern MotorControlParams motorParams;
// extern hw_timer_t *timer;
// extern portMUX_TYPE timerMux;

// Function declarations
//void ARDUINO_ISR_ATTR onTimer();
void setupN20();
void motorControl(bool dir, int speed);
void printEncoderCount();
void autotunePID();
void setSpeeds();
void populateStoppingDistances();
long estimateStoppingDistance(int speed);

#endif
