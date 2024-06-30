#include "n20.h"
#include "oled.h"
#include <ESP32Encoder.h>
#include <driver/ledc.h>

// Constants for motor control
#define CLK 32 // VP - Orange/Yellow - ?CLK ENCODER? 
#define DT 33 // VN - Green - ?DT ENCODER? 
#define MOTOR_IN1 19 // Clockwise
#define MOTOR_IN2 18 // CounterClockwise
#define n20min 90    // Min PWM speed
#define n20max 255   // Max PWM speed
#define rotTarget 14316 // Target encoder count
ESP32Encoder encoder;

const int freq = 5000;
const int M1Channel = 0;
const int M2Channel = 1;
const int resolution = 8;

// Slower motor speeds
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

struct MotorControlParams {
  volatile bool currentDirection;
  volatile int timerCounter;
  volatile int timerDuty;
  volatile bool running;
  volatile int motorSpeedCmd;
};

MotorControlParams motorParams = {true, 100, 0, false};

// PID variables
double Kp = 0.1; // Proportional gain
double Ki = 0.01; // Integral gain
double Kd = 0.0; // Derivative gain
double integral = 0; // Integral term
double last_error = 0; // Last error for derivative term

// Autotuning variables
double best_error = 999999; // Initialize with a large value
double current_error = 0;
double best_Kp, best_Ki, best_Kd; // Store the best PID constants found

// Timer interrupt service routine
void ARDUINO_ISR_ATTR onTimer() {
  // Increment the duty timer
  motorParams.timerCounter++;
  portENTER_CRITICAL_ISR(&timerMux);
  int direction = motorParams.currentDirection ? MOTOR_IN1 : MOTOR_IN2;
  int breaking = motorParams.currentDirection ? MOTOR_IN2 : MOTOR_IN1;
  if (!motorParams.running) {
    if(motorParams.timerCounter <= motorParams.timerDuty){
      
      ledcWrite(breaking, 0);  // Ensure the other direction is off
      ledcWrite(direction, 128);
      motorParams.running = true;
      //Serial.println("on");
      // Serial.println("onTimerDirection:");
      // Serial.println(direction);
    }
  }
  else if (motorParams.running){
    if(motorParams.timerCounter > motorParams.timerDuty){
      ledcWrite(breaking, 0);
      ledcWrite(direction, 0);
      motorParams.running = false;
      //Serial.println("off");
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  if (motorParams.timerCounter >= 115) {
    motorParams.timerCounter = 0;
  }
}

void setupN20(){
  // Initialize encoder and PWM
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(DT, CLK);
  encoder.setFilter(1023);
  encoder.clearCount();

  // Configure PWM using API for 3.0
  if(!ledcAttachChannel(MOTOR_IN1, freq, resolution, M1Channel) || !ledcAttachChannel(MOTOR_IN2, freq, resolution, M2Channel)){
    Serial.println("Failed to setup LEDC channels");
  }
  ledcWrite(MOTOR_IN1, 0);
  ledcWrite(MOTOR_IN2, 0);

  // Set up timer
  timerSemaphore = xSemaphoreCreateBinary();  // Create semaphore to inform us when the timer has fired
  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);
  timerStop(timer);
  timerAttachInterrupt(timer, &onTimer);  // Attach onTimer function
  timerAlarm(timer, 1000, true, 0);  
}

void printEncoderCount() {
  long newPosition = encoder.getCount();
  // Serial.print("EncoderCount: ");
  // Serial.println(newPosition);
  String message = String(newPosition);
  printFit(message.c_str());

}

// Speeds 90-255 are valid for normal PWM
// Speed must be between 0 and 255
void motorControl(bool dir, int speed){
  if(speed != motorParams.motorSpeedCmd){
    //Set up directional control
    int direction = dir ? MOTOR_IN1 : MOTOR_IN2;
    int breaking = dir ? MOTOR_IN2 : MOTOR_IN1;
    motorParams.motorSpeedCmd = speed;
    static bool timerPaused = true;
    if((speed >= 90 || speed == 0) && !timerPaused){
      timerStop(timer);
      timerWrite(timer, 0);
      timerPaused = true;
    }
    // Speed must be between 0 and 255
    if(speed <= 255 && speed >= 1){
      ledcWrite(breaking, 0);  // Ensure the other direction is off
      if(speed <= 255 && speed >= 90){
        ledcWrite(direction, speed);
      }
      // USE FakePWM
      else{
        int duty = map(speed, 1, 90, 25, 115);
        if(timerPaused){
          timerStart(timer);
          timerPaused = false;
        }
        // Set up params for timed motor control
        if (xSemaphoreTake(timerSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
          portENTER_CRITICAL(&timerMux);
          motorParams.currentDirection = dir;
          motorParams.timerDuty = duty;
          portEXIT_CRITICAL(&timerMux);
          Serial.print("FakePWM:");
          Serial.print(duty);
          Serial.print(" ");
        }
        else{
          Serial.println("Semaphore Failed");
        }
      }
    }
    else if(speed == 0){
      if(!timerPaused){
        timerStop(timer);
        timerWrite(timer, 0);
        timerPaused = true;
        motorParams.running = false;
      }
      ledcWrite(breaking, 255);
      ledcWrite(direction, 255);
      
    }
  }
}

void setSpeeds(){
  int minSpeed = 0;
  for (int speed = 1; speed <= 255; speed++){
    Serial.print("PWM value:");
    Serial.print(speed);
    Serial.print(" ");
    int average = 0;
    int variance = 0;
    for (bool dir : {true, false}){
      //If this already failed in the true direction, don't bother.
      if(speed == minSpeed){
        break;
      }
      encoder.clearCount();
      int currentCount = 0;
      //int direction = dir ? MOTOR_IN1 : MOTOR_IN2;
      // Acceleration monitoring
      bool accelerating = true;
      int stableCountDuration = 0;
      int prevCount = 0;
      int distance = 0;
      int prevDistance = 0;
      int elapsedTime = 0;

      //ledcWrite(direction, speed);
      motorControl(dir, speed);
      int startTime = millis();

      while(accelerating){
        currentCount = encoder.getCount();
        elapsedTime = millis() - startTime;
        if (elapsedTime > 100 && currentCount == 0){
          minSpeed = speed;
          Serial.print("minSpeed: ");
          Serial.println(minSpeed);
          break;
        }
        distance = currentCount - prevCount;
        prevCount = currentCount;
        if ( abs(distance - prevDistance < 10)){
          stableCountDuration++;
        }
        else{
          stableCountDuration = 0;
        }
        prevDistance = distance;
        if (stableCountDuration >= 1000){
          //Done Accelerating
          Serial.print("Acceleration took ");
          Serial.print(elapsedTime);
          Serial.print(" ms. ");
          accelerating = false;
        }
      }
      if (speed > minSpeed){
        encoder.clearCount();
        delay(1000);
        int finish = encoder.getCount();
        //break
        motorControl(dir, 0);
        average += abs(finish);
        variance += finish;
        Serial.print(dir ? "Forward:" : "Reverse:");
        Serial.print(finish);
        Serial.print(" ");
        bool stillGoing = true;
        while(stillGoing){
          currentCount = encoder.getCount();
          delay(50);
          if(currentCount == encoder.getCount()){
            break;
          }
        }
      }
    }
    Serial.print(" Average:");
    Serial.print(average/2);
    Serial.print(" Variance:");
    Serial.print(variance/2);
    double rotationDegrees = (double(average) / 2 / 14316.0) * 360.0;
    Serial.print(" Degrees/sec: ");
    Serial.println(rotationDegrees);
  }
  Serial.print("Minimum test speed: ");
  Serial.println(minSpeed);
}

// Constants for the number of tests and speeds
const int numTests = 2; // Number of tests per speed
const int numSpeeds = 100; // Number of speeds to measure

// Lookup table for storing average stopping distances
long stoppingDistances[numSpeeds + 1]; // Index 1 to 100, so +1

// Function to measure and populate the lookup table
void populateStoppingDistances() {
  for (int speed = 1; speed <= numSpeeds; ++speed) {
    long totalDistance = 0;
    Serial.print("Speed:");
    Serial.println(speed);
    for (int test = 0; test < numTests; ++test) {
      // Set motor to the specified speed
      motorControl(true, speed); // Assuming motorControl sets the speed
      
      // Wait for motor to stabilize (adjust as needed)
      delay(1000); // Adjust delay time as per motor characteristics
      
      // Measure stopping distance (in encoder counts)
      long initialPosition = encoder.getCount();
      
      // Ensure motor stops completely
      motorControl(true, 0);
      delay(500); // Delay after stopping to stabilize
      
      
      long finalPosition = encoder.getCount();
      long distance = abs(finalPosition - initialPosition);
      Serial.print(" Test:");
      Serial.print(test);
      Serial.print(" Distance:");
      Serial.print(distance);
      
      totalDistance += distance;
      
      
    }
    // Calculate average stopping distance and store in the lookup table
    stoppingDistances[speed] = totalDistance / numTests;
    Serial.print(" Average:");
    Serial.println(stoppingDistances[speed]);
  }
}

// Function to estimate stopping distance using the lookup table
long estimateStoppingDistance(int speed) {
  // Check if speed is within the valid range
  if (speed < 1) speed = 1;
  if (speed > numSpeeds) speed = numSpeeds;
  
  // Return the pre-measured stopping distance from the lookup table
  return stoppingDistances[speed];
}


void autotunePID() {
  // Gradient descent parameters
  double delta = 0.1; // Increment for each PID constant adjustment
  double tolerance = 0.001; // Tolerance to determine convergence

  // Iterate through PID constants
  for (;;) {
    // Reset PID variables
    integral = 0;
    last_error = 0;
    current_error = 0;

    // Perform PID control for a fixed number of iterations or until convergence
    for (int i = 0; i < 1000; ++i) {
      Serial.print(i);
      // Read encoder position
      long currentPosition = encoder.getCount();

      // Calculate PID error
      double error = rotTarget - currentPosition;
      integral += error;
      double derivative = error - last_error;

      // PID output calculation
      double output = Kp * error + Ki * integral + Kd * derivative;

      // Adjust motor control based on PID output
      int speed = constrain(abs(output), 0, 100);
      Serial.print(": Speed:");
      Serial.println(speed);
      bool direction = output > 0;
      motorControl(direction, speed);

      // Determine when to apply braking (coast detection)
      if (abs(error) < 10) { // Adjust threshold as needed
        // Apply braking
        motorControl(direction, 0); // Braking mode
        delay(100); // Adjust braking duration as needed
      }

      // Update last error for derivative term
      last_error = error;

      // Accumulate current error for this iteration
      current_error += abs(error);

      // Delay for stability
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Calculate average error over iterations
    current_error /= 1000;

    // Check if current PID constants provide better performance
    if (current_error < best_error) {
      best_error = current_error;
      best_Kp = Kp;
      best_Ki = Ki;
      best_Kd = Kd;
    }

    // Adjust PID constants for next iteration
    Kp += delta;
    Ki += delta;
    Kd += delta;

    // Check for convergence based on tolerance
    if (delta < tolerance) {
      break; // Exit if tolerance is met
    }
  }

  // Apply the best PID constants found
  Kp = best_Kp;
  Ki = best_Ki;
  Kd = best_Kd;

  // Print the best PID constants found
  Serial.print("Best PID constants - Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.println(Kd);
}
