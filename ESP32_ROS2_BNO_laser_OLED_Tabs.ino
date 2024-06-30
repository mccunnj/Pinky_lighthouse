#include "oled.h"
#include "imu.h"
// #include "laser.h"
#include "ros2.h"
#include "n20.h"

#define LED_PIN 2

// Task handle declaration
TaskHandle_t Task1;
// Function declarations
void Task1code(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  setupOLED();
  setupIMU();
  //setupROS2();
  setupN20();

  // Create a task on core 1 to output encoder data
  xTaskCreatePinnedToCore(
    Task1code,   // Task function
    "Task1",     // Task name
    10000,       // Stack size (bytes)
    NULL,        // Task input parameters
    1,           // Priority (1 is lower priority than setup(), adjust as needed)
    &Task1,      // Task handle
    1);          // Core to run the task (core 1)
  setSpeeds();
  //populateStoppingDistances();
  // Perform PID autotuning
  //autotunePID();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Check Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi connection lost. Attempting to reconnect...");
    WiFi.begin("PinkyPi", "KendrickVolta");
    while (WiFi.status() != WL_CONNECTED) {
      delay(2000);
      Serial.println("Reconnecting to Wi-Fi...");
      printFit("Reconnecting to WIFI");
    }
    Serial.println("Reconnected to Wi-Fi");
    printFit("Reconnected to Controller");
  }
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // delay(100);

  // ROS2
  //publishIMUData();
  // Handle micro-ROS executor
  //move to ros2.h? rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));



  //printEncoderCount();
  //Scrolls text if necessary
  updateScroll();
}

void Task1code(void *pvParameters) {
  // This task continuously prints encoder count
  for (;;) {
    printEncoderCount();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}