#include <AccelStepper.h>
#include <Conceptinetics.h>

#define DMX_SLAVE_CHANNELS 3
#define RXEN_PIN 2

#define DIR_PIN 7
#define STEP_PIN 6
// #define LED_PIN 9

// Position mode constants:
const float FULL_REV_STEPS = 200.0;   // One full revolution (360°) for a 1.8° step motor
const float POSITION_MAX_SPEED = 200.0; // Maximum speed for position mode (steps per second)
const float MIN_ACCEL = 50.0;           // Minimum acceleration (steps/s²)
const float MAX_ACCEL = 200.0;          // Maximum acceleration (steps/s²)

// Continuous mode: fixed speed (steps per second)
const float CONTINUOUS_SPEED = 80.0;  

// DMX channel variables for change detection.
uint8_t curDmxVal1 = 0;
uint8_t curDmxVal2 = 0;
uint8_t curDmxVal3 = 0;
uint8_t lastDmxVal3 = 0;

DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS, RXEN_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  dmx_slave.enable();
  dmx_slave.setStartAddress(1);

  // Set initial parameters for position mode.
  stepper.setMaxSpeed(POSITION_MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  
 //  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read DMX channel 3 (mode control)
  uint8_t newDmxVal3 = dmx_slave.getChannelValue(3);

  // Detect if we've just switched from continuous mode to position mode.
  if (newDmxVal3 <= 127 && lastDmxVal3 > 127) {
    // Stop continuous rotation: set target to current position.
    stepper.moveTo(stepper.currentPosition());
  }
  lastDmxVal3 = newDmxVal3;

  if (newDmxVal3 > 127) {
    // Continuous mode: ignore channels 1 & 2, run at fixed speed.
    stepper.setSpeed(CONTINUOUS_SPEED);
    stepper.runSpeed();
  } else {
    // Position mode: use DMX channels 1 and 2.
    uint8_t newDmxVal1 = dmx_slave.getChannelValue(1);
    uint8_t newDmxVal2 = dmx_slave.getChannelValue(2);
    
    // Update target position if channel 1 changes.
    if (newDmxVal1 != curDmxVal1) {
      curDmxVal1 = newDmxVal1;
      // Map DMX 0-255 to 0-200 steps (0° to 360°).
      float targetPosition = ((float)newDmxVal1 / 255.0) * FULL_REV_STEPS;
      stepper.moveTo(targetPosition);
      // analogWrite(LED_PIN, newDmxVal1); // Optional debug indicator.
    }
    
    // Update acceleration if channel 2 changes.
    if (newDmxVal2 != curDmxVal2) {
      curDmxVal2 = newDmxVal2;
      float newAccel = ((float)newDmxVal2 / 255.0) * (MAX_ACCEL - MIN_ACCEL) + MIN_ACCEL;
      stepper.setAcceleration(newAccel);
    }
    
    // Move towards the target position.
    stepper.run();
  }
}