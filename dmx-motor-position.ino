#include <AccelStepper.h>
#include <Conceptinetics.h>

#define DMX_SLAVE_CHANNELS 2
#define RXEN_PIN 2

#define DIR_PIN 7
#define STEP_PIN 6
#define LED_PIN 9

// Define max speed (steps per second) for slow movement.
const float MAX_SPEED = 200.0;

// Define acceleration limits (steps/s²)
const float MIN_ACCEL = 50.0;
const float MAX_ACCEL = 200.0;

// Variables to store the last DMX values for change detection.
uint8_t curDmxVal1 = 0;
uint8_t curDmxVal2 = 0;

// Create a DMX slave instance with 2 channels.
DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS, RXEN_PIN);

// Initialize AccelStepper in DRIVER mode (for STEP and DIR pins).
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  dmx_slave.enable();
  dmx_slave.setStartAddress(1);

  stepper.setMaxSpeed(MAX_SPEED);
  // Set a default acceleration; this will be updated by DMX channel 2.
  stepper.setAcceleration(MAX_ACCEL);

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read DMX channel values.
  uint8_t newDmxVal1 = dmx_slave.getChannelValue(1); // Position control.
  uint8_t newDmxVal2 = dmx_slave.getChannelValue(2); // Acceleration control.

  // Update position if DMX channel 1 value has changed.
  if (newDmxVal1 != curDmxVal1) {
    curDmxVal1 = newDmxVal1;
    
    // Map DMX 0-255 to a full revolution (0-200 steps).
    float targetPosition = ((float)newDmxVal1 / 255.0) * 200.0;
    stepper.moveTo(targetPosition);
    
    // Use the LED as a debug indicator (brightness equals DMX value).
    analogWrite(LED_PIN, newDmxVal1);
  }

  // Update acceleration if DMX channel 2 value has changed.
  if (newDmxVal2 != curDmxVal2) {
    curDmxVal2 = newDmxVal2;
    
    // Map DMX 0-255 to an acceleration range between MIN_ACCEL and MAX_ACCEL.
    float newAccel = ((float)newDmxVal2 / 255.0) * (MAX_ACCEL - MIN_ACCEL) + MIN_ACCEL;
    stepper.setAcceleration(newAccel);
  }

  // Run the stepper to move it toward the target position.
  stepper.run();
}
