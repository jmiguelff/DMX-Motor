#include <AccelStepper.h>
#include <Conceptinetics.h>

#define DMX_SLAVE_CHANNELS 2
#define RXEN_PIN 2

#define DIR_PIN 7
#define STEP_PIN 6
// #define LED_PIN 9

// For a standard 1.8° motor, one full revolution = 200 steps.
const float FULL_REV_STEPS = 200.0;   
const float POSITION_MAX_SPEED = 20.0; // Maximum speed for position mode
const float MIN_ACCEL = 10.0;           // Minimum acceleration (steps/s²)
const float MAX_ACCEL = 5.0;          // Maximum acceleration (steps/s²)

// Continuous mode: fixed speed (steps per second)
const float CONTINUOUS_SPEED = 40.0;  

// DMX channel variables for change detection.
uint8_t curDmxVal1 = 0;
uint8_t curDmxVal2 = 0;
uint8_t lastDmxVal2 = 0;

// When switching from continuous to position mode, record the current position.
long baselinePosition = 0;

DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS, RXEN_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  dmx_slave.enable();
  dmx_slave.setStartAddress(511);

  // Set up the stepper for position mode initially.
  stepper.setMaxSpeed(POSITION_MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  
  // pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read DMX channel 2 (mode and speed control)
  uint8_t newDmxVal2 = dmx_slave.getChannelValue(2);

  // Continuous mode: DMX channel 2 > 127
  if (newDmxVal2 > 127) {
    // Ignore channels 1; run continuously at a fixed speed.
    stepper.setSpeed(CONTINUOUS_SPEED);
    stepper.runSpeed();

    lastDmxVal2 = newDmxVal2;
  }
  else {
    if (lastDmxVal2 > 127) {
      baselinePosition = stepper.currentPosition();
      stepper.moveTo(baselinePosition);
    }
    lastDmxVal2 = newDmxVal2;

    // Position mode: DMX channel 2 <= 127.
    // Use DMX channel 1 to set the desired offset relative to baseline.
    uint8_t newDmxVal1 = dmx_slave.getChannelValue(1);
    // Update position if DMX channel 1 value has changed.
    if (newDmxVal1 != curDmxVal1) {
      curDmxVal1 = newDmxVal1;
      // Map DMX channel 1 (0 to 255) so that 127 represents 0 offset.
      // For example: 
      //  DMX = 0   --> offset = -FULL_REV_STEPS/2 (-100 steps)
      //  DMX = 127 --> offset ≈ 0 steps
      //  DMX = 255 --> offset = +FULL_REV_STEPS/2 (+100 steps)
      float offset = (((float)newDmxVal1 / 255.0) * FULL_REV_STEPS) - (FULL_REV_STEPS / 2.0);
      long targetPosition = baselinePosition + (long)offset;
      stepper.moveTo(targetPosition);
    }
    
    // Run the motor toward the updated target position.
    stepper.run();
  }
}