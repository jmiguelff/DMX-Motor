v#include <AccelStepper.h>

#include <Conceptinetics.h>

#define DMX_SLAVE_CHANNELS 2
#define RXEN_PIN 2

#define DIR_PIN 7
#define STEP_PIN 6
#define LED_PIN 9

const float MAX_SPEED = 1000.0;
const float ACCELERATION = 200.0;
uint8_t curDmxVal = 0;

DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS, RXEN_PIN);

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


void setup() {
  dmx_slave.enable();
  dmx_slave.setStartAddress(1);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read DMX val
  uint8_t newDmxVal = dmx_slave.getChannelValue(1);
  if (newDmxVal != curDmxVal) {
    curDmxVal = newDmxVal;

    // Use LED as debug -> brightness equal to dmx value
    analogWrite(LED_PIN, newDmxVal);

    // Map the potentiometer value to a speed between 0 and MAX_SPEED.
    // The division by 1023.0 ensures we get a float value.
    float newSpeed = ((float)newDmxVal / 255.0) * MAX_SPEED;

    // Set the new speed for the stepper motor.
    stepper.setSpeed(newSpeed);
  }

  // Continuously run the motor at the set speed.
  // This non-blocking function should be called as often as possible.
  stepper.runSpeed();
}