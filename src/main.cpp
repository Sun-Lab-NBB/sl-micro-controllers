// To upload the code to each of the target microcontrollers, modify the target microcontroller name on line 26 and
// use platformio to compile and upload the project. When uploading the code, make sure only one
// Teensy 4.1 is connected to the host-PC at the same time.

// The microcontrollers assembled and configured as part of this project are designed to work with the Python
// interfaces available from the sl-experiment project: https://github.com/Sun-Lab-NBB/sl-experiment.
// See https://github.com/Sun-Lab-NBB/sl-micro-controllers for more details on assembling the hardware and
// installing the project.
// API documentation: https://sl-micro-controllers-api-docs.netlify.app/.
// Author: Ivan Kondratyev (Inkaros).

// Dependencies
#include <Arduino.h>
#include <communication.h>
#include <kernel.h>
#include <module.h>

// Initializes the serial communication class.
Communication axmc_communication(Serial);  // NOLINT(*-interfaces-global-init)

// Defines the keepalive interval used during runtime.
static constexpr uint32_t kKeepaliveInterval = 500;  // 500 ms

// Defines the target microcontroller. Our VR system currently has 3 valid targets: ACTOR, SENSOR and ENCODER.
#define ACTOR

// Resolves microcontroller-specific module configuration and layout
#ifdef ACTOR
#include "brake_module.h"
#include "screen_module.h"
#include "valve_module.h"

constexpr uint8_t kControllerID = 101;
BrakeModule<33, false, true> wheel_brake(3, 1, axmc_communication);
ValveModule<35, true, true, 34> reward_valve(5, 1, axmc_communication);
ScreenModule<36, true> screen_trigger(7, 1, axmc_communication);
Module* modules[] = {&wheel_brake, &reward_valve, &screen_trigger};

#elif defined SENSOR
#include "lick_module.h"
#include "torque_module.h"
#include "ttl_module.h"

constexpr uint8_t kControllerID = 152;
TTLModule<34, false, false> mesoscope_frame(1, 1, axmc_communication);
LickModule<41> lick_sensor(4, 1, axmc_communication);
TorqueModule<40, 2048, true> torque_sensor(6, 1, axmc_communication);
Module* modules[] = {&mesoscope_frame, &lick_sensor, &torque_sensor};

#elif defined ENCODER
#include "encoder_module.h"

constexpr uint8_t kControllerID = 203;
EncoderModule<33, 34, 35, true> wheel_encoder(2, 1, axmc_communication);
Module *modules[] = {&wheel_encoder};
#else
static_assert(false, "Define one of the supported microcontroller targets (ACTOR, SENSOR, ENCODER).");
#endif

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, modules, kKeepaliveInterval);

void setup()
{
    Serial.begin(115200);  // The baudrate is ignored for teensy boards.

    // Sets ADC resolution to 12 bits. Teensy boards can support up to 16 bits, but 12 often produces cleaner readouts.
    analogReadResolution(12);

    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.
}

void loop()
{
    axmc_kernel.RuntimeCycle();
}
