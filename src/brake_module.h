/**
 * @file
 *
 * @brief Provides the BrakeModule class that controls an electromagnetic particle brake.
 */

#ifndef AXMC_BRAKE_MODULE_H
#define AXMC_BRAKE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Controls the electromagnetic brake by sending digital or analog Pulse-Width-Modulated (PWM) currents through
 * the brake.
 *
 * @tparam kPin the analog pin connected to the logic terminal of the managed brake's FET-gated power relay.
 * @tparam kNormallyEngaged determines whether the brake is engaged (active) or disengaged (inactive) when unpowered.
 * @tparam kStartEngaged determines the initial state of the brake during class initialization.
 */
template <const uint8_t kPin, const bool kNormallyEngaged, const bool kStartEngaged = true>
class BrakeModule final : public Module
{
        // Ensures that the output pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different pin for the BrakeModule "
            "instance."
        );

    public:
        /// Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kEngaged    = 51,  ///< The brake is engaged at maximum possible strength.
            kDisengaged = 52,  ///< The brake is disengaged.
            kVariable   = 53,  ///< The brake is engaged at the specified non-maximal strength.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kToggleOn        = 1,  ///< Engages the brake at maximum strength.
            kToggleOff       = 2,  ///< Disengages the brake.
            kSetBrakingPower = 3,  ///< Sets the brake to engage at the requested braking strength.
            kSendPulse       = 4,  ///< Briefly engages the brake at maximum strength for the specified duration.
        };

        /// Initializes the base Module class.
        BrakeModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the module's runtime parameters structure with the data received from the PC.
        bool SetCustomParameters() override
        {
            if (_communication.ExtractModuleParameters(_custom_parameters))
            {
                // Adjusts the PWM value to account for whether the brake is normally engaged. This ensures that the
                // strength of 255 means the brake is fully engaged.
                uint8_t value = _custom_parameters.braking_strength;
                if (kNormallyEngaged) value = 255 - value;
                _custom_parameters.braking_strength = value;
                return true;  // Extraction (and adjustment) succeeded.
            }
            return false;  // Returns false if the parameter extraction fails
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // EnableBrake
                case kModuleCommands::kToggleOn: EnableBrake(); return true;
                // DisableBrake
                case kModuleCommands::kToggleOff: DisableBrake(); return true;
                // SetBrakingPower
                case kModuleCommands::kSetBrakingPower: SetBrakingPower(); return true;
                // SendPulse
                case kModuleCommands::kSendPulse: SendPulse(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinModeFast(kPin, OUTPUT);

            // Based on the requested initial brake state and the configuration of the brake (normally engaged or not),
            // either engages or disengages the brake following setup.
            if (kStartEngaged)
            {
                digitalWriteFast(kPin, kEngage);  // Ensures the brake is engaged.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kEngaged));
            }
            else
            {
                digitalWriteFast(kPin, kDisengage);  // Ensures the brake is disengaged.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kDisengaged));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.braking_strength = 128;      //  50% braking strength
            _custom_parameters.pulse_duration   = 1000000;  // 1000000 microseconds == 1 second.

            return true;
        }

        ~BrakeModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint8_t braking_strength = 128;      ///< Determines the strength of the brake in variable mode.
                uint32_t pulse_duration  = 1000000;  ///< The time, in microseconds, to engage the brake during pulses.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the digital signal that needs to be sent to the output pin to engage the brake at maximum strength.
        static constexpr bool kEngage = kNormallyEngaged ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to disengage the brake.
        static constexpr bool kDisengage = kNormallyEngaged ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Engages the brake at the maximum strength.
        void EnableBrake()
        {
            digitalWriteFast(kPin, kEngage);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kEngaged));
            CompleteCommand();
        }

        /// Disengages the brake.
        void DisableBrake()
        {
            digitalWriteFast(kPin, kDisengage);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kDisengaged));
            CompleteCommand();
        }

        /// Engages the brake at the specified strength level.
        void SetBrakingPower()
        {
            // Uses AnalogWrite to make the pin output a square wave pulse with the desired duty cycle (PWM). This
            // results in the brake being applied a certain proportion of time, producing the desired braking power.
            analogWrite(kPin, _custom_parameters.braking_strength);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kVariable));
            CompleteCommand();
        }

        /// Engages the brake at maximum strength for the requested pulse_duration of microseconds and then
        /// disengages it.
        void SendPulse()
        {
            switch (execution_parameters.stage)
            {
                // Engages the brake at maximum strength.
                case 1:
                    digitalWriteFast(kPin, kEngage);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kEngaged));
                    AdvanceCommandStage();
                    return;

                // Delays for the requested number of microseconds.
                case 2:
                    if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Disengages the brake.
                case 3:
                    digitalWriteFast(kPin, kDisengage);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kDisengaged));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }
};

#endif  //AXMC_BRAKE_MODULE_H
