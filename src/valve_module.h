/**
 * @file
 *
 * @brief Provides ValveModule class that controls a solenoid valve and, optionally, a piezoelectric tone buzzer.
 */

#ifndef AXMC_VALVE_MODULE_H
#define AXMC_VALVE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Dispenses precise volumes of fluid by sending digital currents through a solenoid valve and optionally emits
 * tones by sending digital currents through a piezoelectric buzzer.
 *
 * @tparam kPin the digital pin connected to the logic terminal of the managed solenoid valve's FET-gated power relay.
 * @tparam kNormallyClosed determines whether the managed solenoid valve is opened (allows fluid flow) or closed
 * (resists fluid flow) when unpowered.
 * @tparam kStartClosed determines the initial state of the managed solenoid valve during class initialization.
 * @tparam kTonePin the digital pin connected to the logic terminal of the managed piezoelectric buzzer's FET-gated
 * power relay.
 * @tparam kNormallyOff determines whether the FET relay used to control the piezoelectric buzzer is closed
 * (On / conducting) or opened (Off / not conducting) when unpowered.
 * @tparam kStartOff determines the initial state of the managed piezoelectric buzzer during class initialization.
 */
template <
    const uint8_t kPin,
    const bool kNormallyClosed,
    const bool kStartClosed = true,
    const uint8_t kTonePin  = 255,
    const bool kNormallyOff = true,
    const bool kStartOff    = true>
class ValveModule final : public Module
{
        // Ensures that the valve pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different valve pin for the ValveModule "
            "instance."
        );
        // Ensures that the tone pin does not interfere with the LED pin.
        static_assert(
            kTonePin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different tone pin for the ValveModule "
            "instance."
        );

    public:

        /// Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kOpen          = 51,  ///< The valve is open.
            kClosed        = 52,  ///< The valve is closed.
            kCalibrated    = 53,  ///< The valve has completed a calibration cycle.
            kToneOn        = 54,  ///< The tone is played.
            kToneOff       = 55,  ///< The tone is silenced.
            kTonePinNotSet = 56,  ///< The tone pin was not set during class initialization.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse = 1,  ///< Opens the valve for the requested period of time and then closes it.
            kToggleOn  = 2,  ///< Opens the managed valve.
            kToggleOff = 3,  ///< Closes the managed valve.
            kCalibrate = 4,  ///< Repeatedly opens the valve for the requested period of time.
            kTonePulse = 5,  ///< Activates the managed buzzer to play the tone for the requested period of time.
        };

        /// Initializes the base Module class.
        ValveModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the module's runtime parameters structure with the data received from the PC.
        bool SetCustomParameters() override
        {
            // Attempts to extract the received parameters
            if (_communication.ExtractModuleParameters(_custom_parameters))
            {
                if (kTonePin != 255 && _custom_parameters.tone_duration != 0)
                {

                }
                if (kTonePin != 255 && _custom_parameters.tone_duration > _custom_parameters.pulse_duration)
                {
                    _tone_time_delta = _custom_parameters.pulse_duration - _custom_parameters.tone_duration;
                }
                return true;
            }
            return false;
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // Pulse
                case kModuleCommands::kSendPulse: Pulse(); return true;
                // Open
                case kModuleCommands::kToggleOn: Open(); return true;
                // Close
                case kModuleCommands::kToggleOff: Close(); return true;
                // Calibrate
                case kModuleCommands::kCalibrate: Calibrate(); return true;
                // Tone
                case kModuleCommands::kTonePulse: Tone(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // If the instance is configured to interface with the tone buzzer, configures the output pin.
            if (kTonePin != 255)
            {
                // Sets the tone state based on the configuration of the tone buzzer's FET gate and the desired initial
                // state.
                pinModeFast(kTonePin, OUTPUT);
                if (kStartOff)
                {
                    digitalWriteFast(kTonePin, kInactive);  // Ensures that the tone buzzer is powered off.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
                }
                else
                {
                    digitalWriteFast(kTonePin, kActive);  // Ensures that the tone buzzer is powered on.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOn));
                }
            }
            else
            {
                // Otherwise, notifies the PC that the tone is statically off for the entire runtime's duration.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
            }

            // Sets the valve state based on the configuration of the valve's FET gate and the desired initial
            // state.
            pinModeFast(kPin, OUTPUT);
            if (kStartClosed)
            {
                digitalWriteFast(kPin, kClose);  // Ensures the valve is closed.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
            }
            else
            {
                digitalWriteFast(kPin, kOpen);  // Ensures the valve is open.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 35000;   // ~ 5.0 uL of water in the current Sun lab system.
            _custom_parameters.calibration_count = 500;     // The valve is pulsed 500 times during calibration.
            _custom_parameters.tone_duration     = 300000;  // 300 milliseconds.

            return true;
        }

        ~ValveModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration    = 35000;   ///< The time, in microseconds, to keep the valve open.
                uint16_t calibration_count = 500;     ///< The number of times to pulse the valve during calibration.
                uint32_t tone_duration     = 300000;  ///< The time, in microseconds, to keep playing the tone.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the digital signal that needs to be sent to the output pin to open the valve.
        static constexpr bool kOpen = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to close the valve.
        static constexpr bool kClose = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to activate the tone buzzer.
        static constexpr bool kActive = kNormallyOff ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to deactivate the tone buzzer.
        static constexpr bool kInactive = kNormallyOff ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the time, in microseconds, that must separate any two consecutive pulses during valve calibration.
        /// The value for this attribute is selected primarily for a system safety consideration, as pulsing the
        /// valve too fast may generate undue stress in the hydraulic system.
        static constexpr uint32_t kCalibrationDelay = 300000;

        /// Stores the difference, in microseconds, between the current valve's pulse duration and the buzzer's tone
        /// duration.
        uint32_t _tone_time_delta = 0;

        /// Tracks whether the instance is currently configured to use the piezoelectric tone buzzer to emit tone
        /// signals
        bool _tone_used = false;

        /// Opens the valve to deliver a precise volume of fluid and then closes it.
        void Pulse()
        {
            switch (execution_parameters.stage)
            {
                // Opens the valve
                case 1:

                    // Engages the solenoid valve
                    digitalWriteFast(kPin, kOpen);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));

                    // If the instance is configured to deliver audible tones when the valve is open, also activates
                    // the tone buzzer.
                    if (kTonePin != 255 && _custom_parameters.tone_duration != 0)
                    {
                        digitalWriteFast(kTonePin, HIGH);
                        SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOn));
                    }

                    AdvanceCommandStage();
                    return;

                // Waits for the requested valve pulse duration of microseconds to pass.
                case 2:
                    if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Closes the valve
                case 3:
                    digitalWriteFast(kPin, kClose);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));

                    // If the tone buzzer is not used, finished the command's runtime.
                    if (kTonePin == 255 && _custom_parameters.tone_duration != 0) CompleteCommand();
                    else AdvanceCommandStage();
                    return;

                case 4:
                    // If the buzzer's tone duration is less than the valve's pulse duration, advances to the next stage
                    // to silence the buzzer.
                    if (_custom_parameters.tone_duration <= _custom_parameters.pulse_duration) AdvanceCommandStage();

                    // Computes the reminaing
                    if (!WaitForMicros(_custom_parameters.tone_duration - _custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                case 5:
                    digitalWriteFast(kTonePin, LOW);                               // Ensures the tone is turned OFF
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));  // Notifies the PC
                    CompleteCommand();                                             // Finishes command execution

                default: CompleteCommand();
            }
        }

        /// Permanently opens the valve.
        void Open()
        {
            // Sets the pin to Open signal and finishes command execution
            if (DigitalWrite(kPin, kOpen, false))
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
                CompleteCommand();
            }
            else
            {
                // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Permanently closes the valve.
        void Close()
        {
            // Sets the pin to Close signal and finishes command execution
            if (DigitalWrite(kPin, kClose, false))
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
                CompleteCommand();  // Finishes command execution
            }
            else
            {
                // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Pulses the valve calibration_count times without blocking or (majorly) delaying. This is used to establish
        /// the relationship between the pulse_duration and the amount of fluid delivered during the pulse. This
        /// calibration is necessary to precisely control the amount of fluid delivered by the valve by using specific
        /// pulse durations.
        void Calibrate()
        {
            // Pulses the valve the requested number of times. Note, the command logic is very similar to the
            // Pulse command, but it is slightly modified to account for the fact that some boards can issue commands
            // too fast for the valve hardware to properly respond to them. Also, this command is blocking by design and
            // will run all requested pulse cycles in one go.
            for (uint16_t i = 0; i < _custom_parameters.calibration_count; ++i)
            {
                // Opens the valve
                if (!DigitalWrite(kPin, kOpen, false))
                {
                    // Respects the global controller lock state
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();
                    return;
                }

                // Blocks in-place until the pulse duration passes.
                delayMicroseconds(_custom_parameters.pulse_duration);

                // Closes the valve
                if (!DigitalWrite(kPin, kClose, false))
                {
                    // Respects the global controller lock state
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }

                // Blocks for calibration_delay of microseconds to ensure the valve closes before initiating the next
                // cycle.
                delayMicroseconds(kCalibrationDelay);
            }

            // This command completes after running the requested number of cycles.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kCalibrated));
            CompleteCommand();
        }

        /// Cycles activating and inactivating the tone buzzer to deliver an audible tone of the predefined duration,
        /// without changing the current state of the valve.
        void Tone()
        {
            // If the Tone pin is not configured, aborts the runtime and sends an error message to the PC.
            if (kTonePin == 255)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kTonePinNotSet));
                AbortCommand();
                return;
            }

            // Starts the Tone by activating the buzzer
            if (execution_parameters.stage == 1)
            {
                if (DigitalWrite(kTonePin, HIGH, false))
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOn));
                    AdvanceCommandStage();
                }
                else
                {
                    // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();
                    return;
                }
            }

            // Sounds the tone for the required duration of microseconds
            if (execution_parameters.stage == 2)
            {
                // Blocks for the tone_duration of microseconds, relative to the time of the last AdvanceCommandStage()
                // call.
                if (!WaitForMicros(_custom_parameters.tone_duration)) return;
                AdvanceCommandStage();
            }

            // Deactivates the tone
            if (execution_parameters.stage == 3)
            {
                // Once the tone duration has passed, inactivates the pin by setting it to LOW. Finishes
                // command execution if inactivation is successful.
                if (DigitalWrite(kTonePin, LOW, false))
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
                    CompleteCommand();
                }
                else
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();
                }
            }
        }
};

#endif  //AXMC_VALVE_MODULE_H
