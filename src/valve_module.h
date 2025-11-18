/**
 * @file
 *
 * @brief Provides the ValveModule class that controls a solenoid valve and, optionally, a piezoelectric tone buzzer.
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
 * @tparam kValvePin the digital pin connected to the logic terminal of the managed solenoid valve's FET-gated power
 * relay.
 * @tparam kNormallyClosed determines whether the managed solenoid valve is opened (allows the fluid to flow) or closed
 * (prevents the fluid from flowing) when unpowered.
 * @tparam kStartClosed determines the initial state of the managed solenoid valve during class initialization.
 * @tparam kTonePin the digital pin connected to the logic terminal of the managed piezoelectric buzzer's FET-gated
 * power relay.
 * @tparam kNormallyOff determines whether the FET relay used to control the piezoelectric buzzer is closed
 * (on / conducting) or opened (off / not conducting) when unpowered.
 * @tparam kStartOff determines the initial state of the managed piezoelectric buzzer during class initialization.
 */
template <
    const uint8_t kValvePin,
    const bool kNormallyClosed,
    const bool kStartClosed = true,
    const uint8_t kTonePin  = 255,
    const bool kNormallyOff = true,
    const bool kStartOff    = true>
class ValveModule final : public Module
{
        // Ensures that the valve pin does not interfere with the LED pin.
        static_assert(
            kValvePin != LED_BUILTIN,
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
            kOpen                     = 51,  ///< The valve is open.
            kClosed                   = 52,  ///< The valve is closed.
            kCalibrated               = 53,  ///< The valve has completed a calibration cycle.
            kToneOn                   = 54,  ///< The tone is played.
            kToneOff                  = 55,  ///< The tone is silenced.
            kInvalidToneConfiguration = 56,  ///< The instance is not configured to emit audible tones.
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
                // If the instance is not configured to use the tone buzzer, ensures that the tone duration is set to 0.
                // This is used to streamline the logic of some commands.
                if (kTonePin == 255) _custom_parameters.tone_duration = 0;

                // If the tone buzzer is used and the tone duration exceeds the pulse duration, computes and stores the
                // difference between the two durations.
                if (_custom_parameters.tone_duration > _custom_parameters.pulse_duration)
                    _tone_time_delta = _custom_parameters.tone_duration - _custom_parameters.pulse_duration;
                else _tone_time_delta = 0;  // Otherwise, caps the tone duration to the pulse duration.
                return true;
            }
            return false;  // If parameter extraction fails,.
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
                    digitalWriteFast(kTonePin, kInactivate);  // Ensures that the tone buzzer is powered off.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
                }
                else
                {
                    digitalWriteFast(kTonePin, kActivate);  // Ensures that the tone buzzer is powered on.
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
            pinModeFast(kValvePin, OUTPUT);
            if (kStartClosed)
            {
                digitalWriteFast(kValvePin, kClose);  // Ensures the valve is closed.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
            }
            else
            {
                digitalWriteFast(kValvePin, kOpen);  // Ensures the valve is open.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 35000;  // ~ 5.0 uL of water in the current Sun lab system.
            _custom_parameters.calibration_count = 500;    // The valve is pulsed 500 times during calibration.

            // Note, tone duration is set depending on whether the tone pin is used at all.
            if (kTonePin != 255) _custom_parameters.tone_duration = 300000;  // 300 milliseconds.
            else _custom_parameters.tone_duration = 0;                       // No tone is used.

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

        /// Stores the digital signal that needs to be sent to the valve pin to open the valve.
        static constexpr bool kOpen = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the valve pin to close the valve.
        static constexpr bool kClose = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the tone pin to activate the tone buzzer.
        static constexpr bool kActivate = kNormallyOff ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the tone pin to deactivate the tone buzzer.
        static constexpr bool kInactivate = kNormallyOff ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the time, in microseconds, that must separate any two consecutive pulses during the valve
        /// calibration. The value for this attribute is hardcoded for the system's safety, as pulsing the
        /// valve too fast may generate undue stress in the calibrated hydraulic system.
        static constexpr uint32_t kCalibrationDelay = 300000;

        /// Stores the difference, in microseconds, between the valve's pulse duration and the buzzer's tone duration,
        /// if both are used during valve pulsing.
        uint32_t _tone_time_delta = 0;

        /// Tracks the number of valve calibration cycles completed as part of executing the current calibration
        /// command.
        uint16_t _calibration_iteration = 0;

        /// Opens the valve to deliver a precise volume of fluid and then closes it.
        void Pulse()
        {
            switch (execution_parameters.stage)
            {
                // Opens the valve
                case 1:
                    digitalWriteFast(kValvePin, kOpen);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));

                    // Activates the tone buzzer if the module is configured to use it.
                    if (_custom_parameters.tone_duration != 0)
                    {
                        digitalWriteFast(kTonePin, kActivate);
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
                    digitalWriteFast(kValvePin, kClose);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));

                    // If the tone buzzer is not used or the tone duration is equal to the valve's pulse duration,
                    // also shuts down the tone buzzer and aborts the command execution
                    if (_tone_time_delta == 0) CompleteCommand();
                    else AdvanceCommandStage();
                    return;

                // Optional stage: Waits for the remaining tone duration of microseconds to pass.
                case 4:

                    // Waits for the remaining tone duration to pass.
                    if (!WaitForMicros(_tone_time_delta)) return;

                    // Shuts down the tone buzzer.
                    digitalWriteFast(kTonePin, kInactivate);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }

        /// Opens the valve.
        void Open()
        {
            digitalWriteFast(kValvePin, kOpen);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
            CompleteCommand();
        }

        /// Closes the valve.
        void Close()
        {
            digitalWriteFast(kValvePin, kClose);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
            CompleteCommand();
        }

        /// Opens the valve for the requested pulse_duration microseconds and repeats the procedure for the
        /// calibration_count repetitions without blocking or (majorly) delaying.
        void Calibrate()
        {
            switch (execution_parameters.stage)
            {
                // Resets the calibration iteration tracker
                case 1:
                    _calibration_iteration = 0;
                    AdvanceCommandStage();
                    return;

                // Dispenses one reward at each cycle until the requested calibration count is dispensed
                case 2:
                    // Opens the valve
                    digitalWriteFast(kValvePin, kOpen);

                    // Blocks in-place until the pulse duration passes.
                    delayMicroseconds(_custom_parameters.pulse_duration);

                    // Closes the valve
                    digitalWriteFast(kValvePin, kClose);

                    // Blocks for kCalibrationDelay of microseconds to ensure the valve closes before initiating the
                    // next cycle.
                    delayMicroseconds(kCalibrationDelay);

                    _calibration_iteration++;

                    if (_calibration_iteration >= _custom_parameters.calibration_count)
                    {
                        AdvanceCommandStage();
                    }
                    return;

                // Complete calibration
                case 3:
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kCalibrated));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }

        /// Activates the tone buzzer to deliver an audible tone for the specified duration of time and then
        /// inactivates it.
        void Tone()
        {
            // If the Tone pin is not configured, aborts the runtime and sends an error message to the PC.
            // Since version 3.0.0, this check also includes cases when the tone duration is set to 0.
            if (_custom_parameters.tone_duration == 0)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidToneConfiguration));
                AbortCommand();
                return;
            }

            switch (execution_parameters.stage)
            {
                // Activates the tone buzzer.
                case 1:
                    digitalWriteFast(kTonePin, kActivate);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOn));
                    AdvanceCommandStage();
                    return;

                    // Waits for the tone duration to pass.
                case 2:
                    if (!WaitForMicros(_custom_parameters.tone_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Inactivates the tone buzzer.
                case 3:
                    digitalWriteFast(kTonePin, kInactivate);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kToneOff));
                    CompleteCommand();

                default: AbortCommand();
            }
        }
};

#endif  //AXMC_VALVE_MODULE_H
