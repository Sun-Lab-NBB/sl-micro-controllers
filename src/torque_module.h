/**
 * @file
 * @brief Provides the TorqueModule class that monitors and records the data produced by a reaction torque sensor.
 */

#ifndef AXMC_TORQUE_MODULE_H
#define AXMC_TORQUE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Monitors the straightened voltage signal sent through an AD620 microvolt amplifier to
 * detect the direction and the magnitude of the rotational force measured by the managed torque sensor.
 *
 * @note By default, the sensor interprets torque in the counterclockwise (CCW) direction as positive and torque in the
 * clockwise (CW) direction as negative.
 *
 * @tparam kPin the analog pin connected to the signal terminal of the AD620 millivolt amplifier.
 * @tparam kBaseline the value, in 12-bit Analog-to-Digital Converter (ADC) units, of the received signal interpreted
 * as 0 torque.
 * @tparam kInvertDirection determines whether to invert the sign of the value returned by the sensor.
 */
template <const uint8_t kPin, const uint16_t kBaseline, const bool kInvertDirection = false>
class TorqueModule final : public Module
{
        // Ensures that the torque sensor pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different pin for the TorqueModule "
            "instance."
        );

    public:

        ///Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kCCWTorque = 51,  ///< The sensor detects torque in the counterclockwise (CCW) direction.
            kCWTorque  = 52,  ///< The sensor detects torque in the clockwise (CW) direction.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Determines the direction and magnitude of the detected torque since the last check.
        };

        /// Initializes the base Module class.
        TorqueModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the module's runtime parameters structure with the data received from the PC.
        bool SetCustomParameters() override
        {
            return _communication.ExtractModuleParameters(_custom_parameters);
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // Sets pin to Input mode.
            pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW        = true;
            _custom_parameters.report_CW         = true;
            _custom_parameters.signal_threshold  = 100;
            _custom_parameters.delta_threshold   = 70;
            _custom_parameters.average_pool_size = 5;

            // Notifies the PC about the initial sensor state.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque),  // Direction is not relevant for the 0 value
                kPrototypes::kOneUint16,
                0
            );

            return true;
        }

        ~TorqueModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                bool report_CCW           = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW            = true;  ///< Determines whether to report changes in the CW direction.
                uint16_t signal_threshold = 100;   ///< The minimum deviation from the baseline report torque changes.
                uint16_t delta_threshold  = 70;    ///< The minimum signal difference report torque changes.
                uint8_t average_pool_size = 5;     ///< The number of readouts to average when computing torque.
        } PACKED_STRUCT _custom_parameters;

        /// Reads the direction and magnitude of the torque relative recorded by the sensor and sends it to
        /// the PC if it is significantly different from the previous readout.
        void CheckState()
        {
            // Stores the previous readout of the analog pin.
            static uint16_t previous_readout = kBaseline;  // NOLINT(*-dynamic-static-initializers)

            // Tracks whether the previous message sent to the PC included a zero torque value.
            static bool previous_zero = true;  // A zero-message is sent at class initialization.

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value.
            uint16_t signal = AnalogRead(kPin, _custom_parameters.average_pool_size);

            // Calculates the absolute difference between the current signal and the previous readout.
            const uint16_t delta = abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout));

            // Prevents reporting signals that are not significantly different from the previous readout value.
            if (delta <= _custom_parameters.delta_threshold)
            {
                CompleteCommand();
                return;
            }

            previous_readout = signal;  // Overwrites the previous readout with the current signal.

            // Determines the direction of the signal. Signals above the baseline are interpreted as CCW, signals below
            // the baseline as CW. Also rescales the signal as necessary for its value to always mean the same things: 0
            // is no torque, baseline is maximum torque, regardless of direction.
            bool cw;
            if (signal > kBaseline)
            {
                // For CCW signals, subtracts the baseline from the signal to ensure the signal is always scaled between
                // 0 and baseline. CCW torque increases as the level of the signal increases. Therefore, the larger
                // the signal, the higher the torque.
                cw     = false ^ kInvertDirection;
                signal = signal - kBaseline;
            }
            else if (signal < kBaseline)
            {
                // For CW signals, subtracts the signal from the baseline. CW torque increases as the level of
                // the signal decreases. Therefore, the smaller the signal, the higher the torque. This rescaling
                // 'shifts' the signal to increase proportionally to CW torque, from 0 to baseline.
                cw     = true ^ kInvertDirection;
                signal = kBaseline - signal;
            }
            else
            {
                // If the signal is 0, the direction does not really matter
                cw     = false;  // Uses CCW for zero-torque reporting
                signal = 0;
            }

            // If the signal is below the threshold, pulls it to 0 and notifies the PC. The torque direction in this
            // case does not matter
            if (signal < _custom_parameters.signal_threshold)
            {
                // Ensures sub-threshold values are only sent to the PC once if multiple zero-values occur in succession
                if (!previous_zero)
                {
                    SendData(
                        static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque),  // Uses CCW for 0-torque reporting
                        kPrototypes::kOneUint16,
                        0
                    );
                    previous_zero = true;
                }
            }
            else
            {
                // Sends the detected signal to the PC using the event-code to code for the direction and the
                // signal value to provide the absolute directional torque value in raw ADC units. Only sends the data
                // if the class is configured to report changes in that direction
                if (!cw && _custom_parameters.report_CCW)
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque), kPrototypes::kOneUint16, signal);
                }
                else if (cw && _custom_parameters.report_CW)
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kCWTorque), kPrototypes::kOneUint16, signal);
                }

                previous_zero = false;
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_TORQUE_MODULE_H
