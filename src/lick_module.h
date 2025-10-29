/**
 * @file
 *
 * @brief Provides the LickModule class that monitors and records the data produced by a conductive lick sensor.
 */

#ifndef AXMC_LICK_MODULE_H
#define AXMC_LICK_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Monitors the voltage fluctuations measured by a conductive lick sensor to detect interactions with the
 * sensor's circuitry.
 *
 * @tparam kPin the analog pin connected to the output terminal of the lick sensor.
 */
template <const uint8_t kPin>
class LickModule final : public Module
{
        // Ensures that the pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different pin for the "
            "LickModule instance."
        );

    public:

        /// Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kChanged = 51,  /// The sensor has experienced a significant change in the conducted voltage level.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the voltage level across the sensor.
        };

        /// Initializes the base Module class.
        LickModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
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
            // Configures the input pin. Critically, uses the pull-down mode, as the sensor is expected to spend most
            // of its runtime in an uncompleted circuit state, so the pin must be pulled to 0.
            pinModeFast(kPin, INPUT_PULLDOWN);

            // Resets the custom_parameters structure fields to their default values. Assumes 12-bit ADC resolution.
            _custom_parameters.signal_threshold  = 300;  // Ideally should be just high enough to filter out noise
            _custom_parameters.delta_threshold   = 300;  // Ideally should be at least half of the minimal threshold
            _custom_parameters.average_pool_size = 0;    // Better to have at 0 because Teensy already does this

            // Notifies the PC about the initial sensor state.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kChanged), kPrototypes::kOneUint16, 0);

            return true;
        }

        ~LickModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint16_t signal_threshold = 300;  ///< The minimum voltage level to report to the PC.
                uint16_t delta_threshold  = 300;  ///< The minimum change in voltage level readouts to report to the PC.
                uint8_t average_pool_size = 0;    ///< The number of readouts to average to determine the voltage level.
        } PACKED_STRUCT _custom_parameters;

        /// Checks the voltage level across the sensor's circuitry and sends it to the PC if it is significantly
        /// different from the previous readout.
        void CheckState()
        {
            // Stores the previous readout of the analog pin.
            static uint16_t previous_readout = 0;

            // Tracks whether the previous message sent to the PC included a zero-signal value.
            static bool previous_zero = false;

            // Reads the current voltage level across the sensor's circuitry.
            const uint16_t signal = AnalogRead(kPin, _custom_parameters.average_pool_size);

            // Calculates the absolute difference between the current readout and the previous readout.
            const auto delta =
                static_cast<uint16_t>(abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout)));

            // Prevents reporting signals that are not significantly different from the previous readout value.
            if (delta <= _custom_parameters.delta_threshold)
            {
                CompleteCommand();
                return;
            }

            previous_readout = signal;  // Overwrites the previous readout with the current signal

            // If the signal is above the threshold, sends it to the PC
            if (signal >= _custom_parameters.signal_threshold)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kChanged), kPrototypes::kOneUint16, signal);
                previous_zero = false;
            }

            // If the signal is below the threshold and the previously reported signal was not zero, pulls the signal
            // to zero and reports it to the PC.
            else if (!previous_zero)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kChanged),
                    kPrototypes::kOneUint16,
                    0
                );
                previous_zero = true;
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_LICK_MODULE_H
