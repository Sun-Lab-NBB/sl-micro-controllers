/**
 * @file
 *
 * @brief Provides the TTLModule class that supports Transistor-to-Transistor Logic (TTL) communication with other
 * hardware systems by sending or receiving TTL logic pulses.
 */

#ifndef AXMC_TTL_MODULE_H
#define AXMC_TTL_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Sends or receives Transistor-to-Transistor Logic (TTL) signals using the specified digital pin.
 *
 * @warning Each class instance can either receive or output TTL signals, it cannot do both at the same time!
 *
 * @tparam kPin the digital pin used to output or receive TTL signals.
 * @tparam kOutput determines whether the instance is used to send TTL signals (if true) or receive TTL signals
 * (if false).
 * @tparam kStartOn determines the initial state of the output pin when the class is configured to output TTL signals.
 */
template <const uint8_t kPin, const bool kOutput = true, const bool kStartOn = false>
class TTLModule final : public Module
{
        static_assert(
            kPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different pin for the TTLModule instance."
        );

    public:

        /// Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kInputOn        = 51,  ///< The input ttl pin is receiving a HIGH signal.
            kInputOff       = 52,  ///< The input ttl pin is receiving a LOW signal.
            kInvalidPinMode = 53,  ///< The instance's pin mode is not valid for the requested command.
            kOutputOn       = 54,  ///< The output ttl pin is set to HIGH.
            kOutputOff      = 55,  ///< The output ttl pin is set to LOW.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse  = 1,  ///< Sends a ttl pulse through the output pin.
            kToggleOn   = 2,  ///< Sets the output pin to HIGH.
            kToggleOff  = 3,  ///< Sets the output pin to LOW.
            kCheckState = 4,  ///< Checks the state of the input pin.
        };

        /// Initializes the base Module class.
        TTLModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
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
                // SendPulse
                case kModuleCommands::kSendPulse: SendPulse(); return true;
                // ToggleOn
                case kModuleCommands::kToggleOn: ToggleOn(); return true;
                // ToggleOff
                case kModuleCommands::kToggleOff: ToggleOff(); return true;
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // Depending on the output flag, configures the pin as either input or output.
            if (kOutput)
            {
                pinModeFast(kPin, OUTPUT);

                // Depending on the class configuration, initializes the pin to the desired state.
                if (!kStartOn)
                {
                    digitalWriteFast(kPin, LOW);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOff));
                }
                else
                {
                    digitalWriteFast(kPin, HIGH);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOn));
                }
            }
            else
            {
                pinModeFast(kPin, INPUT);

                // Notifies the PC about the initial sensor state, which is always reported as a zero-value baseline.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOff));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 10000;  // 10000 microseconds == 10 milliseconds.
            _custom_parameters.average_pool_size = 0;      // 0 or 1 == no averaging.

            return true;
        }

        ~TTLModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration   = 10000;  ///< The time, in microseconds, the pin outputs HIGH during pulses.
                uint8_t average_pool_size = 0;  ///< The number of digital readouts to average when checking pin state.
        } PACKED_STRUCT _custom_parameters;

        /// Sets the output pin to HIGH for the requested pulse_duration of microseconds and then sets it LOW.
        void SendPulse()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();
                return;
            }

            switch (execution_parameters.stage)
            {
                // Initializes the pulse
                case 1:
                    digitalWriteFast(kPin, HIGH);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOn));
                    AdvanceCommandStage();
                    return;

                // Delays for the requested number of microseconds.
                case 2:
                    if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Inactivates the pulse.
                case 3:
                    digitalWriteFast(kPin, LOW);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOff));
                    CompleteCommand();

                default: AbortCommand();
            }
        }

        /// Sets the output pin to emit the HIGH signal.
        void ToggleOn()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();
                return;
            }

            // Sets the pin to HIGH and finishes command execution
            digitalWriteFast(kPin, HIGH);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOn));
            CompleteCommand();
        }

        /// Sets the output pin to emit the LOW signal.
        void ToggleOff()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();
                return;
            }

            // Sets the pin to LOW and finishes command execution
            digitalWriteFast(kPin, LOW);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOff));
            CompleteCommand();
        }

        /// Checks the state of the input pin and sends it to the PC if it is significantly different from the
        /// previous readout.
        void CheckState()
        {
            // Tracks the previous input_pin status. This is used to optimize data transmission by only reporting input
            // pin state changes to the PC.
            static bool previous_input_status = false;

            // Calling this command when the class is configured to output TTL pulses triggers an invalid
            // pin mode error.
            if (kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
                return;
            }

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // state-value. To optimize communication, only sends data to the PC if the state has changed.
            if (const bool current_state = DigitalRead(kPin, _custom_parameters.average_pool_size);
                previous_input_status != current_state)
            {
                // Updates the state tracker.
                previous_input_status = current_state;

                // If the state is true, sends InputOn message, otherwise sends InputOff message.
                if (current_state) SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOn));
                else SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOff));
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_TTL_MODULE_H
