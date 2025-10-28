/**
 * @file
 * @brief Provides the BreakModule class that controls an electromagnetic break module.
 */

#ifndef AXMC_BREAK_MODULE_H
#define AXMC_BREAK_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Controls the electromagnetic break by sending Pulse-Width-Modulated (PWM) currents through the break.
 *
 * This module sends digital or analog signals that trigger Field-Effect-Transistor (FET) gated relay hardware to
 * deliver voltage that variably engages the break.
 *
 * @tparam kPin the analog pin connected to the break FET-gated relay.
 * @tparam kNormallyEngaged determines whether the break is engaged (active) or disengaged (inactive) when unpowered.
 * @tparam kStartEngaged determines the initial state of the break during class initialization.
 */
template <const uint8_t kPin, const bool kNormallyEngaged, const bool kStartEngaged = true>
class BreakModule final : public Module
{
        // Ensures that the output pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for the BreakModule instance."
        );

    public:
        /**
         * @brief Defines the codes used by each module instance to communicate its runtime state to the PC.
         */
        enum class kCustomStatusCodes : uint8_t
        {
            kEngaged    = 52,  ///< The break is engaged at maximum possible strength.
            kDisengaged = 53,  ///< The break is disengaged.
            kVariable   = 54,  ///< The break is engaged at the specified non-maximal strength.
        };

        /**
         * @brief Defines the codes for the commands supported by the module's instance.
         */
        enum class kModuleCommands : uint8_t
        {
            kToggleOn         = 1,  ///< Engages the break at maximum strength.
            kToggleOff        = 2,  ///< Disengages the break.
            kSetBreakingPower = 3,  ///< Sets the break to engage at the requested breaking strength.
        };

        /// Initializes the base Module class.
        BreakModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /**
         * @brief Overwrites the module's runtime parameters structure with the data received from the PC.
         */
        bool SetCustomParameters() override
        {
            if (_communication.ExtractModuleParameters(_custom_parameters))
            {
                // Adjusts the PWM value to account for whether the break is normally engaged. This ensures that the
                // strength of 255 means the break is fully engaged.
                uint8_t value = _custom_parameters.breaking_strength;
                if (kNormallyEngaged) value = 255 - value;
                _custom_parameters.breaking_strength = value;
                return true;  // Extraction (and adjustment) succeeded.
            }
            return false;  // Returns false if the parameter extraction fails
        }

        /**
         * @brief Resolves and executes the currently active command.
         *
         * @return bool true of the module has recognized the currently active command and false otherwise.
         */
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // EnableBreak
                case kModuleCommands::kToggleOn: EnableBreak(); return true;
                // DisableBreak
                case kModuleCommands::kToggleOff: DisableBreak(); return true;
                // SetBreakingPower
                case kModuleCommands::kSetBreakingPower: SetBreakingPower(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /**
         * @brief Sets the module instance's software and hardware parameters to the default values.
         *
         * @return true for all expected runtime scenarios.
         */
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinModeFast(kPin, OUTPUT);

            // Based on the requested initial break state and the configuration of the break (normally engaged or not),
            // either engages or disengages the breaks following setup.
            if (kStartEngaged)
            {
                digitalWriteFast(kPin, kEngage);  // Ensures the break is engaged.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kEngaged));
            }
            else
            {
                digitalWriteFast(kPin, kDisengage);  // Ensures the break is disengaged.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kDisengaged));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.breaking_strength = 128;  //  50% breaking strength

            return true;
        }

        ~BreakModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint8_t breaking_strength = 128;  ///< Determines the strength of the break in variable mode.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the digital signal that needs to be sent to the output pin to engage the break at maximum strength.
        static constexpr bool kEngage = kNormallyEngaged ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to disengage the break.
        static constexpr bool kDisengage = kNormallyEngaged ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Engages the break at the maximum strength.
        void EnableBreak()
        {
            digitalWriteFast(kPin, kEngage);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kEngaged));
            CompleteCommand();
        }

        /// Disengages the break
        void DisableBreak()
        {
            digitalWriteFast(kPin, kDisengage);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kDisengaged));
            CompleteCommand();
        }

        /// Engages the break at the specified strength level.
        void SetBreakingPower()
        {
            // Uses AnalogWrite to make the pin output a square wave pulse with the desired duty cycle (PWM). This
            // results in the breaks being applied a certain proportion of time, producing the desired breaking power.
            analogWrite(kPin, _custom_parameters.breaking_strengthl);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kVariable));
            CompleteCommand();
        }
};

#endif  //AXMC_BREAK_MODULE_H
