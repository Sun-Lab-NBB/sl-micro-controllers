/**
 * @file
 *
 * @brief Provides the ScreenModule class that controls the screen power boards used in the Sun lab's Virtual
 * Reality system.
 */

#ifndef SCREEN_MODULE_H
#define SCREEN_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Switches the screen power state by sending digital currents to the FET gate that shorts the power board's
 * button terminals.
 *
 * @tparam kLeftScreenPin the digital pin connected to the logic terminal of the left VR screen's power board FET gate.
 * @tparam kCenterScreenPin the digital pin connected to the logic terminal of the center VR screen's power board FET
 * gate.
 * @tparam kRightScreenPin the digital pin connected to the logic terminal of the right VR screen's power board FET
 * gate.
 * @tparam kNormallyClosed determines whether the FET relays used to control the screens' power are closed
 * (On / conducting) or opened (Off / not conducting) when unpowered.
 */
template <
    const uint8_t kLeftScreenPin,
    const uint8_t kCenterScreenPin,
    const uint8_t kRightScreenPin,
    const bool kNormallyClosed = false>
class ScreenModule final : public Module
{
        static_assert(
            kLeftScreenPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Left screen pin for "
            "the ScreenModule instance."
        );
        static_assert(
            kCenterScreenPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Center screen pin for "
            "the ScreenModule instance."
        );
        static_assert(
            kRightScreenPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Right screen pin for "
            "the ScreenModule instance."
        );

    public:

        /// Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kOn  = 51,  ///< The screen power board relay is receiving a HIGH signal.
            kOff = 52,  ///< The screen power board relay is receiving a LOW signal.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kToggle = 1,  ///< Pulses the screens' power board FET gates simulate pressing the screens' power button.
        };

        /// Initializes the base Module class.
        ScreenModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
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
                // Toggles screen power state
                case kModuleCommands::kToggle: Toggle(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // Sets the control pins to output mode.
            pinModeFast(kLeftScreenPin, OUTPUT);
            pinModeFast(kCenterScreenPin, OUTPUT);
            pinModeFast(kRightScreenPin, OUTPUT);

            // Ensures the logic gates are disabled at startup
            digitalWriteFast(kLeftScreenPin, kOff);
            digitalWriteFast(kCenterScreenPin, kOff);
            digitalWriteFast(kRightScreenPin, kOff);

            // Notifies the PC about the initial state of the FET gates.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration = 1000000;  // 1000000 microseconds == 1 second.

            return true;
        }

        ~ScreenModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 1000000;  ///< The time of the toggling pulse's HIGH phase, in microseconds.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the digital signal that needs to be sent to the output pin to simulate pressing the screens' power
        /// button.
        static constexpr bool kOn = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the output pin to simulate releasing the screens' power
        /// button.
        static constexpr bool kOff = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Sends a digital pulse through all output pins, using the preconfigured pulse_duration of microseconds, to
        /// simulate pressing and releasing the screens' power button.
        void Toggle()
        {
            switch (execution_parameters.stage)
            {
                // Simulates pressing the screens' power button
                case 1:
                    digitalWriteFast(kLeftScreenPin, kOn);
                    digitalWriteFast(kCenterScreenPin, kOn);
                    digitalWriteFast(kRightScreenPin, kOn);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOn));
                    AdvanceCommandStage();
                    return;

                // Simulates holding the button down. This allows the power boards logic to debounce and detect the
                // signal.
                case 2:
                    if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Simulates releasing the screens' power button
                case 3:
                    // Sets all screen pins to the OFF state
                    digitalWriteFast(kLeftScreenPin, kOff);
                    digitalWriteFast(kCenterScreenPin, kOff);
                    digitalWriteFast(kRightScreenPin, kOff);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }
};

#endif  //SCREEN_MODULE_H
