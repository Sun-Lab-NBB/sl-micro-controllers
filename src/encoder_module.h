/**
 * @file
 * @brief Provides the EncoderModule class that monitors and records the data produced by a quadrature encoder.
 *
 * @warning This file is written in a way that is @b NOT compatible with any other library or class that uses
 * AttachInterrupt(). Disable the 'ENCODER_USE_INTERRUPTS' macro defined at the top of the file to make this file
 * compatible with other interrupt libraries.
 */

#ifndef AXMC_ENCODER_MODULE_H
#define AXMC_ENCODER_MODULE_H

// Note, this definition has to precede Encoder.h inclusion. This increases the resolution of the encoder, but
// interferes with any other library that makes use of AttachInterrupt() function.
#define ENCODER_USE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include <axmc_shared_assets.h>
#include <module.h>

/**
 * @brief Wraps an Encoder class instance and provides access to its pulse counter to monitor the direction and
 * magnitude of the rotation measured by the managed quadrature encoder.
 *
 * @warning Both Pin A and Pin B must be hardware interrupt pins to achieve the maximum encoder state readout
 * resolution.
 *
 * @note By default, the clockwise (CW) movement of the encoder is measured as a negative displacement, while the
 * counterclockwise (CCW) movement of the encoder is measured as a positive displacement.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @tparam kPinX the digital pin connected to the 'X' (index) channel of the quadrature encoder.
 * @tparam kInvertDirection determines whether to invert the sign of the value returned by the encoder.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const uint8_t kPinX, const bool kInvertDirection = false>
class EncoderModule final : public Module
{
        // Ensures that the encoder pins are different.
        static_assert(kPinA != kPinB, "EncoderModule PinA and PinB cannot be the same!");
        static_assert(kPinA != kPinX, "EncoderModule PinA and PinX cannot be the same!");
        static_assert(kPinB != kPinX, "EncoderModule PinB and PinX cannot be the same!");

        // Also ensures that encoder pins do not interfere with LED pin.
        static_assert(
            kPinA != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Channel A pin for the "
            "EncoderModule instance."
        );
        static_assert(
            kPinB != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Channel B pin for the "
            "EncoderModule instance."
        );
        static_assert(
            kPinX != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different Index (X) pin for the "
            "EncoderModule instance."
        );

    public:
        ///Defines the codes used by each module instance to communicate its runtime state to the PC.
        enum class kCustomStatusCodes : uint8_t
        {
            kRotatedCCW = 51,  ///< The encoder was rotated in the counterclockwise (CCW) direction.
            kRotatedCW  = 52,  ///< The encoder was rotated in the clockwise (CW) direction.
            kPPR        = 53,  ///< Communicates the estimated Pulse-Per-Revolution (PPR) value of the encoder.
        };

        /// Defines the codes for the commands supported by the module's instance.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Determines the direction and magnitude of the encoder's motion since the last check.
            kReset      = 2,  ///< Resets the encoder's pulse tracker to 0.
            kGetPPR     = 3,  ///< Estimates the encoder's Pulse-Per-Revolution (PPR) value.
        };

        /// Initializes the base Module class.
        EncoderModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the module's runtime parameters structure with the data received from the PC.
        bool SetCustomParameters() override
        {
            if (_communication.ExtractModuleParameters(_custom_parameters))
            {
                // Uses the delta_threshold to calculate positive and negative amortization. Amortization allows the
                // overflow to store pulses in the non-reported direction, up to the delta_threshold limit. This is
                // used to counter (amortize) small, insignificant movements. For example, if the running wheel
                // is locked, it may still move slightly in either direction. Without amortization, the overflow
                // eventually accumulates small forward jitters and erroneously report forward movement. With
                // amortization, the positive movement is likely to be counteracted by negative movement, resulting in
                // a net 0 movement.
                _positive_amortization = static_cast<int32_t>(_custom_parameters.delta_threshold);
                _negative_amortization = -_positive_amortization;
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
                // ReadEncoder
                case kModuleCommands::kCheckState: ReadEncoder(); return true;
                // ResetEncoder
                case kModuleCommands::kReset: ResetEncoder(); return true;
                // GetPPR
                case kModuleCommands::kGetPPR: GetPPR(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets the module instance's software and hardware parameters to the default values.
        bool SetupModule() override
        {
            // Since the wrapped Encoder class does not manage the Index pin, configures it as an input pin.
            pinMode(kPinX, INPUT);

            // Resets the encoder's pulse counter. The hardware is statically configured during Encoder class
            // instantiation.
            _encoder.write(0);

            // Resets the overflow tracker
            _overflow = 0;

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW      = true;
            _custom_parameters.report_CW       = true;
            _custom_parameters.delta_threshold = 15;

            // Notifies the PC about the initial sensor state.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kRotatedCW),  // Direction is not relevant for 0-value.
                kPrototypes::kOneUint32,
                0
            );

            return true;
        }

        ~EncoderModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                bool report_CCW          = true;  ///< Determines whether to report rotation in the CCW direction.
                bool report_CW           = true;  ///< Determines whether to report rotation in the CW direction.
                uint32_t delta_threshold = 15;    ///< The minimum displacement change (delta) for reporting rotation.
        } PACKED_STRUCT _custom_parameters;

        /// The encoder class that monitors the encoder's rotation.
        Encoder _encoder = Encoder(kPinA, kPinB);  // HAS to be initialized statically or the runtime crashes!

        /// Stores the multiplier used to optionally invert the pulse counter sign to virtually flip the direction of
        /// encoder readouts.
        static constexpr int32_t kMultiplier = kInvertDirection ? -1 : 1;  // NOLINT(*-dynamic-static-initializers)

        /// Accumulates insignificant encoder readouts to be reused during future encoder state evaluation calls.
        int32_t _overflow = 0;

        /// Determines the maximum encoder displacement (rotation) in the counterclockwise (CCW) that can be accumulated
        /// in the _overflow attribute during runtime when reporting the CCW rotation is disabled.
        int32_t _positive_amortization = 0;

        /// Determines the maximum encoder displacement (rotation) in the clockwise (CW) that can be accumulated in the
        /// _overflow attribute during runtime when reporting the CW rotation is disabled.
        int32_t _negative_amortization = 0;

        /// Reads the direction and magnitude of the encoder's rotation relative to the previous check and sends it to
        /// the PC if it is significantly different from the previous readout.
        void ReadEncoder()
        {
            // Retrieves and, if necessary, flips the direction of the displacement (rotation) recorded by the encoder.
            const int32_t new_motion = _encoder.readAndReset() * kMultiplier;

            // If the encoder has not moved since the last call to this method, returns without further processing.
            if (new_motion == 0)
            {
                CompleteCommand();
                return;
            }

            // Combines the new motion with the accumulated motion stored in the overflow tracker.
            _overflow += new_motion;

            if (new_motion < 0 && !_custom_parameters.report_CW)
            {
                // Caps the accumulated CW motion (negative) to the amortization threshold when not reporting CW
                // direction.
                _overflow = max(_overflow, _negative_amortization);
            }
            else if (new_motion > 0 && !_custom_parameters.report_CCW)
            {
                /// Caps the accumulated CCW motion (positive) to the amortization threshold when not reporting CCW
                // direction.
                _overflow = min(_overflow, _positive_amortization);
            }

            // Converts the pulse count delta to an absolute value for the threshold checking below.
            auto delta = static_cast<uint32_t>(abs(_overflow));

            // Uses the direction (sign) of the displacement to determine the event code to use when reporting the
            // Displacement to the PC. Only reports displacements that are above the delta threshold.
            if (_overflow < 0 && delta > _custom_parameters.delta_threshold)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kRotatedCW), kPrototypes::kOneUint32, delta);
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
            }

            // If the value is positive, this is interpreted as the CCW movement direction.
            // Same as above, if the delta is greater than or equal to the readout threshold, sends the data to the PC.
            else if (_overflow > 0 && delta > _custom_parameters.delta_threshold)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kRotatedCCW), kPrototypes::kOneUint32, delta);
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
            }

            // Completes the command execution
            CompleteCommand();
        }

        /// Resets the encoder pulse counter back to 0.
        void ResetEncoder()
        {
            _encoder.write(0);
            CompleteCommand();
        }

        /// Estimates the Pulse-Per-Revolution (PPR) value of the encoder by using the index pin to precisely measure
        /// the number of pulses emitted during the full (360-degree) encoder rotation.
        void GetPPR()
        {
            // First, establishes the measurement baseline. Since the algorithm does not know the current position of
            // the encoder, waits until the index pin is triggered. This is used to establish the baseline for tracking
            // the pulses per rotation.
            while (!digitalReadFast(kPinX))
            {
            }
            _encoder.write(0);  // Resets the pulse tracker to 0

            // Measures 10 full rotations (indicated by index pin pulses). Resets the pulse tracker to 0 at each
            // measurement and does not care about the motion direction.
            uint32_t pprs = 0;
            for (uint8_t i = 0; i < 10; ++i)
            {
                // Delays for 100 milliseconds to ensure the object spins past the range of index pin trigger
                delay(100);

                // Blocks until the index pin is triggered.
                while (!digitalReadFast(kPinX))
                {
                }

                // Accumulates the pulse counter into the summed value and reset the encoder each call.
                pprs += abs(_encoder.readAndReset());
            }

            // Computes the average PPR by using half-up rounding to get a whole number.
            const uint32_t average_ppr = (pprs + 10 / 2) / 10;

            // Sends the average PPR count to the PC.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kPPR),
                kPrototypes::kOneUint32,
                average_ppr
            );

            // Completes the command execution
            CompleteCommand();
        }
};

#endif  //AXMC_ENCODER_MODULE_H
