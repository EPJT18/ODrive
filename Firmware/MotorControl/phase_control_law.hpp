#ifndef __PHASE_CONTROL_LAW_HPP
#define __PHASE_CONTROL_LAW_HPP

#include <autogen/interfaces.hpp>
#include <variant>

template<size_t N_PHASES>
class PhaseControlLaw {
public:
    /**
     * @brief Shall calculate the PWM timings based on the currently available
     * current measurements.
     * 
     * This function gets called in a high priority interrupt context and should
     * run fast.
     * 
     * @param vbus_voltage: The most recently measured DC link voltage. NAN if
     *        the measurement is not available or valid for some reason.
     * @param currents: The most recently measured (or inferred) phase currents
     *        in Amps. Any of the values can be NAN if the measurement is not
     *        available or valid for some reason.
     * @param input_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the vbus_voltage and current measurement.
     * @param output_timestamp: The timestamp (in HCLK ticks) corresponding to
     *        the middle of the time span during which the output will be
     *        active.
     * 
     * @returns: This function shall return an array of PWM timings, each item
     *           corresponding to one phase. Each of the PWM values must lie in
     *           0.0f...1.0f.
     *           If the function returns std::nullopt the motor gets disarmed.
     */
    virtual std::variant<std::array<float, N_PHASES>, ODriveIntf::MotorIntf::Error> calculate(
        float vbus_voltage, std::array<float, N_PHASES> currents,
        uint32_t input_timestamp, uint32_t output_timestamp) = 0;
};

#endif // __PHASE_CONTROL_LAW_HPP