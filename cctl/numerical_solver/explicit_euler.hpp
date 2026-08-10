#ifndef CCTL_NUMERICAL_SOLVER_EXPLICIT_EULER_HPP
#define CCTL_NUMERICAL_SOLVER_EXPLICIT_EULER_HPP

#include <stdexcept>

namespace cctl
{

/**
 * @brief Advance an autonomous or zero-order-held model by one Euler step.
 *
 * Model must expose scalar_type, state_type, input_type and
 * derivative(time, state, input).  The model is intentionally passed by const
 * reference so evaluating a derivative cannot mutate simulation state.
 */
template <typename Model>
inline void explicit_euler_step(const Model &model, typename Model::scalar_type time,
                                typename Model::scalar_type dt, typename Model::state_type &state,
                                const typename Model::input_type &input)
{
    if (!(dt > typename Model::scalar_type(0)))
        throw std::invalid_argument("Euler step must be positive");
    state += model.derivative(time, state, input) * dt;
}

} // namespace cctl

#endif // CCTL_NUMERICAL_SOLVER_EXPLICIT_EULER_HPP
