#ifndef CCTL_NUMERICAL_SOLVER_RUNGE_KUTTA_4_HPP
#define CCTL_NUMERICAL_SOLVER_RUNGE_KUTTA_4_HPP

#include <stdexcept>

namespace cctl
{

/**
 * @brief Advance a zero-order-held model by one classical fourth-order RK step.
 *
 * Input is held constant across k1..k4, while state-dependent effects inside
 * Model::derivative are recalculated at every intermediate state.
 */
template <typename Model>
inline void runge_kutta_4_step(const Model &model, typename Model::scalar_type time,
                               typename Model::scalar_type dt, typename Model::state_type &state,
                               const typename Model::input_type &input)
{
    typedef typename Model::scalar_type scalar_type;
    typedef typename Model::state_type state_type;

    if (!(dt > scalar_type(0)))
        throw std::invalid_argument("Runge-Kutta step must be positive");

    const scalar_type half_dt = dt * scalar_type(0.5);
    const state_type k1 = model.derivative(time, state, input);
    const state_type k2 = model.derivative(time + half_dt, state + k1 * half_dt, input);
    const state_type k3 = model.derivative(time + half_dt, state + k2 * half_dt, input);
    const state_type k4 = model.derivative(time + dt, state + k3 * dt, input);

    state += (k1 + k2 * scalar_type(2) + k3 * scalar_type(2) + k4) * (dt / scalar_type(6));
}

} // namespace cctl

#endif // CCTL_NUMERICAL_SOLVER_RUNGE_KUTTA_4_HPP
