#ifndef CCTL_LEGACY_PMSM_MODEL_HPP
#define CCTL_LEGACY_PMSM_MODEL_HPP

// Compatibility entry point for the historical constant-flux PMSM model.
// New code should include motor_model/pmsm_average_model.hpp directly.
#include <cctl/power_electronics_objects/motor_model/pmsm_average_model.hpp>

template <typename T> using st_pmsm_motor = cctl::fixed_vector<T, 4U>;
template <typename T> using diff_pmsm_motor = cctl::pmsm_average_model<T>;

#endif // CCTL_LEGACY_PMSM_MODEL_HPP
