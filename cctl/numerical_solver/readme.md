# CCTL numerical solvers

**English** | [简体中文](readme_cn.md)

This directory provides fixed-step C++ numerical solvers for host-side real-time and offline simulation.

- `fixed_vector.hpp` provides an inline, heap-free `std::array`-backed vector whose short loops can be unrolled and auto-vectorized.
- `fixed_point.hpp` provides a saturating signed 32-bit Q-format scalar with
  64-bit arithmetic intermediates for generated MNA fixed-point models.
- `explicit_euler.hpp` implements one explicit Euler step.
- `runge_kutta_4.hpp` implements one classical fourth-order Runge–Kutta step.

Models declare `scalar_type`, `state_type`, and `input_type`, and expose a const `derivative(time, state, input)` function. Inputs are zero-order held for one step; RK4 recalculates state-dependent effects at each intermediate state.

`fixed_point32<FractionalBits>` is the arithmetic primitive, not a range-analysis
policy. CCTL Studio's MNA generator selects power-of-two per-unit bases, embeds
per-matrix-family coefficient Q formats, uses 64-bit mixed-Q dot accumulators,
converts external inputs at the circuit boundary, and
converts public outputs back to floating point. Production designs should still
provide validated input full-scale ranges and compare the fixed-point result
against the Eigen reference model.
