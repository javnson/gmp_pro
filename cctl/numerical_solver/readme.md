# CCTL numerical solvers

**English** | [简体中文](readme_cn.md)

This directory provides fixed-step C++ numerical solvers for host-side real-time and offline simulation.

- `fixed_vector.hpp` provides an inline, heap-free `std::array`-backed vector whose short loops can be unrolled and auto-vectorized.
- `explicit_euler.hpp` implements one explicit Euler step.
- `runge_kutta_4.hpp` implements one classical fourth-order Runge–Kutta step.

Models declare `scalar_type`, `state_type`, and `input_type`, and expose a const `derivative(time, state, input)` function. Inputs are zero-order held for one step; RK4 recalculates state-dependent effects at each intermediate state.
