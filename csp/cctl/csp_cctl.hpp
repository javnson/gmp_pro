/**
 * @file csp_cctl.hpp
 * @brief Hosted CCTL simulation runtime and project integration contract.
 */

#ifndef GMP_CSP_CCTL_HOST_SIMULATION_HPP
#define GMP_CSP_CCTL_HOST_SIMULATION_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <string>

namespace gmp::csp::cctl
{

class simulation_runtime;

/** @brief Immutable coordinates and host services for one GMP simulation loop. */
struct simulation_step_context
{
    std::size_t step_index;
    double time_s;
    simulation_runtime &runtime;
};

/**
 * @brief Deterministic compute-budget scheduler for a simulated MCU task.
 *
 * Unlike an integer divider, this phase accumulator supports task frequencies
 * that are not integer divisors of the plant rate (for example 33 kHz on a
 * 100 ns simulation step). consume() returns the number of executions granted
 * on the current plant step.
 */
class compute_budget_scheduler
{
  public:
    /** Construct and initialize one task-frequency phase accumulator. */
    compute_budget_scheduler(double simulation_step_s,
                             double task_frequency_hz,
                             bool dispatch_at_start = true);
    /** Reset the scheduler for a simulation step and task frequency. */
    void initialize(double simulation_step_s, double task_frequency_hz,
                    bool dispatch_at_start = true);
    /** @return Executions granted on the current simulation step. */
    std::size_t consume() noexcept;
    /** @return Total executions granted since the last initialization. */
    std::size_t total_executions() const noexcept;

  private:
    long double executions_per_step_{};
    long double phase_{};
    std::size_t total_executions_{};
    bool dispatch_at_start_{};
    bool first_step_{};
};

/** @brief Standard hosted model of the embedded processor/control domain. */
class embedded_chip_simulation
{
  public:
    /** Allow polymorphic destruction by the common simulation system. */
    virtual ~embedded_chip_simulation() = default;
    /** Initialize simulated MCU state and compute schedulers. */
    virtual void initialize_embedded_chip() = 0;
    /** Spend background compute budget for one plant step. */
    virtual void step_embedded_chip(const simulation_step_context &context) = 0;
    /** Finalize the simulated MCU after peripheral and plant shutdown. */
    virtual void finalize_embedded_chip() = 0;
};

/** @brief Standard hosted model of ADC, PWM, encoder, and other peripherals. */
class peripheral_simulation
{
  public:
    /** Allow polymorphic destruction by the common simulation system. */
    virtual ~peripheral_simulation() = default;
    /** Initialize all simulated peripheral state. */
    virtual void initialize_peripherals() = 0;
    /** Apply controller-visible peripheral outputs before plant stepping. */
    virtual void apply_peripheral_outputs(
        const simulation_step_context &context) = 0;
    /** Sample plant outputs and synchronously dispatch pending interrupts. */
    virtual void sample_peripheral_inputs(
        const simulation_step_context &context) = 0;
    /** Finalize peripherals after circuit shutdown. */
    virtual void finalize_peripherals() = 0;
};

/** @brief Standard hosted model of the electrical/mechanical plant domain. */
class circuit_simulation
{
  public:
    /** Allow polymorphic destruction by the common simulation system. */
    virtual ~circuit_simulation() = default;
    /** Initialize and settle the electrical/mechanical plant. */
    virtual void initialize_circuit() = 0;
    /** Advance the plant by one numerical simulation step. */
    virtual void step_circuit(const simulation_step_context &context) = 0;
    /** Validate and finalize the plant. */
    virtual void finalize_circuit() = 0;
};

/**
 * @brief Deterministic composition of chip, peripheral, and circuit models.
 *
 * One step executes chip background work, applies peripheral outputs to the
 * plant, advances the circuit, then samples plant outputs. Peripheral sampling
 * may synchronously raise a simulated interrupt in the embedded-chip model.
 */
class simulation_system
{
  public:
    /** Bind one implementation of each simulation domain. */
    simulation_system(embedded_chip_simulation &chip,
                      peripheral_simulation &peripherals,
                      circuit_simulation &circuit) noexcept;
    /** Initialize chip, peripherals, then circuit. */
    void initialize();
    /** Execute one deterministic composed simulation step. */
    void step(std::size_t step_index, double time_s,
              simulation_runtime &runtime);
    /** Finalize circuit, peripherals, then chip. */
    void finalize();

  private:
    embedded_chip_simulation &chip_;
    peripheral_simulation &peripherals_;
    circuit_simulation &circuit_;
};

/**
 * @brief Common command-line policy parsed by gmp_csp_startup().
 */
struct command_line_options
{
    bool suppress_pause{};
    bool request_realtime_priority{};
    bool profile_enabled{};
    bool print_build_info{};
    std::string output_path;
};

/** @brief Project build metadata printed by the generic --build-info path. */
struct build_information
{
    std::string backend;
    std::string storage;
    std::string configuration;
    bool optimized{};
};

/** @return Options already parsed during gmp_csp_startup(). */
const command_line_options &command_line();

/** @brief Project-owned callbacks used by the generic CCTL host runtime. */
struct simulation_callbacks
{
    std::function<void()> initialize;
    std::function<void(std::size_t, double, simulation_runtime &)> step;
    std::function<void(std::size_t, std::size_t, simulation_runtime &)> step_range;
    std::function<void()> finalize;
    std::function<void(const void *, std::ostream &)> write_record;
    std::function<void(std::ostream &)> print_summary;
};

/** @brief Runtime and output policy, normally populated from project SDPE. */
struct simulation_config
{
    std::size_t total_steps{};
    double plant_step_s{};
    std::size_t record_size{};
    std::size_t output_ring_bytes{32U * 1024U * 1024U};
    std::size_t output_batch_bytes{1024U * 1024U};
    std::uint32_t progress_interval_ms{1000U};
    std::size_t step_chunk_size{4096U};
    std::string output_path{"cctl_simulation.csv"};
    std::string output_header;
    std::string console_title{"GMP CCTL Simulation Kit"};
    std::string execution_label;
    std::size_t console_bar_width{64U};
    bool request_realtime_priority{false};
    bool pause_on_exit{true};
};

/** @brief Final statistics collected after simulation and workers finish. */
struct simulation_summary
{
    bool success{};
    std::string message;
    std::size_t completed_steps{};
    std::size_t total_steps{};
    std::size_t queued_records{};
    std::size_t written_records{};
    std::size_t dropped_records{};
    std::size_t peak_queued_records{};
    std::uint64_t output_bytes{};
    double output_worker_busy_time_s{};
    double simulated_time_s{};
    double wall_time_s{};
    double realtime_factor{};
    bool realtime_priority_requested{};
    bool realtime_priority_applied{};
    std::string priority_message;
};

/**
 * Generic hosted CCTL simulation CSP.
 *
 * In the standard GMP path, gmp_csp_loop() advances the simulation on the main
 * thread while asynchronous file output and console progress run as workers.
 * The main thread is the sole producer of the lock-free SPSC record ring.
 */
class simulation_runtime
{
  public:
    /** Construct an uninitialized hosted simulation runtime. */
    simulation_runtime();
    /** Join any owned workers and release runtime storage. */
    ~simulation_runtime();

    simulation_runtime(const simulation_runtime &) = delete;
    simulation_runtime &operator=(const simulation_runtime &) = delete;

    /** Validate and store the configuration and project callbacks. */
    void initialize(simulation_config config, simulation_callbacks callbacks);
    /** Start plant initialization plus file and console workers. */
    void start();
    /** Advance one callback step outside the normal worker-driven run. */
    bool step();
    /** Nonblocking-copy one fixed-size record into the output SPSC ring. */
    bool interface_transfer(const void *record, std::size_t record_size);
    /** Convenience blocking run using the caller plus two service workers. */
    simulation_summary run();
    /** Join workers and materialize the final summary if still running. */
    void finalize();

    /** Mark the current run failed and request a clean framework exit. */
    void fail(const std::string &message) noexcept;

    /** @return Number of numerical steps completed by the simulation thread. */
    std::size_t completed_steps() const noexcept;
    /** @return Records currently queued or staged for the file worker. */
    std::size_t buffered_records() const noexcept;
    /** @return Validated configuration of the current run. */
    const simulation_config &config() const;
    /** @return Latest final or in-progress summary snapshot. */
    const simulation_summary &summary() const;

    /** Print the common CSP completion summary to a stream. */
    void print_summary(std::ostream &stream) const;
    /** Print the optional project-owned completion summary. */
    void print_project_summary(std::ostream &stream) const;
    /** Apply the configured Windows pause unless explicitly suppressed. */
    void pause_if_requested(bool suppress_pause = false) const;

  private:
    class implementation;
    std::unique_ptr<implementation> impl_;
};

/** Register project build metadata from the standard project init() hook. */
void configure_build_information(build_information information);

/**
 * @brief Register one project simulation from the standard project init() hook.
 *
 * gmp_csp_post_process() starts the registered runtime. gmp_csp_loop() advances
 * it exactly one step per GMP background iteration, and gmp_csp_exit() owns
 * finalization and reporting.
 */
void configure_simulation(simulation_config config,
                          simulation_callbacks callbacks);

} // namespace gmp::csp::cctl

#endif // GMP_CSP_CCTL_HOST_SIMULATION_HPP
