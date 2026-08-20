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

/** Project-owned callbacks used by the generic CCTL host runtime. */
struct simulation_callbacks
{
    std::function<void()> initialize;
    std::function<void(std::size_t, double, simulation_runtime &)> step;
    std::function<void(std::size_t, std::size_t, simulation_runtime &)> step_range;
    std::function<void()> finalize;
    std::function<void(const void *, std::ostream &)> write_record;
};

/** Runtime and output policy. Values normally come from a project SDPE header. */
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

/** Final statistics collected after the three worker threads have joined. */
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
 * run() owns three workers: simulation, asynchronous file output, and console
 * progress. The simulation worker is the sole producer of the lock-free SPSC
 * record ring; interface_transfer() is its non-blocking transfer boundary.
 */
class simulation_runtime
{
  public:
    simulation_runtime();
    ~simulation_runtime();

    simulation_runtime(const simulation_runtime &) = delete;
    simulation_runtime &operator=(const simulation_runtime &) = delete;

    void initialize(simulation_config config, simulation_callbacks callbacks);
    bool step();
    bool interface_transfer(const void *record, std::size_t record_size);
    simulation_summary run();
    void finalize();

    std::size_t completed_steps() const noexcept;
    std::size_t buffered_records() const noexcept;
    const simulation_config &config() const;
    const simulation_summary &summary() const;

    void print_summary(std::ostream &stream) const;
    void pause_if_requested(bool suppress_pause = false) const;

  private:
    class implementation;
    std::unique_ptr<implementation> impl_;
};

} // namespace gmp::csp::cctl

#endif // GMP_CSP_CCTL_HOST_SIMULATION_HPP
