#include <csp_cctl.hpp>
#include <cctl/dsa/spsc_record_ring.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace gmp::csp::cctl
{
namespace
{
using clock_type = std::chrono::steady_clock;

bool enable_in_place_console_refresh() noexcept
{
#if defined(_WIN32)
    const HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0U;
    if (output == nullptr || output == INVALID_HANDLE_VALUE ||
        !GetConsoleMode(output, &mode))
        return false;
    return SetConsoleMode(output, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0;
#else
    return ::isatty(STDOUT_FILENO) != 0;
#endif
}
} // namespace

class simulation_runtime::implementation
{
  public:
    void initialize(simulation_config requested_config,
                    simulation_callbacks requested_callbacks)
    {
        if (running_)
            throw std::logic_error("CCTL simulation is already running");
        if (requested_config.total_steps == 0U ||
            !(requested_config.plant_step_s > 0.0) ||
            !std::isfinite(requested_config.plant_step_s) ||
            requested_config.record_size == 0U ||
            requested_config.output_path.empty() ||
            (!requested_callbacks.step && !requested_callbacks.step_range) ||
            !requested_callbacks.write_record)
            throw std::invalid_argument("invalid CCTL simulation configuration");

        config_ = std::move(requested_config);
        callbacks_ = std::move(requested_callbacks);
        ring_.initialize(config_.output_ring_bytes, config_.record_size);
        summary_ = {};
        summary_.total_steps = config_.total_steps;
        completed_steps_.store(0U, std::memory_order_relaxed);
        queued_records_.store(0U, std::memory_order_relaxed);
        written_records_.store(0U, std::memory_order_relaxed);
        dropped_records_.store(0U, std::memory_order_relaxed);
        output_bytes_.store(0U, std::memory_order_relaxed);
        stop_requested_.store(false, std::memory_order_relaxed);
        simulation_done_.store(false, std::memory_order_relaxed);
        failed_.store(false, std::memory_order_relaxed);
        initialized_ = true;
    }

    const simulation_config &config() const;
    const simulation_summary &summary() const;
    std::size_t completed_steps() const noexcept;
    std::size_t buffered_records() const noexcept;

    bool step(simulation_runtime &owner)
    {
        const std::size_t index = completed_steps_.load(std::memory_order_relaxed);
        if (index >= config_.total_steps || stop_requested_.load(std::memory_order_acquire))
            return false;
        callbacks_.step(index, static_cast<double>(index) * config_.plant_step_s,
                        owner);
        completed_steps_.store(index + 1U, std::memory_order_release);
        return index + 1U < config_.total_steps;
    }

    bool step_range(simulation_runtime &owner)
    {
        const std::size_t begin =
            completed_steps_.load(std::memory_order_relaxed);
        if (begin >= config_.total_steps ||
            stop_requested_.load(std::memory_order_acquire))
            return false;
        const std::size_t end = std::min(
            config_.total_steps,
            begin + std::max<std::size_t>(config_.step_chunk_size, 1U));
        callbacks_.step_range(begin, end, owner);
        completed_steps_.store(end, std::memory_order_release);
        return end < config_.total_steps;
    }

    bool interface_transfer(const void *record, std::size_t record_size)
    {
        if (!record || record_size != config_.record_size)
            throw std::invalid_argument("CCTL interface_transfer record size mismatch");
        if (!ring_.try_push(record))
        {
            dropped_records_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        queued_records_.fetch_add(1U, std::memory_order_relaxed);
        return true;
    }

    simulation_summary run(simulation_runtime &owner)
    {
        if (!initialized_)
            throw std::logic_error("initialize() must be called before run()");
        if (running_)
            throw std::logic_error("CCTL simulation is already running");

        running_ = true;
        configure_process_priority();
        start_time_ = clock_type::now();
        try
        {
            file_thread_ = std::thread([this] { file_worker(); });
            console_thread_ = std::thread([this] { console_worker(); });
            simulation_thread_ =
                std::thread([this, &owner] { simulation_worker(owner); });
        }
        catch (const std::exception &error)
        {
            fail(std::string("cannot start CCTL worker threads: ") + error.what());
            simulation_done_.store(true, std::memory_order_release);
            finalize();
            throw;
        }
        finalize();
        return summary_;
    }

    void finalize()
    {
        if (simulation_thread_.joinable())
            simulation_thread_.join();
        if (file_thread_.joinable())
            file_thread_.join();
        if (console_thread_.joinable())
            console_thread_.join();
        if (!running_)
            return;

        restore_process_priority();

        const double wall = std::chrono::duration<double>(clock_type::now() - start_time_).count();
        summary_.completed_steps = completed_steps_.load(std::memory_order_acquire);
        summary_.queued_records = queued_records_.load(std::memory_order_acquire);
        summary_.written_records = written_records_.load(std::memory_order_acquire);
        summary_.dropped_records = dropped_records_.load(std::memory_order_acquire);
        summary_.output_bytes = output_bytes_.load(std::memory_order_acquire);
        summary_.simulated_time_s =
            static_cast<double>(summary_.completed_steps) * config_.plant_step_s;
        summary_.wall_time_s = wall;
        summary_.realtime_factor = wall > 0.0 ? summary_.simulated_time_s / wall : 0.0;
        summary_.success = !failed_.load(std::memory_order_acquire) &&
                           summary_.completed_steps == summary_.total_steps;
        if (summary_.message.empty())
            summary_.message = summary_.success ? "simulation completed" : "simulation stopped";
        running_ = false;
    }

    void simulation_worker(simulation_runtime &owner) noexcept
    {
        try
        {
            if (callbacks_.initialize)
                callbacks_.initialize();
            if (callbacks_.step_range)
                while (step_range(owner))
                {
                }
            else
                while (step(owner))
                {
                }
            if (callbacks_.finalize)
                callbacks_.finalize();
        }
        catch (const std::exception &error)
        {
            fail(error.what());
        }
        catch (...)
        {
            fail("unknown exception in the CCTL simulation worker");
        }
        simulation_done_.store(true, std::memory_order_release);
    }

    void file_worker() noexcept
    {
        try
        {
            std::ofstream output(config_.output_path, std::ios::binary | std::ios::trunc);
            if (!output)
                throw std::runtime_error("cannot create simulation output: " + config_.output_path);
            if (!config_.output_header.empty())
                output << config_.output_header << '\n';

            const std::size_t target_records = std::max<std::size_t>(
                1U, config_.output_batch_bytes / std::max<std::size_t>(config_.record_size, 1U));
            std::vector<std::byte> records(target_records * config_.record_size);
            std::size_t count = 0U;
            while (!simulation_done_.load(std::memory_order_acquire) ||
                   ring_.size() != 0U || count != 0U)
            {
                while (count < target_records &&
                       ring_.try_pop(records.data() + count * config_.record_size))
                    ++count;
                const bool drained =
                    simulation_done_.load(std::memory_order_acquire) && ring_.size() == 0U;
                if (count < target_records && !drained)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                std::ostringstream batch;
                batch << std::setprecision(17);
                for (std::size_t index = 0U; index < count; ++index)
                    callbacks_.write_record(
                        records.data() + index * config_.record_size, batch);
                const std::string payload = batch.str();
                output.write(payload.data(), static_cast<std::streamsize>(payload.size()));
                if (!output)
                    throw std::runtime_error("failed while writing simulation output");
                written_records_.fetch_add(count, std::memory_order_relaxed);
                output_bytes_.fetch_add(static_cast<std::uint64_t>(payload.size()),
                                        std::memory_order_relaxed);
                count = 0U;
            }
            output.flush();
            if (!output)
                throw std::runtime_error("failed while flushing simulation output");
        }
        catch (const std::exception &error)
        {
            fail(error.what());
        }
        catch (...)
        {
            fail("unknown exception in the CCTL output worker");
        }
    }

    void console_worker() noexcept
    {
        const std::chrono::milliseconds interval(
            std::max<std::uint32_t>(config_.progress_interval_ms, 1U));
        interactive_console_ = enable_in_place_console_refresh();
        std::cout << config_.console_title << "\n\n"
                  << std::fixed << std::setprecision(6)
                  << "total_time="
                  << static_cast<double>(config_.total_steps) * config_.plant_step_s
                  << "s  step=" << std::scientific << config_.plant_step_s
                  << "s  total_steps=" << std::fixed << config_.total_steps
                  << (config_.execution_label.empty() ? "" : "  backend=")
                  << config_.execution_label << "\npriority="
                  << summary_.priority_message << "\n\n";
        for (;;)
        {
            const std::size_t completed = completed_steps_.load(std::memory_order_acquire);
            const bool done = simulation_done_.load(std::memory_order_acquire);
            print_progress(completed, done);
            if (done)
                break;
            std::this_thread::sleep_for(interval);
        }
        if (interactive_console_)
            std::cout << '\n';
    }

    void print_progress(std::size_t completed, bool done) const
    {
        const std::size_t width =
            std::max<std::size_t>(config_.console_bar_width, 20U);
        const double ratio = std::min(1.0, static_cast<double>(completed) /
                                              static_cast<double>(config_.total_steps));
        const std::size_t fill = static_cast<std::size_t>(ratio * width);
        const double elapsed = std::chrono::duration<double>(clock_type::now() - start_time_).count();
        const double eta = completed > 0U
                               ? elapsed * static_cast<double>(config_.total_steps - completed) /
                                     static_cast<double>(completed)
                               : 0.0;
        std::ostringstream status;
        status << "elapsed=" << std::fixed << std::setprecision(1) << elapsed
               << "s  ETA=" << (done ? 0.0 : eta)
               << "s  queue=" << ring_.size() << '/' << ring_.capacity()
               << "  dropped=" << dropped_records_.load(std::memory_order_relaxed);
        std::ostringstream progress;
        progress << '[';
        for (std::size_t index = 0U; index < width; ++index)
        {
            if (index < fill)
                progress << '=';
            else if (index == fill && !done)
                progress << '>';
            else
                progress << ' ';
        }
        progress << "] " << std::fixed << std::setprecision(1) << ratio * 100.0
                 << "%  sim=" << std::setprecision(3)
                 << static_cast<double>(completed) * config_.plant_step_s << "s";
        if (!interactive_console_ && !done)
            return;
        if (interactive_console_ && progress_drawn_)
            std::cout << "\x1b[2F";
        if (interactive_console_)
            std::cout << "\r\x1b[2K" << status.str() << '\n'
                      << "\r\x1b[2K" << progress.str() << '\n' << std::flush;
        else
            std::cout << status.str() << '\n' << progress.str() << '\n' << std::flush;
        progress_drawn_ = true;
    }

    void configure_process_priority() noexcept
    {
        summary_.realtime_priority_requested = config_.request_realtime_priority;
        summary_.realtime_priority_applied = false;
        if (!config_.request_realtime_priority)
        {
            summary_.priority_message = "normal (realtime disabled)";
            return;
        }
#if defined(_WIN32)
        const HANDLE process = GetCurrentProcess();
        original_priority_class_ = static_cast<std::uint32_t>(GetPriorityClass(process));
        if (original_priority_class_ == 0U)
        {
            summary_.priority_message =
                "normal (cannot query process priority, error=" +
                std::to_string(GetLastError()) + ')';
            return;
        }
        if (!SetPriorityClass(process, REALTIME_PRIORITY_CLASS))
        {
            summary_.priority_message =
                "normal (realtime request denied, error=" +
                std::to_string(GetLastError()) + ')';
            return;
        }
        summary_.realtime_priority_applied = true;
        summary_.priority_message = "realtime (applied for simulation)";
#else
        summary_.priority_message = "normal (realtime priority unsupported on this host)";
#endif
    }

    void restore_process_priority() noexcept
    {
#if defined(_WIN32)
        if (!summary_.realtime_priority_applied || original_priority_class_ == 0U ||
            original_priority_class_ == REALTIME_PRIORITY_CLASS)
            return;
        if (!SetPriorityClass(GetCurrentProcess(),
                              static_cast<DWORD>(original_priority_class_)))
            summary_.priority_message +=
                "; restore failed, error=" + std::to_string(GetLastError());
#endif
    }

    void fail(const std::string &message) noexcept
    {
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            if (summary_.message.empty())
                summary_.message = message;
        }
        failed_.store(true, std::memory_order_release);
        stop_requested_.store(true, std::memory_order_release);
    }

    simulation_config config_;
    simulation_callbacks callbacks_;
    ::cctl::dsa::spsc_record_ring ring_;
    simulation_summary summary_;
    std::thread simulation_thread_;
    std::thread file_thread_;
    std::thread console_thread_;
    clock_type::time_point start_time_{};
    std::atomic<std::size_t> completed_steps_{0U};
    std::atomic<std::size_t> queued_records_{0U};
    std::atomic<std::size_t> written_records_{0U};
    std::atomic<std::size_t> dropped_records_{0U};
    std::atomic<std::uint64_t> output_bytes_{0U};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> simulation_done_{false};
    std::atomic<bool> failed_{false};
    std::mutex error_mutex_;
    std::uint32_t original_priority_class_{};
    bool interactive_console_{};
    bool progress_drawn_{};
    bool initialized_{};
    bool running_{};
};

simulation_runtime::simulation_runtime() : impl_(std::make_unique<implementation>())
{
}

simulation_runtime::~simulation_runtime()
{
    impl_->finalize();
}

void simulation_runtime::initialize(simulation_config config,
                                    simulation_callbacks callbacks)
{
    impl_->initialize(std::move(config), std::move(callbacks));
}

bool simulation_runtime::step()
{
    return impl_->step(*this);
}

bool simulation_runtime::interface_transfer(const void *record,
                                            std::size_t record_size)
{
    return impl_->interface_transfer(record, record_size);
}

simulation_summary simulation_runtime::run()
{
    return impl_->run(*this);
}

void simulation_runtime::finalize()
{
    impl_->finalize();
}

std::size_t simulation_runtime::completed_steps() const noexcept
{
    return impl_->completed_steps();
}

std::size_t simulation_runtime::buffered_records() const noexcept
{
    return impl_->buffered_records();
}

const simulation_config &simulation_runtime::config() const
{
    return impl_->config();
}

const simulation_summary &simulation_runtime::summary() const
{
    return impl_->summary();
}

void simulation_runtime::print_summary(std::ostream &stream) const
{
    const simulation_summary &value = summary();
    stream << std::setprecision(6)
           << "CCTL CSP summary: " << (value.success ? "PASS" : "FAIL") << " ("
           << value.message << ")\n"
           << "  simulated/wall: " << value.simulated_time_s << " s / "
           << value.wall_time_s << " s, realtime factor=" << value.realtime_factor
           << "\n  priority: " << value.priority_message
           << "\n  steps: " << value.completed_steps << '/' << value.total_steps
           << "\n  output: queued=" << value.queued_records
           << ", written=" << value.written_records
           << ", dropped=" << value.dropped_records
           << ", bytes=" << value.output_bytes << "\n";
}

void simulation_runtime::pause_if_requested(bool suppress_pause) const
{
#if defined(_WIN32)
    if (config().pause_on_exit && !suppress_pause)
        std::system("@pause");
#else
    (void)suppress_pause;
#endif
}

const simulation_config &simulation_runtime::implementation::config() const
{
    return config_;
}

const simulation_summary &simulation_runtime::implementation::summary() const
{
    return summary_;
}

std::size_t simulation_runtime::implementation::buffered_records() const noexcept
{
    return ring_.size();
}

std::size_t simulation_runtime::implementation::completed_steps() const noexcept
{
    return completed_steps_.load(std::memory_order_acquire);
}

} // namespace gmp::csp::cctl
