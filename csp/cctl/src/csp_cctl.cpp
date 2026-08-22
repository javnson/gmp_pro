/**
 * @file csp_cctl.cpp
 * @brief Main-thread simulation runtime with two hosted service workers.
 */

#include <csp_cctl.hpp>
#include <cctl/dsa/spsc_record_ring.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
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
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#include <conio.h>
#if defined(_MSC_VER)
#pragma comment(lib, "Shell32.lib")
#endif
#else
#include <sys/ioctl.h>
#include <sys/select.h>
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

std::size_t console_column_count() noexcept
{
#if defined(_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info{};
    const HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
    if (output == nullptr || output == INVALID_HANDLE_VALUE ||
        !GetConsoleScreenBufferInfo(output, &info))
        return 0U;
    return static_cast<std::size_t>(info.srWindow.Right - info.srWindow.Left + 1);
#else
    winsize size{};
    if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) != 0)
        return 0U;
    return static_cast<std::size_t>(size.ws_col);
#endif
}

bool quit_key_pressed() noexcept
{
#if defined(_WIN32)
    while (_kbhit())
    {
        const int key = _getch();
        if (key == 'q' || key == 'Q')
            return true;
    }
    return false;
#else
    fd_set input;
    FD_ZERO(&input);
    FD_SET(STDIN_FILENO, &input);
    timeval timeout{};
    if (::select(STDIN_FILENO + 1, &input, nullptr, nullptr, &timeout) <= 0)
        return false;
    char key = 0;
    return ::read(STDIN_FILENO, &key, 1) == 1 &&
           (key == 'q' || key == 'Q');
#endif
}
} // namespace

compute_budget_scheduler::compute_budget_scheduler(
    double simulation_step_s, double task_frequency_hz,
    bool dispatch_at_start)
{
    initialize(simulation_step_s, task_frequency_hz, dispatch_at_start);
}

void compute_budget_scheduler::initialize(double simulation_step_s,
                                          double task_frequency_hz,
                                          bool dispatch_at_start)
{
    if (!(simulation_step_s > 0.0) ||
        !(task_frequency_hz > 0.0) ||
        !std::isfinite(simulation_step_s) ||
        !std::isfinite(task_frequency_hz))
        throw std::invalid_argument("invalid MCU compute scheduler frequency");
    executions_per_step_ =
        static_cast<long double>(simulation_step_s) *
        static_cast<long double>(task_frequency_hz);
    phase_ = 0.0L;
    total_executions_ = 0U;
    dispatch_at_start_ = dispatch_at_start;
    first_step_ = true;
}

std::size_t compute_budget_scheduler::consume() noexcept
{
    std::size_t executions = 0U;
    if (first_step_)
    {
        first_step_ = false;
        if (dispatch_at_start_)
            ++executions;
    }
    else
    {
        phase_ += executions_per_step_;
        executions += static_cast<std::size_t>(phase_);
        phase_ -= static_cast<long double>(executions);
    }
    total_executions_ += executions;
    return executions;
}

std::size_t compute_budget_scheduler::total_executions() const noexcept
{
    return total_executions_;
}

simulation_system::simulation_system(embedded_chip_simulation &chip,
                                     peripheral_simulation &peripherals,
                                     circuit_simulation &circuit) noexcept
    : chip_(chip), peripherals_(peripherals), circuit_(circuit)
{
}

void simulation_system::initialize()
{
    chip_.initialize_embedded_chip();
    peripherals_.initialize_peripherals();
    circuit_.initialize_circuit();
}

void simulation_system::step(std::size_t step_index, double time_s,
                             simulation_runtime &runtime)
{
    const simulation_step_context context{step_index, time_s, runtime};
    chip_.step_embedded_chip(context);
    peripherals_.apply_peripheral_outputs(context);
    circuit_.step_circuit(context);
    peripherals_.sample_peripheral_inputs(context);
}

void simulation_system::finalize()
{
    circuit_.finalize_circuit();
    peripherals_.finalize_peripherals();
    chip_.finalize_embedded_chip();
}

class simulation_runtime::implementation
{
  public:
    struct output_state
    {
        simulation_output_config config;
        ::cctl::dsa::spsc_record_ring ring;
        std::atomic<std::size_t> queued{0U};
        std::atomic<std::size_t> written{0U};
        std::atomic<std::size_t> dropped{0U};
        std::atomic<std::size_t> peak{0U};
        std::atomic<std::size_t> staged{0U};
        std::atomic<std::uint64_t> bytes{0U};
    };

    void initialize(simulation_config requested_config,
                    simulation_callbacks requested_callbacks)
    {
        if (running_)
            throw std::logic_error("CCTL simulation is already running");
        if ((!requested_config.continuous && requested_config.total_steps == 0U) ||
            !(requested_config.plant_step_s > 0.0) ||
            !std::isfinite(requested_config.plant_step_s) ||
            (!requested_callbacks.step && !requested_callbacks.step_range))
            throw std::invalid_argument("invalid CCTL simulation configuration");

        if (requested_config.outputs.empty())
        {
            if (requested_config.record_size == 0U ||
                requested_config.output_path.empty() ||
                !requested_callbacks.write_record)
                throw std::invalid_argument("invalid legacy CCTL output configuration");
            simulation_output_config legacy;
            legacy.name = "simulation";
            legacy.record_size = requested_config.record_size;
            legacy.ring_bytes = requested_config.output_ring_bytes;
            legacy.batch_bytes = requested_config.output_batch_bytes;
            legacy.path = requested_config.output_path;
            legacy.header = requested_config.output_header;
            legacy.write_record = requested_callbacks.write_record;
            requested_config.outputs.push_back(std::move(legacy));
        }
        for (const auto &output : requested_config.outputs)
            if (output.record_size == 0U || output.path.empty() ||
                !output.write_record)
                throw std::invalid_argument("invalid CCTL output stream configuration");

        config_ = std::move(requested_config);
        callbacks_ = std::move(requested_callbacks);
        outputs_.clear();
        for (const auto &output_config : config_.outputs)
        {
            auto output = std::make_unique<output_state>();
            output->config = output_config;
            output->ring.initialize(output_config.ring_bytes,
                                    output_config.record_size);
            outputs_.push_back(std::move(output));
        }
        summary_ = {};
        summary_.total_steps = config_.total_steps;
        summary_.continuous = config_.continuous;
        completed_steps_.store(0U, std::memory_order_relaxed);
        output_worker_busy_ns_ = 0U;
        stop_requested_.store(false, std::memory_order_relaxed);
        simulation_done_.store(false, std::memory_order_relaxed);
        failed_.store(false, std::memory_order_relaxed);
        user_stop_requested_.store(false, std::memory_order_relaxed);
        original_priority_class_ = 0U;
        last_progress_time_ = {};
        last_progress_steps_ = 0U;
        interactive_console_ = false;
        progress_anchor_saved_ = false;
        progress_rate_initialized_ = false;
        callback_initialized_ = false;
        callback_finalized_ = false;
        initialized_ = true;
    }

    const simulation_config &config() const;
    const simulation_summary &summary() const;
    std::size_t completed_steps() const noexcept;
    std::size_t buffered_records() const noexcept;

    void start()
    {
        if (!initialized_)
            throw std::logic_error("initialize() must be called before start()");
        if (running_)
            throw std::logic_error("CCTL simulation is already running");

        running_ = true;
        start_time_ = clock_type::now();
        try
        {
            if (callbacks_.initialize)
                callbacks_.initialize();
            callback_initialized_ = true;
            file_thread_ = std::thread([this] { file_worker(); });
            if (config_.launch_viewer)
                launch_result_viewer();
            configure_process_priority();
            console_thread_ = std::thread([this] { console_worker(); });
        }
        catch (const std::exception &error)
        {
            fail(std::string("cannot start CCTL runtime: ") + error.what());
            simulation_done_.store(true, std::memory_order_release);
            finalize();
            throw;
        }
    }

    bool step(simulation_runtime &owner)
    {
        if (!running_)
            throw std::logic_error("start() must be called before step()");
        const std::size_t index = completed_steps_.load(std::memory_order_relaxed);
        if ((!config_.continuous && index >= config_.total_steps) ||
            stop_requested_.load(std::memory_order_acquire))
            return false;
        if (callbacks_.step)
            callbacks_.step(index,
                            static_cast<double>(index) * config_.plant_step_s,
                            owner);
        else
            callbacks_.step_range(index, index + 1U, owner);
        completed_steps_.store(index + 1U, std::memory_order_release);
        return config_.continuous
                   ? !stop_requested_.load(std::memory_order_acquire)
                   : index + 1U < config_.total_steps;
    }

    bool step_range(simulation_runtime &owner)
    {
        const std::size_t begin =
            completed_steps_.load(std::memory_order_relaxed);
        if ((!config_.continuous && begin >= config_.total_steps) ||
            stop_requested_.load(std::memory_order_acquire))
            return false;
        const std::size_t chunk =
            std::max<std::size_t>(config_.step_chunk_size, 1U);
        const std::size_t end = config_.continuous
                                    ? begin + chunk
                                    : std::min(config_.total_steps, begin + chunk);
        callbacks_.step_range(begin, end, owner);
        completed_steps_.store(end, std::memory_order_release);
        return config_.continuous
                   ? !stop_requested_.load(std::memory_order_acquire)
                   : end < config_.total_steps;
    }

    bool interface_transfer(const void *record, std::size_t record_size)
    {
        return interface_transfer(0U, record, record_size);
    }

    bool interface_transfer(std::size_t stream_index, const void *record,
                            std::size_t record_size)
    {
        if (stream_index >= outputs_.size())
            throw std::out_of_range("CCTL output stream index out of range");
        output_state &output = *outputs_[stream_index];
        if (!record || record_size != output.config.record_size)
            throw std::invalid_argument("CCTL interface_transfer record size mismatch");
        if (!output.ring.try_push(record))
        {
            output.dropped.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        output.queued.fetch_add(1U, std::memory_order_relaxed);
        const std::size_t queued = std::max<std::size_t>(output.ring.size(), 1U);
        std::size_t peak = output.peak.load(std::memory_order_relaxed);
        while (queued > peak &&
               !output.peak.compare_exchange_weak(
                   peak, queued, std::memory_order_relaxed,
                   std::memory_order_relaxed))
        {
        }
        return true;
    }

    simulation_summary run(simulation_runtime &owner)
    {
        start();
        try
        {
            if (callbacks_.step_range)
                while (step_range(owner))
                {
                }
            else
                while (step(owner))
                {
                }
        }
        catch (const std::exception &error)
        {
            fail(error.what());
        }
        catch (...)
        {
            fail("unknown exception in the CCTL simulation loop");
        }
        finalize();
        return summary_;
    }

    void finalize()
    {
        if (!running_)
            return;

        if (callback_initialized_ && !callback_finalized_ && callbacks_.finalize)
        {
            try
            {
                callbacks_.finalize();
            }
            catch (const std::exception &error)
            {
                fail(error.what());
            }
            catch (...)
            {
                fail("unknown exception while finalizing the CCTL plant");
            }
            callback_finalized_ = true;
        }

        simulation_done_.store(true, std::memory_order_release);
        if (file_thread_.joinable())
            file_thread_.join();
        if (console_thread_.joinable())
            console_thread_.join();

        restore_process_priority();

        const double wall = std::chrono::duration<double>(clock_type::now() - start_time_).count();
        summary_.completed_steps = completed_steps_.load(std::memory_order_acquire);
        summary_.outputs.clear();
        summary_.queued_records = summary_.written_records =
            summary_.dropped_records = summary_.peak_queued_records = 0U;
        summary_.output_bytes = 0U;
        for (const auto &output : outputs_)
        {
            simulation_output_summary item;
            item.name = output->config.name;
            item.path = output->config.path;
            item.queued_records = output->queued.load(std::memory_order_acquire);
            item.written_records = output->written.load(std::memory_order_acquire);
            item.dropped_records = output->dropped.load(std::memory_order_acquire);
            item.peak_queued_records = output->peak.load(std::memory_order_acquire);
            item.output_bytes = output->bytes.load(std::memory_order_acquire);
            summary_.queued_records += item.queued_records;
            summary_.written_records += item.written_records;
            summary_.dropped_records += item.dropped_records;
            summary_.peak_queued_records += item.peak_queued_records;
            summary_.output_bytes += item.output_bytes;
            summary_.outputs.push_back(std::move(item));
        }
        summary_.output_worker_busy_time_s =
            static_cast<double>(output_worker_busy_ns_) * 1.0e-9;
        summary_.simulated_time_s =
            static_cast<double>(summary_.completed_steps) * config_.plant_step_s;
        summary_.wall_time_s = wall;
        summary_.realtime_factor = wall > 0.0 ? summary_.simulated_time_s / wall : 0.0;
        summary_.stopped_by_user =
            user_stop_requested_.load(std::memory_order_acquire);
        summary_.success = !failed_.load(std::memory_order_acquire) &&
                           (config_.continuous
                                ? summary_.stopped_by_user
                                : summary_.completed_steps == summary_.total_steps);
        if (summary_.message.empty())
            summary_.message = summary_.stopped_by_user
                                   ? "continuous simulation stopped by user"
                                   : (summary_.success ? "simulation completed"
                                                       : "simulation stopped");
        running_ = false;
    }

    void file_worker() noexcept
    {
        try
        {
            struct writer_state
            {
                std::ofstream stream;
                std::vector<std::byte> records;
                std::size_t target_records{};
                std::size_t count{};
            };
            std::vector<std::unique_ptr<writer_state>> writers;
            for (const auto &output : outputs_)
            {
                auto writer = std::make_unique<writer_state>();
                writer->stream.open(output->config.path,
                                    std::ios::binary | std::ios::trunc);
                if (!writer->stream)
                    throw std::runtime_error("cannot create simulation output: " +
                                             output->config.path);
                if (!output->config.header.empty())
                    writer->stream << output->config.header << '\n';
                writer->stream.flush();
                writer->target_records = std::max<std::size_t>(
                    1U, output->config.batch_bytes / output->config.record_size);
                writer->records.resize(writer->target_records *
                                       output->config.record_size);
                writers.push_back(std::move(writer));
            }

            for (;;)
            {
                bool pending = false;
                bool wrote = false;
                const bool done = simulation_done_.load(std::memory_order_acquire);
                for (std::size_t stream_index = 0U;
                     stream_index < outputs_.size(); ++stream_index)
                {
                    output_state &output = *outputs_[stream_index];
                    writer_state &writer = *writers[stream_index];
                    while (writer.count < writer.target_records &&
                           output.ring.try_pop(
                               writer.records.data() + writer.count *
                                   output.config.record_size))
                        ++writer.count;
                    output.staged.store(writer.count, std::memory_order_relaxed);
                    pending = pending || output.ring.size() != 0U ||
                              writer.count != 0U;
                    if (writer.count == 0U ||
                        (writer.count < writer.target_records && !done))
                        continue;

                    const clock_type::time_point busy_begin = clock_type::now();
                    std::ostringstream batch;
                    batch << std::setprecision(17);
                    for (std::size_t index = 0U; index < writer.count; ++index)
                        output.config.write_record(
                            writer.records.data() + index *
                                output.config.record_size,
                            batch);
                    const std::string payload = batch.str();
                    writer.stream.write(payload.data(),
                                        static_cast<std::streamsize>(payload.size()));
                    if (!writer.stream)
                        throw std::runtime_error("failed while writing simulation output: " +
                                                 output.config.path);
                    output.written.fetch_add(writer.count,
                                             std::memory_order_relaxed);
                    output.bytes.fetch_add(payload.size(),
                                           std::memory_order_relaxed);
                    output_worker_busy_ns_ += static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            clock_type::now() - busy_begin).count());
                    writer.count = 0U;
                    output.staged.store(0U, std::memory_order_relaxed);
                    wrote = true;
                }
                if (done && !pending)
                    break;
                if (!wrote)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            const clock_type::time_point flush_begin = clock_type::now();
            for (auto &writer : writers)
                writer->stream.flush();
            output_worker_busy_ns_ += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    clock_type::now() - flush_begin)
                    .count());
            for (std::size_t index = 0U; index < writers.size(); ++index)
                if (!writers[index]->stream)
                    throw std::runtime_error("failed while flushing simulation output: " +
                                             outputs_[index]->config.path);
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
        std::cout << config_.console_title << "\n\n" << std::fixed
                  << std::setprecision(6);
        if (config_.continuous)
            std::cout << "mode=continuous (press q to stop)  step="
                      << std::scientific << config_.plant_step_s << "s";
        else
            std::cout << "total_time="
                      << static_cast<double>(config_.total_steps) *
                             config_.plant_step_s
                      << "s  step=" << std::scientific << config_.plant_step_s
                      << "s  total_steps=" << std::fixed << config_.total_steps;
        std::cout
                  << (config_.execution_label.empty() ? "" : "  backend=")
                  << config_.execution_label << "\npriority="
                  << summary_.priority_message << "\n\n";
        clock_type::time_point next_progress = clock_type::now();
        for (;;)
        {
            if (config_.continuous && quit_key_pressed())
            {
                user_stop_requested_.store(true, std::memory_order_release);
                stop_requested_.store(true, std::memory_order_release);
            }
            const std::size_t completed = completed_steps_.load(std::memory_order_acquire);
            const bool done = simulation_done_.load(std::memory_order_acquire);
            const clock_type::time_point now = clock_type::now();
            if (done || now >= next_progress)
            {
                print_progress(completed, done);
                next_progress = now + interval;
            }
            if (done)
                break;
            std::this_thread::sleep_for(config_.continuous
                                            ? std::chrono::milliseconds(25)
                                            : interval);
        }
        if (interactive_console_)
            std::cout << '\n';
    }

    void print_progress(std::size_t completed, bool done)
    {
        const double ratio = config_.continuous
                                 ? 0.0
                                 : std::min(1.0, static_cast<double>(completed) /
                                                     static_cast<double>(config_.total_steps));
        const clock_type::time_point now = clock_type::now();
        const double elapsed =
            std::chrono::duration<double>(now - start_time_).count();
        double step_rate = 0.0;
        if (progress_rate_initialized_)
        {
            const double interval_s =
                std::chrono::duration<double>(now - last_progress_time_).count();
            if (interval_s > 0.0 && completed >= last_progress_steps_)
                step_rate = static_cast<double>(completed - last_progress_steps_) /
                            interval_s;
        }
        if (done && elapsed > 0.0)
            step_rate = static_cast<double>(completed) / elapsed;
        last_progress_time_ = now;
        last_progress_steps_ = completed;
        progress_rate_initialized_ = true;

        std::ostringstream progress_suffix;
        if (config_.continuous)
            progress_suffix << "] press q to stop";
        else
            progress_suffix << "] " << std::fixed << std::setprecision(1)
                            << ratio * 100.0 << '%';
        std::size_t width = std::max<std::size_t>(config_.console_bar_width, 20U);
        const std::size_t columns = interactive_console_ ? console_column_count() : 0U;
        const std::size_t fixed_characters = progress_suffix.str().size() + 2U;
        if (columns > fixed_characters + 20U)
            width = std::min<std::size_t>(columns - fixed_characters, 512U);
        const std::size_t fill = config_.continuous
                                     ? width
                                     : static_cast<std::size_t>(ratio * width);
        const double eta = !config_.continuous && completed > 0U
                               ? elapsed * static_cast<double>(config_.total_steps - completed) /
                                     static_cast<double>(completed)
                               : 0.0;
        std::ostringstream status;
        std::size_t queued = 0U, capacity = 0U, staged = 0U, dropped = 0U;
        for (const auto &output : outputs_)
        {
            queued += output->ring.size();
            capacity += output->ring.capacity();
            staged += output->staged.load(std::memory_order_relaxed);
            dropped += output->dropped.load(std::memory_order_relaxed);
        }
        status << "elapsed=" << std::fixed << std::setprecision(1) << elapsed;
        if (!config_.continuous)
            status << "s ETA=" << (done ? 0.0 : eta);
        status << "s sim=" << std::setprecision(3)
               << static_cast<double>(completed) * config_.plant_step_s
               << "s rate=" << std::setprecision(2) << step_rate / 1.0e6
               << "Mstep/s"
               << " queue=" << queued << '/' << capacity
               << " staged=" << staged << " drop=" << dropped;
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
        progress << progress_suffix.str();
        if (!interactive_console_ && !done)
            return;
        if (interactive_console_)
        {
            if (!progress_anchor_saved_)
            {
                std::cout << "\x1b[s";
                progress_anchor_saved_ = true;
            }
            else
                std::cout << "\x1b[u";
            std::cout << "\x1b[J" << status.str() << '\n'
                      << progress.str() << '\n' << std::flush;
        }
        else
            std::cout << status.str() << '\n' << progress.str() << '\n' << std::flush;
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

    void launch_result_viewer()
    {
#if defined(_WIN32)
        const char *root = std::getenv("GMP_PRO_LOCATION");
        if (!root || !*root)
            throw std::runtime_error(
                "--viewer requires the GMP_PRO_LOCATION environment variable");
        const std::filesystem::path launcher =
            std::filesystem::path(root) / "tools" / "cctl_studio" /
            "result_viewer" / "run_result_viewer.bat";
        if (!std::filesystem::exists(launcher))
            throw std::runtime_error("result viewer launcher not found: " +
                                     launcher.string());
        for (unsigned retry = 0U; retry < 200U; ++retry)
        {
            bool ready = true;
            for (const auto &output : outputs_)
                ready = ready && std::filesystem::exists(output->config.path) &&
                        std::filesystem::file_size(output->config.path) != 0U;
            if (ready)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::wstring parameters = L"--live";
        if (config_.continuous)
            parameters += L" --rolling-window 0.1";
        for (const auto &output : outputs_)
        {
            parameters += L" \"";
            parameters += std::filesystem::absolute(output->config.path).wstring();
            parameters += L"\"";
        }
        const HINSTANCE result = ShellExecuteW(
            nullptr, L"open", launcher.c_str(), parameters.c_str(),
            launcher.parent_path().c_str(), SW_SHOWNORMAL);
        if (reinterpret_cast<std::intptr_t>(result) <= 32)
            throw std::runtime_error("failed to launch GMP result viewer");
#else
        throw std::runtime_error("--viewer is supported only on Windows");
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
    std::vector<std::unique_ptr<output_state>> outputs_;
    simulation_summary summary_;
    std::thread file_thread_;
    std::thread console_thread_;
    clock_type::time_point start_time_{};
    std::atomic<std::size_t> completed_steps_{0U};
    std::uint64_t output_worker_busy_ns_{};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> simulation_done_{false};
    std::atomic<bool> failed_{false};
    std::atomic<bool> user_stop_requested_{false};
    std::mutex error_mutex_;
    std::uint32_t original_priority_class_{};
    clock_type::time_point last_progress_time_{};
    std::size_t last_progress_steps_{};
    bool interactive_console_{};
    bool progress_anchor_saved_{};
    bool progress_rate_initialized_{};
    bool callback_initialized_{};
    bool callback_finalized_{};
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

void simulation_runtime::start()
{
    impl_->start();
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

bool simulation_runtime::interface_transfer(std::size_t stream_index,
                                            const void *record,
                                            std::size_t record_size)
{
    return impl_->interface_transfer(stream_index, record, record_size);
}

simulation_summary simulation_runtime::run()
{
    return impl_->run(*this);
}

void simulation_runtime::finalize()
{
    impl_->finalize();
}

void simulation_runtime::fail(const std::string &message) noexcept
{
    impl_->fail(message);
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
           << "\n  steps: " << value.completed_steps;
    if (value.continuous)
        stream << " (continuous, stopped by user="
               << (value.stopped_by_user ? "yes" : "no") << ')';
    else
        stream << '/' << value.total_steps;
    stream
           << "\n  output: queued=" << value.queued_records
           << ", written=" << value.written_records
           << ", dropped=" << value.dropped_records
           << ", peak_ring=" << value.peak_queued_records
           << ", bytes=" << value.output_bytes
           << ", writer_busy=" << value.output_worker_busy_time_s
           << " s (asynchronous)\n";
    for (const auto &output : value.outputs)
        stream << "    [" << output.name << "] written="
               << output.written_records << ", dropped="
               << output.dropped_records << ", bytes=" << output.output_bytes
               << ", path=" << output.path << '\n';
}

void simulation_runtime::print_project_summary(std::ostream &stream) const
{
    if (impl_->callbacks_.print_summary)
        impl_->callbacks_.print_summary(stream);
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
    std::size_t total = 0U;
    for (const auto &output : outputs_)
        total += output->ring.size() +
                 output->staged.load(std::memory_order_relaxed);
    return total;
}

std::size_t simulation_runtime::implementation::completed_steps() const noexcept
{
    return completed_steps_.load(std::memory_order_acquire);
}

} // namespace gmp::csp::cctl
