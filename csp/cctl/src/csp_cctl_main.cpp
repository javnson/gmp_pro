/**
 * @file csp_cctl_main.cpp
 * @brief Standard GMP lifecycle orchestration for hosted CCTL simulations.
 */

#include <csp.general.h>
#include <csp_cctl.hpp>
#include <gmp_core.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <atomic>
#include <cstdio>
#include <exception>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#if defined(_MSC_VER)
#pragma comment(lib, "Shell32.lib")
#endif
#endif

#ifndef CCTL_SIM_REALTIME_PRIORITY
#define CCTL_SIM_REALTIME_PRIORITY 0
#endif
#ifndef CCTL_SIM_OUTPUT_FILENAME
#define CCTL_SIM_OUTPUT_FILENAME "cctl_simulation.csv"
#endif
#ifndef CCTL_SIM_PAUSE_ON_EXIT
#define CCTL_SIM_PAUSE_ON_EXIT 1
#endif

namespace
{
int process_argc = 0;
char **process_argv = nullptr;
int process_result = EXIT_FAILURE;
bool simulation_configured = false;
bool runtime_started = false;
bool project_configured = false;
bool exit_requested = false;
bool exit_completed = false;
bool build_information_configured = false;
volatile fast_gt output_enabled = 0;
std::string lifecycle_failure;
gmp::csp::cctl::command_line_options parsed_options;
gmp::csp::cctl::build_information registered_build;
gmp::csp::cctl::simulation_runtime runtime;
std::atomic<std::uint64_t> controller_interrupt_count{0U};
float controller_scope[CSP_CCTL_SCOPE_CHANNEL_COUNT]{};
int hosted_console_token = 0;

/** Parse CSP-common process options before project initialization begins. */
std::string parse_command_line(
    int argc, char **argv,
    gmp::csp::cctl::command_line_options &options)
{
    options = {};
    options.delegate_to_viewer = argc == 1;
    options.request_realtime_priority = CCTL_SIM_REALTIME_PRIORITY != 0;
    options.output_path = CCTL_SIM_OUTPUT_FILENAME;
    std::string first_error;

    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--no-pause")
            options.suppress_pause = true;
        else if (argument == "--realtime-priority")
            options.request_realtime_priority = true;
        else if (argument == "--normal-priority" ||
                 argument == "--no-realtime-priority")
            options.request_realtime_priority = false;
        else if (argument == "--profile")
            options.profile_enabled = true;
        else if (argument == "--build-info")
            options.print_build_info = true;
        else if (argument == "--viewer")
            options.launch_viewer = true;
        else if (argument == "--headless")
            options.delegate_to_viewer = false;
        else if (argument == "--continuous")
            options.continuous = true;
        else if (argument == "--supervised")
            options.supervised = true;
        else if (argument == "--wait-for-start")
        {
            options.supervised = true;
            options.wait_for_start = true;
        }
        else if (argument == "--duration" && index + 1 < argc)
        {
            try
            {
                options.target_duration_s = std::stod(argv[++index]);
                options.duration_overridden = true;
                if (!std::isfinite(options.target_duration_s) ||
                    options.target_duration_s < 0.0)
                    throw std::invalid_argument("range");
                options.continuous = options.target_duration_s == 0.0;
            }
            catch (...)
            {
                if (first_error.empty())
                    first_error = "--duration requires a finite nonnegative value";
            }
        }
        else if (argument == "--output" && index + 1 < argc)
            options.output_path = argv[++index];
        else if (first_error.empty())
            first_error = "unknown or incomplete argument: " + argument;
    }
    return first_error;
}

/** Preserve the first lifecycle failure and request normal GMP unwinding. */
void record_lifecycle_failure(const std::string &message) noexcept
{
    if (lifecycle_failure.empty())
        lifecycle_failure = message;
    if (runtime_started)
        runtime.fail(message);
    process_result = EXIT_FAILURE;
    exit_requested = true;
}

/** Print metadata without loading or initializing the simulation plant. */
void print_build_information()
{
    if (!build_information_configured)
        throw std::logic_error(
            "project did not register CCTL build information from "
            "csp_cctl_project_configure()");
    std::cout << "backend=" << registered_build.backend << '/'
              << registered_build.storage << " build="
              << registered_build.configuration << " optimized="
              << (registered_build.optimized ? "yes" : "no") << '\n';
}

/** Start the cross-platform Viewer Manager as the interactive process owner. */
void delegate_process_to_viewer()
{
#if defined(_WIN32)
    const char *root = std::getenv("GMP_PRO_LOCATION");
    if (!root || !*root)
        throw std::runtime_error(
            "interactive CCTL launch requires GMP_PRO_LOCATION");
    const std::filesystem::path launcher =
        std::filesystem::path(root) / "tools" / "cctl_studio" /
        "result_viewer" / "run_result_viewer.bat";
    if (!std::filesystem::exists(launcher))
        throw std::runtime_error("Viewer Manager launcher not found: " +
                                 launcher.string());
    wchar_t executable_buffer[32768]{};
    const DWORD length = GetModuleFileNameW(
        nullptr, executable_buffer,
        static_cast<DWORD>(sizeof(executable_buffer) / sizeof(wchar_t)));
    if (length == 0U || length >= sizeof(executable_buffer) / sizeof(wchar_t))
        throw std::runtime_error("cannot resolve the CCTL executable path");
    const double target_time = static_cast<double>(runtime.target_steps()) *
                               runtime.config().plant_step_s;
    std::wostringstream parameters;
    parameters << L"--simulator \"" << executable_buffer
               << L"\" --duration " << std::setprecision(17) << target_time
               << L" --output \""
               << std::filesystem::absolute(parsed_options.output_path).wstring()
               << L"\" --autostart";
    const HINSTANCE result = ShellExecuteW(
        nullptr, L"open", launcher.c_str(), parameters.str().c_str(),
        launcher.parent_path().c_str(), SW_SHOWNORMAL);
    if (reinterpret_cast<std::intptr_t>(result) <= 32)
        throw std::runtime_error("failed to launch CCTL Viewer Manager");
#else
    throw std::runtime_error(
        "interactive Viewer delegation is not yet installed on this host");
#endif
}
} // namespace

namespace gmp::csp::cctl
{

const command_line_options &command_line()
{
    return parsed_options;
}

void configure_build_information(build_information information)
{
    if (build_information_configured)
        throw std::logic_error("CCTL build information was registered twice");
    registered_build = std::move(information);
    build_information_configured = true;
}

void configure_simulation(simulation_config config,
                          simulation_callbacks callbacks)
{
    if (simulation_configured)
        throw std::logic_error("CCTL simulation was configured twice");
    if (config.outputs.empty())
        config.output_path = parsed_options.output_path;
    if (parsed_options.duration_overridden)
    {
        config.continuous = parsed_options.target_duration_s == 0.0;
        config.total_steps = config.continuous
                                 ? 0U
                                 : static_cast<std::size_t>(
                                       parsed_options.target_duration_s /
                                           config.plant_step_s +
                                       0.5);
    }
    config.launch_viewer = parsed_options.launch_viewer &&
                           !parsed_options.supervised;
    config.continuous = parsed_options.continuous;
    config.supervised = parsed_options.supervised;
    config.wait_for_start = parsed_options.wait_for_start;
    config.request_realtime_priority =
        parsed_options.request_realtime_priority;
    runtime.initialize(std::move(config), std::move(callbacks));
    simulation_configured = true;
}

} // namespace gmp::csp::cctl

extern "C"
{

/** Parse process options before setup_peripheral(), ctl_init(), and init(). */
void gmp_csp_startup(void)
{
    /* Enable GMP diagnostics before setup_peripheral() and the startup Logo. */
    default_debug_dev = static_cast<GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE>(
        &hosted_console_token);

    process_result = EXIT_FAILURE;
    simulation_configured = false;
    runtime_started = false;
    project_configured = false;
    exit_requested = false;
    exit_completed = false;
    build_information_configured = false;
    output_enabled = 0;
    controller_interrupt_count.store(0U, std::memory_order_relaxed);
    for (float &value : controller_scope)
        value = 0.0F;
    lifecycle_failure.clear();
    const std::string parse_error =
        parse_command_line(process_argc, process_argv, parsed_options);
    if (!parse_error.empty())
        record_lifecycle_failure(parse_error);
}

/** Ask the project to register its simulation, then start CSP services. */
void gmp_csp_post_process(void)
{
    if (exit_requested)
        return;
    try
    {
        if (project_configured)
            throw std::logic_error(
                "CCTL project configuration hook was invoked twice");
        csp_cctl_project_configure();
        project_configured = true;

        if (parsed_options.print_build_info)
            return;
        if (!simulation_configured)
            throw std::logic_error(
                "csp_cctl_project_configure() did not configure a CCTL "
                "simulation");
        if (parsed_options.delegate_to_viewer)
        {
            delegate_process_to_viewer();
            process_result = EXIT_SUCCESS;
            exit_requested = true;
            return;
        }
        runtime.start();
        runtime_started = true;
    }
    catch (const std::exception &error)
    {
        record_lifecycle_failure(error.what());
    }
    catch (...)
    {
        record_lifecycle_failure(
            "unknown exception while starting the CCTL runtime");
    }
}

/** Advance exactly one plant/peripheral/control iteration. */
void gmp_csp_loop(void)
{
    try
    {
        if (exit_requested)
            return;
        if (parsed_options.print_build_info)
        {
            print_build_information();
            process_result = EXIT_SUCCESS;
            exit_requested = true;
            return;
        }

        if (!runtime_started)
            throw std::logic_error("CCTL runtime was not started");
        if (!runtime.step())
            exit_requested = true;
    }
    catch (const std::exception &error)
    {
        record_lifecycle_failure(error.what());
    }
    catch (...)
    {
        record_lifecycle_failure("unknown exception in gmp_csp_loop()");
    }
}

/** @return Nonzero once the finite hosted simulation has completed. */
fast_gt gmp_csp_should_exit(void)
{
    return exit_requested ? 1 : 0;
}

/** Finalize workers, print reports, and apply the configured exit pause. */
void gmp_csp_exit(void)
{
    if (exit_completed)
        return;
    exit_completed = true;
    output_enabled = 0;

    if (runtime_started)
    {
        runtime.finalize();
        runtime_started = false;
        if (parsed_options.supervised)
        {
            const auto &summary = runtime.summary();
            const nlohmann::json message = {
                {"type", "summary"}, {"success", summary.success},
                {"message", summary.message},
                {"completed_steps", summary.completed_steps},
                {"target_steps", runtime.target_steps()},
                {"simulated_time_s", summary.simulated_time_s},
                {"wall_time_s", summary.wall_time_s},
                {"dropped", summary.dropped_records}};
            std::cout << message.dump() << std::endl;
        }
        else
        {
            runtime.print_summary(std::cout);
            runtime.print_project_summary(std::cout);
            if (runtime.config().outputs.empty())
                std::cout << "  CSV: " << runtime.config().output_path << '\n';
            else
                for (const auto &output : runtime.config().outputs)
                    std::cout << "  CSV[" << output.name << "]: " << output.path
                              << '\n';
        }
        process_result = runtime.summary().success ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (!lifecycle_failure.empty())
    {
        std::cerr << "[FAIL] " << lifecycle_failure << '\n';
        process_result = EXIT_FAILURE;
    }

    if (simulation_configured && !parsed_options.supervised &&
        !parsed_options.delegate_to_viewer)
        runtime.pause_if_requested(parsed_options.suppress_pause);
#if defined(_WIN32)
    else if (!parsed_options.print_build_info &&
             !parsed_options.delegate_to_viewer &&
             CCTL_SIM_PAUSE_ON_EXIT != 0 &&
             !parsed_options.suppress_pause)
        std::system("@pause");
#endif
}

/** Abort a hosted process after an unrecoverable CSP failure. */
void gmp_csp_stuck_routine(void)
{
    std::abort();
}

/** Abort when a required hosted CSP service is not implemented. */
void gmp_csp_not_implement(void)
{
    std::abort();
}

/** Route the portable GMP diagnostic stream to the hosted console. */
ec_gt gmp_hal_uart_send(GMP_BASE_PRINT_DEFAULT_HANDLE_TYPE,
                        gmp_print_buffer_t *buffer)
{
    if (buffer == nullptr || buffer->buf == nullptr || buffer->length == 0U)
        return GMP_EC_OK;

    FILE *stream = parsed_options.supervised ? stderr : stdout;
    const std::size_t written =
        std::fwrite(buffer->buf, 1U, buffer->length, stream);
    std::fflush(stream);
    return written == buffer->length ? GMP_EC_OK : GMP_EC_GENERAL_ERROR;
}

/** Enable the CSP-owned simulated power-stage output flag. */
void csp_sl_enable_output(void)
{
    output_enabled = 1;
}

/** Disable the CSP-owned simulated power-stage output flag. */
void csp_sl_disable_output(void)
{
    output_enabled = 0;
}

/** Query the CSP-owned simulated power-stage output flag. */
fast_gt csp_cctl_output_is_enabled(void)
{
    return output_enabled;
}

/** @copydoc csp_cctl_notify_controller_interrupt */
void csp_cctl_notify_controller_interrupt(void)
{
    controller_interrupt_count.fetch_add(1U, std::memory_order_relaxed);
}

/** @copydoc csp_cctl_controller_interrupt_count */
uint64_t csp_cctl_controller_interrupt_count(void)
{
    return controller_interrupt_count.load(std::memory_order_relaxed);
}

/** @copydoc csp_cctl_scope_write */
void csp_cctl_scope_write(uint32_t channel, float value)
{
    if (channel < CSP_CCTL_SCOPE_CHANNEL_COUNT)
        controller_scope[channel] = value;
}

/** @copydoc csp_cctl_scope_read */
float csp_cctl_scope_read(uint32_t channel)
{
    return channel < CSP_CCTL_SCOPE_CHANNEL_COUNT
               ? controller_scope[channel]
               : 0.0F;
}

/** CSP-owned hosted time service, expressed in simulated milliseconds. */
time_gt gmp_base_get_system_tick(void)
{
    if (!simulation_configured || runtime.config().plant_step_s <= 0.0)
        return (time_gt)0;
    const double milliseconds =
        static_cast<double>(runtime.completed_steps()) *
        runtime.config().plant_step_s * 1000.0;
    return static_cast<time_gt>(milliseconds);
}

/** Hosted simulations do not require watchdog servicing. */
void gmp_hal_wd_feed(void)
{
}

/** Hosted simulations do not require watchdog activation. */
void gmp_hal_wd_enable(void)
{
}

/** Hosted simulations do not require watchdog deactivation. */
void gmp_hal_wd_disable(void)
{
}

} // extern "C"

/** Capture process arguments and enter the standard GMP lifecycle. */
int main(int argc, char **argv)
{
    process_argc = argc;
    process_argv = argv;
    try
    {
        gmp_base_entry();
    }
    catch (const std::exception &error)
    {
        record_lifecycle_failure(error.what());
        gmp_csp_exit();
    }
    catch (...)
    {
        record_lifecycle_failure("unhandled non-standard CCTL CSP exception");
        gmp_csp_exit();
    }
    return process_result;
}
