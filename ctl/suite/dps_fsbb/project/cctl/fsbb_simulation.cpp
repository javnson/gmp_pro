#include <gmp_core.hpp>
#include <cctl/component/control_peripheral/peripheral_if.hpp>
#include <csp/cctl/common/cctl_simulation.hpp>
#include <xplt.peripheral.h>

#include "fsbb_closed_loop.hpp"
#include "user_main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <ostream>
#include <stdexcept>

namespace
{
namespace host = gmp::csp::cctl;
using Circuit = FsbbClosedLoopCircuit;
using Pwm = cctl::ti_epwm<sim_real_gt>;
using Adc = cctl::ti_adc<sim_real_gt, FSBB_ADC_COUNT>;

constexpr double plant_step = CCTL_SIM_PLANT_STEP_S;
constexpr double simulation_duration = BUILD_LEVEL == 3 ? CCTL_SIM_LEVEL3_DURATION_S : CCTL_SIM_DURATION_S;
constexpr double voltage_tolerance = 0.02 * FSBB_DEFAULT_OUTPUT_VOLTAGE;
constexpr double pwm_clock = CCTL_SIM_EPWM_TBCLK_HZ;
constexpr auto clocks_per_step = static_cast<std::uint64_t>(pwm_clock * plant_step + 0.5);
constexpr auto pwm_period_clocks = 2U * (CTRL_PWM_CMP_MAX + 1U);
constexpr auto steps_per_carrier = pwm_period_clocks / clocks_per_step;
static_assert(Circuit::normal_step_s == plant_step, "FSBB plant step differs from the generated model");
static_assert(pwm_period_clocks % clocks_per_step == 0U, "PWM carrier must align with plant steps");
static_assert(pwm_clock / pwm_period_clocks == CONTROLLER_FREQUENCY, "FSBB PWM and control rates differ");

constexpr const char* circuit_csv_header = "time_s,source_v,vin_v,vout_v,il_a,iout_a,pwm1,pwm2,pwm3,pwm4";
constexpr const char* control_csv_header =
    "time_s,enabled,cia402_state,faults,buck_high_cmp,boost_low_cmp,adc_vin,adc_vout,adc_il,adc_iout,"
    "vin_v,vout_v,il_a,iout_a,v_command_v,i_command_a";

struct CircuitRecord
{
    double time, source, vin, vout, il, iout;
    std::uint32_t pwm1, pwm2, pwm3, pwm4;
};

struct ControlRecord
{
    double time;
    unsigned enabled, state, faults;
    pwm_gt buck, boost;
    std::array<adc_gt, FSBB_ADC_COUNT> adc;
    std::array<double, FSBB_ADC_COUNT> physical;
    double voltage_command, current_command;
};

void write_circuit_record(const void* data, std::ostream& out)
{
    const auto& r = *static_cast<const CircuitRecord*>(data);
    out << r.time << ',' << r.source << ',' << r.vin << ',' << r.vout << ','
        << r.il << ',' << r.iout << ',' << r.pwm1 << ',' << r.pwm2 << ','
        << r.pwm3 << ',' << r.pwm4 << '\n';
}

void write_control_record(const void* data, std::ostream& out)
{
    const auto& r = *static_cast<const ControlRecord*>(data);
    out << r.time << ',' << r.enabled << ',' << r.state << ',' << r.faults << ','
        << r.buck << ',' << r.boost;
    for (auto code : r.adc)
        out << ',' << code;
    for (double value : r.physical)
        out << ',' << value;
    out << ',' << r.voltage_command << ',' << r.current_command << '\n';
}

struct SteadyWindow
{
    double vin{}, vout{}, il{}, iout{}, buck{}, boost{};
    std::size_t count{};

    void add(const Circuit::Outputs& sample)
    {
        vin += sample.VIN;
        vout += sample.VF1;
        il += sample.VAM1;
        iout += sample.IOUT;
        buck += double(fsbb_buck_compare) / CTRL_PWM_CMP_MAX;
        boost += double(fsbb_boost_compare) / CTRL_PWM_CMP_MAX;
        ++count;
    }

    double mean(double sum) const { return count == 0U ? 0.0 : sum / static_cast<double>(count); }
};

class FsbbSimulation final : public host::embedded_chip_simulation,
                             public host::peripheral_simulation,
                             public host::circuit_simulation
{
public:
    explicit FsbbSimulation(bool validate) : validate_(validate) {}

    void initialize_embedded_chip() override
    {
        background_.initialize(plant_step, CCTL_SIM_USER_CODE_FREQUENCY_HZ);
    }

    void initialize_peripherals() override
    {
        adc_.reset();
        buck_.reset();
        boost_.reset();
        adc_.set_interrupt_handler([](void*) { fsbb_adc_interrupt(); });
    }

    void initialize_circuit() override
    {
        circuit_.reset();
        recording_.initialize(plant_step, CCTL_SIM_CIRCUIT_RECORD_FREQUENCY_HZ);
    }

    void step_embedded_chip(const host::simulation_step_context&) override
    {
        const auto calls = background_.consume();
        // The FSBB user scheduler already dispatches ctl_mainloop() every millisecond.
        for (std::size_t i = 0; i < calls; ++i)
            mainloop();
    }

    void apply_peripheral_outputs(const host::simulation_step_context& context) override
    {
        const auto clocks = context.step_index * clocks_per_step;
        const auto buck = buck_.sample_time_base_count(clocks);
        const auto boost = boost_.sample_time_base_count(clocks);
        adc_trigger_ = buck.adc_trigger;
        if ((buck.upper && buck.lower) || (boost.upper && boost.lower))
            throw std::runtime_error("FSBB complementary gate overlap");

        const double source = BUILD_LEVEL == 3
            ? (context.time_s < CCTL_SIM_VIN_STEP_S ? CCTL_SIM_VIN_BEFORE_STEP : CCTL_SIM_VIN_AFTER_STEP)
            : FSBB_INPUT_VOLTAGE_NOMINAL;
        // The TINA netlist names its boost upper gate PWM4 and lower gate PWM3.
        inputs_ = {buck.upper, buck.lower, boost.lower, boost.upper, source};
    }

    void step_circuit(const host::simulation_step_context& context) override
    {
        const auto& sample = circuit_.step_normal(inputs_);
        ++plant_steps_;
        if (!std::isfinite(sample.VF1) || !std::isfinite(sample.VAM1) ||
            !std::isfinite(sample.VIN) || !std::isfinite(sample.IOUT))
            throw std::runtime_error("non-finite FSBB circuit output");
        peak_vout_ = std::max(peak_vout_, sample.VF1);
        peak_il_ = std::max(peak_il_, std::abs(sample.VAM1));
        if constexpr (BUILD_LEVEL == 3)
        {
            if (context.time_s >= CCTL_SIM_VIN_STEP_S)
            {
                if (context.time_s < CCTL_SIM_VIN_STEP_S + 0.15)
                {
                    if (sample.VF1 < step_min_vout_)
                    {
                        step_min_vout_ = sample.VF1;
                        step_min_time_ = context.time_s;
                    }
                    step_max_vout_ = std::max(step_max_vout_, sample.VF1);
                }
                if (std::abs(sample.VF1 - FSBB_DEFAULT_OUTPUT_VOLTAGE) > voltage_tolerance)
                    step_last_outside_time_ = context.time_s;
            }
        }
        if (recording_.consume())
        {
            const CircuitRecord record{context.time_s, inputs_.VS1, sample.VIN, sample.VF1,
                sample.VAM1, sample.IOUT, inputs_.PWM1, inputs_.PWM2, inputs_.PWM3, inputs_.PWM4};
            context.runtime.interface_transfer(0U, &record, sizeof(record));
        }
    }

    void sample_peripheral_inputs(const host::simulation_step_context& context) override
    {
        if (!adc_trigger_)
            return;
        const auto& sample = circuit_.output;
        adc_.set_input_voltages({
            sample.VIN * CTRL_FSBB_VIN_SENSITIVITY + CTRL_FSBB_VIN_BIAS,
            sample.VF1 * CTRL_FSBB_VOUT_SENSITIVITY + CTRL_FSBB_VOUT_BIAS,
            sample.VAM1 * CTRL_FSBB_IL_SENSITIVITY + CTRL_FSBB_IL_BIAS,
            sample.IOUT * CTRL_FSBB_IOUT_SENSITIVITY + CTRL_FSBB_IOUT_BIAS,
        });
        adc_.trigger_and_transfer(fsbb_adc_result, FSBB_ADC_COUNT);
        adc_.acknowledge_interrupt();

        const bool enabled = csp_cctl_output_is_enabled() != 0;
        buck_.apply_control(fsbb_buck_compare, enabled);
        // CMP from GMP represents the boost *lower* duty. The ePWM model takes upper duty.
        boost_.apply_control(CTRL_PWM_CMP_MAX - fsbb_boost_compare, enabled);
        ever_enabled_ = ever_enabled_ || enabled;
        const ControlRecord record{context.time_s, unsigned(enabled), unsigned(cia402_sm.current_state),
            g_fsbb_faults, fsbb_buck_compare, fsbb_boost_compare,
            {fsbb_adc_result[0], fsbb_adc_result[1], fsbb_adc_result[2], fsbb_adc_result[3]},
            {sample.VIN, sample.VF1, sample.VAM1, sample.IOUT},
            ctrl2param(dcdc_core.v_out_formal) * CTRL_VOLTAGE_BASE,
            ctrl2param(dcdc_core.i_target) * CTRL_CURRENT_BASE};
        context.runtime.interface_transfer(1U, &record, sizeof(record));
        if (g_fsbb_faults != FSBB_FAULT_NONE)
            throw std::runtime_error("FSBB controller latched fault " + std::to_string(g_fsbb_faults));

        if (context.time_s >= CCTL_SIM_VIN_STEP_S - 0.05 && context.time_s < CCTL_SIM_VIN_STEP_S)
            before_step_.add(sample);
        if (context.time_s >= simulation_duration - 0.05 && context.time_s < simulation_duration)
            tail_.add(sample);
    }

    void finalize_circuit() override
    {
        const auto expected = (plant_steps_ + steps_per_carrier - 1U) / steps_per_carrier;
        if (adc_.trigger_count() != expected || csp_cctl_controller_interrupt_count() != expected ||
            adc_.interrupt_pending())
            throw std::runtime_error("FSBB PWM, ADC and control ISR counts differ");
        if (!validate_)
            return;
        if (!ever_enabled_ || tail_.count == 0U || cia402_sm.current_state != CIA402_SM_OPERATION_ENABLED)
            throw std::runtime_error("FSBB did not reach operation enabled");

        if constexpr (BUILD_LEVEL == 1)
        {
            const double reference = FSBB_OPEN_LOOP_VOLTAGE_COMMAND;
            if (std::abs(tail_.mean(tail_.vout) - reference) > 0.15 * reference)
                throw std::runtime_error("FSBB open-loop output is outside its commissioning window");
        }
        if constexpr (BUILD_LEVEL == 2)
        {
            if (std::abs(tail_.mean(tail_.il) - FSBB_DEFAULT_CURRENT_LIMIT) > 0.15)
                throw std::runtime_error("FSBB current loop did not track its setpoint");
        }
        if constexpr (BUILD_LEVEL == 3)
        {
            const double reference = FSBB_DEFAULT_OUTPUT_VOLTAGE;
            if (before_step_.count == 0U ||
                std::abs(before_step_.mean(before_step_.vout) - reference) > voltage_tolerance ||
                std::abs(tail_.mean(tail_.vout) - reference) > voltage_tolerance ||
                tail_.mean(tail_.boost) <= before_step_.mean(before_step_.boost) + 0.10)
                throw std::runtime_error("FSBB voltage loop did not track in both buck and boost");
        }
    }

    void finalize_peripherals() override {}
    void finalize_embedded_chip() override {}

    void print_summary(std::ostream& out) const
    {
        out << std::setprecision(9)
            << "FSBB BUILD_LEVEL=" << BUILD_LEVEL << ", ADC SOC=" << adc_.trigger_count()
            << ", control ISR=" << csp_cctl_controller_interrupt_count()
            << ", background calls=" << background_.total_executions()
            << ", faults=" << g_fsbb_faults << '\n'
            << "  Tail mean: Vin=" << tail_.mean(tail_.vin) << " V, Vout=" << tail_.mean(tail_.vout)
            << " V, IL=" << tail_.mean(tail_.il) << " A, Iout=" << tail_.mean(tail_.iout)
            << " A; PWM buck high=" << tail_.mean(tail_.buck)
            << ", boost low=" << tail_.mean(tail_.boost) << '\n'
            << "  Peak Vout=" << peak_vout_ << " V, peak |IL|=" << peak_il_ << " A\n";
        if constexpr (BUILD_LEVEL == 3)
        {
            if (before_step_.count != 0U)
                out << "  Before input step: Vin=" << before_step_.mean(before_step_.vin)
                    << " V, Vout=" << before_step_.mean(before_step_.vout)
                    << " V; boost low=" << before_step_.mean(before_step_.boost) << '\n';
            if (std::isfinite(step_min_vout_))
            {
                out << "  First 150 ms at plant resolution: Vout=" << step_min_vout_ << " to "
                    << step_max_vout_ << " V, minimum at +" << (step_min_time_ - CCTL_SIM_VIN_STEP_S) * 1e3
                    << " ms\n";
                if (step_last_outside_time_ >= CCTL_SIM_VIN_STEP_S)
                    out << "  Last Vout outside +/-2% at +"
                        << (step_last_outside_time_ - CCTL_SIM_VIN_STEP_S) * 1e3 << " ms\n";
            }
        }
    }

private:
    bool validate_;
    Circuit circuit_;
    Circuit::Inputs inputs_{};
    Pwm buck_ = Pwm::make(pwm_clock, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP,
        CTRL_PWM_DEADBAND_CMP, false, cctl::ti_epwm_trigger_event::compare_b_up, 0U);
    Pwm boost_ = Pwm::make(pwm_clock, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP,
        CTRL_PWM_DEADBAND_CMP, false);
    Adc adc_ = Adc::make(CTRL_ADC_RESOLUTION, sim_real_gt(CTRL_ADC_VOLTAGE_REF));
    host::compute_budget_scheduler background_{plant_step, CCTL_SIM_USER_CODE_FREQUENCY_HZ};
    host::compute_budget_scheduler recording_{plant_step, CCTL_SIM_CIRCUIT_RECORD_FREQUENCY_HZ};
    bool adc_trigger_{};
    bool ever_enabled_{};
    std::size_t plant_steps_{};
    SteadyWindow before_step_;
    SteadyWindow tail_;
    double peak_vout_{};
    double peak_il_{};
    double step_min_vout_ = std::numeric_limits<double>::infinity();
    double step_max_vout_ = -std::numeric_limits<double>::infinity();
    double step_min_time_{};
    double step_last_outside_time_ = -1.0;
};

std::unique_ptr<FsbbSimulation> topology;
std::unique_ptr<host::simulation_system> system_model;

host::simulation_output_config csv_output(const std::string& filename, const char* stream,
    std::size_t size, const char* header, void (*write)(const void*, std::ostream&))
{
    const std::filesystem::path path(filename);
    host::simulation_output_config result;
    result.name = stream;
    result.path = (path.parent_path() / (path.stem().string() + "_" + stream + ".csv")).string();
    result.record_size = size;
    result.header = header;
    result.write_record = write;
    return result;
}
} // namespace

extern "C" void csp_cctl_project_configure(void)
{
    host::configure_build_information({Circuit::matrix_backend, Circuit::matrix_storage,
        FSBB_BUILD_CONFIGURATION, FSBB_OPTIMIZED_BUILD != 0});
    const auto& options = host::command_line();
    if (options.print_build_info)
        return;
    topology = std::make_unique<FsbbSimulation>(
        !options.supervised && !options.continuous && !options.duration_overridden);
    system_model = std::make_unique<host::simulation_system>(*topology, *topology, *topology);

    host::simulation_config config;
    config.plant_step_s = plant_step;
    config.total_steps = static_cast<std::size_t>(simulation_duration / plant_step + 0.5);
    config.pause_on_exit = false;
    config.console_title = "GMP CCTL Four-Switch Buck-Boost";
    config.outputs.push_back(csv_output(options.output_path, "circuit",
        sizeof(CircuitRecord), circuit_csv_header, write_circuit_record));
    config.outputs.push_back(csv_output(options.output_path, "control",
        sizeof(ControlRecord), control_csv_header, write_control_record));

    host::simulation_callbacks callbacks;
    callbacks.initialize = [] { system_model->initialize(); };
    callbacks.step = [](std::size_t index, double time, host::simulation_runtime& runtime) {
        system_model->step(index, time, runtime);
    };
    callbacks.finalize = [] { system_model->finalize(); };
    callbacks.print_summary = [](std::ostream& out) { topology->print_summary(out); };
    host::configure_simulation(std::move(config), std::move(callbacks));
}
