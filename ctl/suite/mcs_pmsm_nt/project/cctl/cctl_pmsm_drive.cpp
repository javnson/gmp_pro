#include "pmsmcircuit.hpp"

#include <cctl/circuit_model/pmsm_cs.hpp>
#include <cctl/peripheral_if/peripheral_if.hpp>
#include <csp_cctl.hpp>
#include <gmp_core.h>
#include <xplt.peripheral.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#ifndef CCTL_SIM_MATRIX_BACKEND_NAME
#define CCTL_SIM_MATRIX_BACKEND_NAME "unknown"
#endif

namespace
{

constexpr double kPlantStepS = CCTL_SIM_PLANT_STEP_S;
constexpr double kControlStepS = 1.0 / CCTL_SIM_CONTROL_FREQUENCY_HZ;
constexpr double kSimulationDurationS = CCTL_SIM_DURATION_S;

static_assert(PmsmCircuit::normal_step_s > kPlantStepS * 0.999999999 &&
                  PmsmCircuit::normal_step_s < kPlantStepS * 1.000000001,
              "SDPE plant step does not match the generated main circuit");
static_assert(CTRL_PWM_CMP_MAX == CCTL_SIM_EPWM_PERIOD_COUNT,
              "controller and ePWM compare ranges must match");
static_assert(CTRL_POS_ENC_FS == CCTL_SIM_EQEP_COUNTS_PER_REV,
              "controller and eQEP resolutions must match");
static_assert(CCTL_ADC_COUNT == 7,
              "the generated VADC contract requires seven ADC channels");

struct simulation_record
{
    double time_s{};
    std::uint32_t enabled{};
    std::int32_t compare_a{};
    std::int32_t compare_b{};
    std::int32_t compare_c{};
    double phase_voltage_v[3]{};
    double phase_current_a[3]{};
    double d_axis_current_a{};
    double q_axis_current_a{};
    double electromagnetic_torque_nm{};
    double load_torque_nm{};
    double mechanical_speed_rpm{};
    std::uint32_t encoder_count{};
    std::uint32_t adc_code[7]{};
};

static_assert(std::is_trivially_copyable<simulation_record>::value,
              "the CSP output ring requires trivially copyable records");

cctl::pmsm_cs_parameters<double> motor_parameters()
{
    cctl::pmsm_cs_parameters<double> parameters;
    parameters.sample_period_s = kPlantStepS;
    parameters.stator_resistance_ohm = MOTOR_PARAM_RS;
    parameters.d_axis_inductance_h = MOTOR_PARAM_LD;
    parameters.q_axis_inductance_h = MOTOR_PARAM_LQ;
    parameters.permanent_magnet_flux_wb = MOTOR_PARAM_FLUX;
    parameters.pole_pairs = static_cast<std::uint16_t>(MOTOR_PARAM_POLE_PAIRS);
    /* SDPE motor presets store g*cm^2 and uN*m*s/rad; the plant uses SI. */
    parameters.inertia_kg_m2 = MOTOR_PARAM_INERTIA * 1.0e-7;
    parameters.viscous_friction_nm_s = MOTOR_PARAM_FRICTION * 1.0e-6;
    return parameters;
}

cctl::ti_adc_config<double> adc_config()
{
    cctl::ti_adc_config<double> config;
    config.resolution_bits = CCTL_SIM_ADC_RESOLUTION_BITS;
    config.reference_voltage_v = CCTL_SIM_ADC_REFERENCE_V;
    return config;
}

cctl::ti_epwm_config<double> epwm_config(bool adc_trigger = false)
{
    cctl::ti_epwm_config<double> config;
    config.time_base_clock_hz = CCTL_SIM_EPWM_TBCLK_HZ;
    config.period_count = CCTL_SIM_EPWM_PERIOD_COUNT;
    config.rising_edge_delay_count = CCTL_SIM_EPWM_DBRED_COUNT;
    config.falling_edge_delay_count = CCTL_SIM_EPWM_DBFED_COUNT;
    config.upper_active_above_compare = true;
    config.adc_trigger_event =
        adc_trigger ? cctl::ti_epwm_trigger_event::compare_b_up
                    : cctl::ti_epwm_trigger_event::disabled;
    config.adc_trigger_compare_count = CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT;
    return config;
}

bool verify_peripheral_models()
{
    cctl::ti_adc<double, 1U> adc(adc_config());
    bool adc_interrupt_called = false;
    adc.set_input_voltage(0U, CCTL_SIM_ADC_REFERENCE_V * 0.5);
    adc.trigger([&adc_interrupt_called] { adc_interrupt_called = true; });
    if (!adc_interrupt_called || !adc.interrupt_pending() ||
        adc.trigger_count() != 1U ||
        adc.result(0U) != (adc.maximum_code() + 1U) / 2U)
        return false;
    adc.acknowledge_interrupt();
    if (adc.interrupt_pending())
        return false;

    cctl::ti_eqep<double> eqep(CCTL_SIM_EQEP_COUNTS_PER_REV);
    if (eqep.sample_mechanical_angle(0.5 * 3.14159265358979323846) !=
        CCTL_SIM_EQEP_COUNTS_PER_REV / 4U)
        return false;

    cctl::ti_epwm<double> epwm(epwm_config(true));
    epwm.set_enabled(true);
    epwm.set_compare_a(CCTL_SIM_EPWM_PERIOD_COUNT / 2U);
    std::size_t adc_triggers = 0U;
    for (std::size_t index = 0U;
         index < 2U * (CCTL_SIM_EPWM_PERIOD_COUNT + 1U); ++index)
    {
        const auto gate = epwm.sample(
            static_cast<double>(index) / CCTL_SIM_EPWM_TBCLK_HZ);
        if (gate.upper != 0U && gate.lower != 0U)
            return false;
        if (gate.adc_trigger)
        {
            ++adc_triggers;
            if (gate.upper != 0U || gate.lower == 0U)
                return false;
        }
    }
    return adc_triggers == 1U &&
           std::abs(epwm.switching_frequency_hz() -
                    CCTL_SIM_CONTROL_FREQUENCY_HZ) < 1.0e-9;
}

void stage_adc_inputs(cctl::ti_adc<double, 8U> &adc,
                      const PmsmCircuit::Outputs &inverter_output)
{
    adc.set_input_voltage(CCTL_ADC_UDC, inverter_output.VADC_VDC);
    adc.set_input_voltage(CCTL_ADC_UA, inverter_output.VADC_VA);
    adc.set_input_voltage(CCTL_ADC_UB, inverter_output.VADC_VB);
    adc.set_input_voltage(CCTL_ADC_UC, inverter_output.VADC_VC);
    /*
     * PMSM.CIR routes the physical A/B/C low-side shunts to the nets named
     * IC/IB/IA respectively.  Keep the controller channels in physical phase
     * order; the VADC names describe PCB ADC pins, not motor phase order.
     */
    adc.set_input_voltage(CCTL_ADC_IA, inverter_output.VADC_IC);
    adc.set_input_voltage(CCTL_ADC_IB, inverter_output.VADC_IB);
    adc.set_input_voltage(CCTL_ADC_IC, inverter_output.VADC_IA);
}

using epwm_outputs = std::array<cctl::ti_epwm_gate_pair, 3U>;

epwm_outputs sample_epwm(std::array<cctl::ti_epwm<double>, 3U> &epwm,
                         double time_s)
{
    return {epwm[0].sample(time_s), epwm[1].sample(time_s),
            epwm[2].sample(time_s)};
}

void set_gate_inputs(PmsmCircuit::Inputs &input, const epwm_outputs &output)
{
    /* Project-local PMSM.CIR maps A->PWM5/6, B->PWM3/4, C->PWM1/2. */
    input.PWM5 = output[0].upper;
    input.PWM6 = output[0].lower;
    input.PWM3 = output[1].upper;
    input.PWM4 = output[1].lower;
    input.PWM1 = output[2].upper;
    input.PWM2 = output[2].lower;
}

bool all_low_sides_conducting(const epwm_outputs &output) noexcept
{
    return std::all_of(output.begin(), output.end(), [](const auto &phase) {
        return phase.upper == 0U && phase.lower != 0U;
    });
}

class pmsm_drive_topology
{
  public:
    pmsm_drive_topology()
        : motor_(motor_parameters()), adc_(adc_config()),
          eqep_(CCTL_SIM_EQEP_COUNTS_PER_REV),
          epwm_{cctl::ti_epwm<double>(epwm_config(true)),
                cctl::ti_epwm<double>(epwm_config()),
                cctl::ti_epwm<double>(epwm_config())}
    {
    }

    void initialize()
    {
        if (!verify_peripheral_models())
            throw std::runtime_error("SDPE-configured TI peripheral self-test failed");

        setup_peripheral();
        ctl_init();
        flag_enable_adc_calibrator = 0;
        for (time_gt &delay : cia402_sm.minimum_transit_delay)
            delay = 0;

        for (std::size_t index = 0U; index < CCTL_SIM_STARTUP_SHORT_STEPS; ++index)
        {
            PmsmCircuit::Inputs startup_input;
            startup_input.VS1 = CCTL_SIM_DC_BUS_V;
            inverter_output_ = inverter_.step_short(startup_input);
        }
    }

    void step(std::size_t, double time_s,
              gmp::csp::cctl::simulation_runtime &runtime)
    {
        const epwm_outputs pwm_output = sample_epwm(epwm_, time_s);
        PmsmCircuit::Inputs inverter_input;
        set_gate_inputs(inverter_input, pwm_output);
        if ((inverter_input.PWM1 && inverter_input.PWM2) ||
            (inverter_input.PWM3 && inverter_input.PWM4) ||
            (inverter_input.PWM5 && inverter_input.PWM6))
            throw std::runtime_error("complementary ePWM overlap");
        inverter_input.IPMSM1_A = motor_.output.phase_current_a[0];
        inverter_input.IPMSM1_B = motor_.output.phase_current_a[1];
        inverter_input.IPMSM1_C = motor_.output.phase_current_a[2];
        inverter_input.VS1 = CCTL_SIM_DC_BUS_V;
        inverter_output_ = inverter_.step_normal(inverter_input);

        const double load_torque = time_s >= CCTL_SIM_LOAD_TORQUE_START_S
                                       ? CCTL_SIM_LOAD_TORQUE_NM
                                       : 0.0;
        const auto &motor_output = motor_.step(
            inverter_output_.VPMSM1_A, inverter_output_.VPMSM1_B,
            inverter_output_.VPMSM1_C, load_torque);
        for (double current : motor_output.phase_current_a)
        {
            if (!std::isfinite(current))
                throw std::runtime_error("non-finite motor current");
            maximum_current_a_ = std::max(maximum_current_a_, std::abs(current));
        }
        if (!std::isfinite(motor_output.mechanical_speed_rpm))
            throw std::runtime_error("non-finite motor speed");
        if (time_s >= kSimulationDurationS - 50.0e-3)
        {
            final_window_speed_sum_ += motor_output.mechanical_speed_rpm;
            ++final_window_count_;
        }

        if (pwm_output[0].adc_trigger)
        {
            if (pwm_output[1].adc_trigger || pwm_output[2].adc_trigger)
                throw std::runtime_error("more than one ePWM module drives ADC SOC");
            if (cctl_pwm_output_enabled != 0 &&
                !all_low_sides_conducting(pwm_output))
                throw std::runtime_error(
                    "ADC SOC occurred outside the three-low-side sampling window");

            stage_adc_inputs(adc_, inverter_output_);
            adc_.trigger([this, time_s, &runtime] {
                adc_interrupt(time_s, runtime);
            });
        }
    }

    void finalize()
    {
        if (final_window_count_ == 0U)
            throw std::runtime_error("final speed window is empty");
        mean_final_speed_rpm_ =
            final_window_speed_sum_ / static_cast<double>(final_window_count_);

        const std::size_t expected_controller_steps = static_cast<std::size_t>(
            kSimulationDurationS / kControlStepS + 0.5);
        if (!ever_enabled_ || controller_steps_ != expected_controller_steps ||
            adc_.trigger_count() != expected_controller_steps)
            throw std::runtime_error("controller scheduling or CiA402 enable failed");
        if (maximum_current_a_ > 20.0)
            throw std::runtime_error("phase current exceeded the expected range");
        if (mean_final_speed_rpm_ < 260.0 || mean_final_speed_rpm_ > 340.0)
            throw std::runtime_error(
                "closed-loop speed did not approach the 300 rpm command");
    }

    void print_summary(std::ostream &stream) const
    {
        stream << std::setprecision(10)
               << "PMSM drive topology:\n"
               << "  main circuit backend=" << PmsmCircuit::matrix_backend
               << '/' << PmsmCircuit::matrix_storage
               << ", DC bus=" << CCTL_SIM_DC_BUS_V << " V\n"
               << "  plant/control step: " << kPlantStepS << " s / "
               << kControlStepS << " s, ADC SOC CMPB="
               << CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT
               << "\n  controller steps=" << controller_steps_
               << ", ADC triggers=" << adc_.trigger_count()
               << ", PWM enabled=" << ever_enabled_
               << "\n  mean/final speed=" << mean_final_speed_rpm_ << " / "
               << motor_.output.mechanical_speed_rpm
               << " rpm\n  maximum phase current=" << maximum_current_a_
               << " A, final torque=" << motor_.output.electromagnetic_torque_nm
               << " N*m\n";
    }

  private:
    void adc_interrupt(double time_s,
                       gmp::csp::cctl::simulation_runtime &runtime)
    {
        if (!adc_.interrupt_pending())
            throw std::runtime_error("ADC interrupt dispatched without completion");
        for (std::size_t channel = 0U; channel < CCTL_ADC_COUNT; ++channel)
            cctl_adc_result[channel] =
                static_cast<adc_gt>(adc_.result(channel));
        cctl_encoder_position = eqep_.sample_mechanical_angle(
            motor_.output.mechanical_angle_rad);

        ctl_mainloop();
        gmp_base_ctl_step();
        cctl_advance_controller_tick();
        ++controller_steps_;

        for (std::size_t phase = 0U; phase < 3U; ++phase)
        {
            epwm_[phase].set_compare_a(cctl_pwm_compare[phase]);
            epwm_[phase].set_enabled(cctl_pwm_output_enabled != 0);
        }
        ever_enabled_ = ever_enabled_ || cctl_pwm_output_enabled != 0;
        adc_.acknowledge_interrupt();

        const simulation_record record = make_record(time_s);
        runtime.interface_transfer(&record, sizeof(record));
    }

    simulation_record make_record(double time_s) const
    {
        simulation_record record;
        record.time_s = time_s;
        record.enabled = cctl_pwm_output_enabled != 0 ? 1U : 0U;
        record.compare_a = static_cast<std::int32_t>(cctl_pwm_compare[0]);
        record.compare_b = static_cast<std::int32_t>(cctl_pwm_compare[1]);
        record.compare_c = static_cast<std::int32_t>(cctl_pwm_compare[2]);
        record.phase_voltage_v[0] = inverter_output_.VPMSM1_A;
        record.phase_voltage_v[1] = inverter_output_.VPMSM1_B;
        record.phase_voltage_v[2] = inverter_output_.VPMSM1_C;
        for (std::size_t phase = 0U; phase < 3U; ++phase)
            record.phase_current_a[phase] = motor_.output.phase_current_a[phase];
        record.d_axis_current_a = motor_.output.d_axis_current_a;
        record.q_axis_current_a = motor_.output.q_axis_current_a;
        record.electromagnetic_torque_nm = motor_.output.electromagnetic_torque_nm;
        record.load_torque_nm = time_s >= CCTL_SIM_LOAD_TORQUE_START_S
                                    ? CCTL_SIM_LOAD_TORQUE_NM
                                    : 0.0;
        record.mechanical_speed_rpm = motor_.output.mechanical_speed_rpm;
        record.encoder_count = cctl_encoder_position;
        for (std::size_t channel = 0U; channel < CCTL_ADC_COUNT; ++channel)
            record.adc_code[channel] =
                static_cast<std::uint32_t>(cctl_adc_result[channel]);
        return record;
    }

    PmsmCircuit inverter_;
    cctl::pmsm_cs<double> motor_;
    cctl::ti_adc<double, 8U> adc_;
    cctl::ti_eqep<double> eqep_;
    std::array<cctl::ti_epwm<double>, 3U> epwm_;
    PmsmCircuit::Outputs inverter_output_{};
    double maximum_current_a_{};
    double final_window_speed_sum_{};
    double mean_final_speed_rpm_{};
    std::size_t final_window_count_{};
    std::size_t controller_steps_{};
    bool ever_enabled_{};
};

void write_record(const void *data, std::ostream &stream)
{
    const auto &record = *static_cast<const simulation_record *>(data);
    stream << record.time_s << ',' << record.enabled << ',' << record.compare_a
           << ',' << record.compare_b << ',' << record.compare_c;
    for (double value : record.phase_voltage_v)
        stream << ',' << value;
    for (double value : record.phase_current_a)
        stream << ',' << value;
    stream << ',' << record.d_axis_current_a << ',' << record.q_axis_current_a
           << ',' << record.electromagnetic_torque_nm << ','
           << record.load_torque_nm << ',' << record.mechanical_speed_rpm << ','
           << record.encoder_count;
    for (std::uint32_t code : record.adc_code)
        stream << ',' << code;
    stream << '\n';
}

constexpr const char *kCsvHeader =
    "time_s,enabled,cmp_a,cmp_b,cmp_c,va_v,vb_v,vc_v,ia_a,ib_a,ic_a,"
    "id_a,iq_a,torque_nm,load_torque_nm,speed_rpm,encoder_count,"
    "adc_vdc,adc_va,adc_vb,adc_vc,adc_ia,adc_ib,adc_ic";

} // namespace

int main(int argc, char **argv)
{
    bool suppress_pause = false;
    bool request_realtime_priority = CCTL_SIM_REALTIME_PRIORITY != 0;
    std::string output_path = CCTL_SIM_OUTPUT_FILENAME;
    gmp::csp::cctl::simulation_runtime runtime;
    bool runtime_initialized = false;

    try
    {
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--no-pause")
                suppress_pause = true;
            else if (argument == "--realtime-priority")
                request_realtime_priority = true;
            else if (argument == "--normal-priority" ||
                     argument == "--no-realtime-priority")
                request_realtime_priority = false;
            else if (argument == "--output" && index + 1 < argc)
                output_path = argv[++index];
            else
                throw std::invalid_argument("unknown or incomplete argument: " + argument);
        }

        pmsm_drive_topology topology;
        gmp::csp::cctl::simulation_config config;
        config.total_steps = static_cast<std::size_t>(
            kSimulationDurationS / kPlantStepS + 0.5);
        config.plant_step_s = kPlantStepS;
        config.record_size = sizeof(simulation_record);
        config.output_ring_bytes = CCTL_SIM_OUTPUT_RING_BYTES;
        config.output_batch_bytes = CCTL_SIM_OUTPUT_BATCH_BYTES;
        config.progress_interval_ms = CCTL_SIM_PROGRESS_INTERVAL_MS;
        config.step_chunk_size = CCTL_SIM_STEP_CHUNK_STEPS;
        config.output_path = output_path;
        config.output_header = kCsvHeader;
        config.console_title = "GMP CCTL Motor Simulation Kit";
        config.execution_label = CCTL_SIM_MATRIX_BACKEND_NAME;
        config.console_bar_width = 64U;
        config.request_realtime_priority = request_realtime_priority;
        config.pause_on_exit = CCTL_SIM_PAUSE_ON_EXIT != 0;

        gmp::csp::cctl::simulation_callbacks callbacks;
        callbacks.initialize = [&topology] { topology.initialize(); };
        callbacks.step_range = [&topology](
                                   std::size_t begin, std::size_t end,
                                   gmp::csp::cctl::simulation_runtime &host) {
            for (std::size_t index = begin; index < end; ++index)
                topology.step(index, static_cast<double>(index) * kPlantStepS,
                              host);
        };
        callbacks.finalize = [&topology] { topology.finalize(); };
        callbacks.write_record = write_record;

        runtime.initialize(std::move(config), std::move(callbacks));
        runtime_initialized = true;
        const gmp::csp::cctl::simulation_summary summary = runtime.run();
        runtime.print_summary(std::cout);
        topology.print_summary(std::cout);
        std::cout << "  CSV: " << output_path << '\n';
        runtime.pause_if_requested(suppress_pause);
        return summary.success ? 0 : 1;
    }
    catch (const std::exception &error)
    {
        std::cerr << "[FAIL] " << error.what() << '\n';
        if (runtime_initialized)
            runtime.pause_if_requested(suppress_pause);
#if defined(_WIN32)
        else if (CCTL_SIM_PAUSE_ON_EXIT != 0 && !suppress_pause)
            std::system("@pause");
#endif
        return 1;
    }
}
