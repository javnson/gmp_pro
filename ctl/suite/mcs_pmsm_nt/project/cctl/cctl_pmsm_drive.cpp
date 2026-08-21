/**
 * @file cctl_pmsm_drive.cpp
 * @brief PMSM plant composition and project entry for the hosted CCTL CSP.
 */

#include "pmsmcircuit.hpp"

#include <cctl/circuit_model/pmsm_cs.hpp>
#include <csp.general.hpp>
#include <gmp_core.hpp>
#include <mcu_simulation.hpp>
#include <xplt.peripheral.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#ifndef CCTL_SIM_MATRIX_BACKEND_NAME
#define CCTL_SIM_MATRIX_BACKEND_NAME "unknown"
#endif
#ifndef CCTL_SIM_BUILD_CONFIGURATION
#define CCTL_SIM_BUILD_CONFIGURATION "unknown"
#endif
#ifndef CCTL_SIM_OPTIMIZED_BUILD
#define CCTL_SIM_OPTIMIZED_BUILD 0
#endif

namespace
{

constexpr sim_real_gt kPlantStepS = sim_real_gt(CCTL_SIM_PLANT_STEP_S);
constexpr sim_real_gt kControlStepS =
    sim_real_gt(1.0L / CCTL_SIM_CONTROL_FREQUENCY_HZ);
constexpr sim_real_gt kSimulationDurationS =
    sim_real_gt(CCTL_SIM_DURATION_S);
constexpr sim_real_gt kTbclkCountsPerPlantStepExact =
    kPlantStepS * CCTL_SIM_EPWM_TBCLK_HZ;
constexpr std::uint64_t kTbclkCountsPerPlantStep =
    static_cast<std::uint64_t>(kTbclkCountsPerPlantStepExact + 0.5);

static_assert(PmsmCircuit::normal_step_s > kPlantStepS * 0.999999999 &&
                  PmsmCircuit::normal_step_s < kPlantStepS * 1.000000001,
              "SDPE plant step does not match the generated main circuit");
static_assert(CTRL_PWM_CMP_MAX == CCTL_SIM_EPWM_PERIOD_COUNT,
              "controller and ePWM compare ranges must match");
static_assert(CTRL_POS_ENC_FS == CCTL_SIM_EQEP_COUNTS_PER_REV,
              "controller and eQEP resolutions must match");
static_assert(CCTL_ADC_COUNT == 7,
              "the generated VADC contract requires seven ADC channels");
static_assert(CCTL_SIM_PMSM_INTEGRATION_ORDER == 1 ||
                  CCTL_SIM_PMSM_INTEGRATION_ORDER == 2 ||
                  CCTL_SIM_PMSM_INTEGRATION_ORDER == 4,
              "PMSM integration order must be 1, 2, or 4");
static_assert(CCTL_SIM_USER_CODE_FREQUENCY_HZ > 0.0,
              "simulated MCU user-code frequency must be positive");
static_assert(kTbclkCountsPerPlantStep > 0U &&
                  kTbclkCountsPerPlantStepExact -
                          static_cast<sim_real_gt>(kTbclkCountsPerPlantStep) <
                      1.0e-9 &&
                  static_cast<sim_real_gt>(kTbclkCountsPerPlantStep) -
                          kTbclkCountsPerPlantStepExact <
                      1.0e-9,
              "plant step must contain an integer number of ePWM TBCLK counts");

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

/** @return PMSM parameters converted from the selected SDPE motor preset. */
cctl::pmsm_cs_parameters<sim_real_gt> motor_parameters()
{
    cctl::pmsm_cs_parameters<sim_real_gt> parameters;
    parameters.sample_period_s = kPlantStepS;
    parameters.stator_resistance_ohm = MOTOR_PARAM_RS;
    parameters.d_axis_inductance_h = MOTOR_PARAM_LD;
    parameters.q_axis_inductance_h = MOTOR_PARAM_LQ;
    parameters.permanent_magnet_flux_wb = MOTOR_PARAM_FLUX;
    parameters.pole_pairs = static_cast<std::uint16_t>(MOTOR_PARAM_POLE_PAIRS);
    /* SDPE motor presets store g*cm^2 and uN*m*s/rad; the plant uses SI. */
    parameters.inertia_kg_m2 = MOTOR_PARAM_INERTIA * 1.0e-7;
    parameters.viscous_friction_nm_s = MOTOR_PARAM_FRICTION * 1.0e-6;
    parameters.integration_method = static_cast<cctl::pmsm_cs_integration_method>(
        CCTL_SIM_PMSM_INTEGRATION_ORDER);
    return parameters;
}

/** Map the three simulated half bridges to the generated netlist ports. */
void set_gate_inputs(PmsmCircuit::Inputs &input,
                     const mcs::cctl_xplt::epwm_outputs &output)
{
    /* Project-local PMSM.CIR maps A->PWM1/2, B->PWM3/4, C->PWM5/6. */
    input.PWM1 = output[0].upper;
    input.PWM2 = output[0].lower;
    input.PWM3 = output[1].upper;
    input.PWM4 = output[1].lower;
    input.PWM5 = output[2].upper;
    input.PWM6 = output[2].lower;
}

/** @return True when the handwritten phase-to-PWM mapping is unchanged. */
bool verify_gate_phase_mapping() noexcept
{
    mcs::cctl_xplt::epwm_outputs output{};
    PmsmCircuit::Inputs input;
    output[0] = {1U, 0U, false};
    output[1] = {0U, 1U, false};
    output[2] = {0U, 1U, false};
    set_gate_inputs(input, output);
    if (input.PWM1 != 1U || input.PWM2 != 0U || input.PWM3 != 0U ||
        input.PWM4 != 1U || input.PWM5 != 0U || input.PWM6 != 1U)
        return false;

    output[0] = {0U, 1U, false};
    output[1] = {1U, 0U, false};
    set_gate_inputs(input, output);
    if (input.PWM1 != 0U || input.PWM2 != 1U || input.PWM3 != 1U ||
        input.PWM4 != 0U || input.PWM5 != 0U || input.PWM6 != 1U)
        return false;

    output[1] = {0U, 1U, false};
    output[2] = {1U, 0U, false};
    set_gate_inputs(input, output);
    return input.PWM1 == 0U && input.PWM2 == 1U && input.PWM3 == 0U &&
           input.PWM4 == 1U && input.PWM5 == 1U && input.PWM6 == 0U;
}

/** @brief Project wiring for the standard chip/peripheral/circuit interfaces. */
class pmsm_drive_topology final
    : public gmp::csp::cctl::embedded_chip_simulation,
      public gmp::csp::cctl::peripheral_simulation,
      public gmp::csp::cctl::circuit_simulation
{
  public:
    /** Construct the coupled plant and optional sparse profiler. */
    explicit pmsm_drive_topology(bool profile_enabled = false)
        : motor_(motor_parameters()), profile_enabled_(profile_enabled)
    {
    }

    /** Controller storage is initialized by ctl_init() before this hook. */
    void initialize_embedded_chip() override
    {
        user_code_scheduler_.initialize(
            kPlantStepS, CCTL_SIM_USER_CODE_FREQUENCY_HZ);
    }

    /** Reset and validate the project MCU peripheral models. */
    void initialize_peripherals() override
    {
        mcu_.initialize();
    }

    /** Validate wiring and settle the generated electrical circuit. */
    void initialize_circuit() override
    {
        if (!verify_gate_phase_mapping())
            throw std::runtime_error("ePWM-to-netlist phase mapping self-test failed");

        for (std::size_t index = 0U; index < CCTL_SIM_STARTUP_SHORT_STEPS; ++index)
        {
            PmsmCircuit::Inputs startup_input;
            startup_input.VS1 = CCTL_SIM_DC_BUS_V;
            inverter_output_ = inverter_.step_short(startup_input);
        }
    }

    /** Spend the simulated MCU user-code budget; control math is ISR-owned. */
    void step_embedded_chip(
        const gmp::csp::cctl::simulation_step_context &) override
    {
        const std::size_t executions = user_code_scheduler_.consume();
        for (std::size_t index = 0U; index < executions; ++index)
        {
            mainloop();
            ctl_mainloop();
        }
    }

    /** Apply ePWM outputs and motor feedback to the circuit input ports. */
    void apply_peripheral_outputs(
        const gmp::csp::cctl::simulation_step_context &context) override
    {
        sample_profile_step_ =
            profile_enabled_ &&
            context.step_index % kProfileSamplePeriod == 0U;
        profile_begin_ = sample_profile_step_ ? profile_clock::now()
                                              : profile_clock::time_point{};
        pwm_output_ = mcu_.sample_epwm(
            static_cast<std::uint64_t>(context.step_index) *
            kTbclkCountsPerPlantStep);
        inverter_input_ = {};
        set_gate_inputs(inverter_input_, pwm_output_);
        if ((inverter_input_.PWM1 && inverter_input_.PWM2) ||
            (inverter_input_.PWM3 && inverter_input_.PWM4) ||
            (inverter_input_.PWM5 && inverter_input_.PWM6))
            throw std::runtime_error("complementary ePWM overlap");
        inverter_input_.IPMSM1_A = motor_.output.phase_current_a[0];
        inverter_input_.IPMSM1_B = motor_.output.phase_current_a[1];
        inverter_input_.IPMSM1_C = motor_.output.phase_current_a[2];
        inverter_input_.VS1 = CCTL_SIM_DC_BUS_V;
        profile_after_peripheral_ =
            sample_profile_step_ ? profile_clock::now()
                                 : profile_clock::time_point{};
    }

    /** Advance the generated circuit and current-source PMSM plant. */
    void step_circuit(
        const gmp::csp::cctl::simulation_step_context &context) override
    {
        inverter_output_ = inverter_.step_normal(inverter_input_);
        profile_after_circuit_ =
            sample_profile_step_ ? profile_clock::now()
                                 : profile_clock::time_point{};

        const double load_torque =
            context.time_s >= CCTL_SIM_LOAD_TORQUE_START_S
                                       ? CCTL_SIM_LOAD_TORQUE_NM
                                       : 0.0;
        const auto &motor_output = motor_.step(
            inverter_output_.VPMSM1_A, inverter_output_.VPMSM1_B,
            inverter_output_.VPMSM1_C, load_torque);
        profile_after_motor_ =
            sample_profile_step_ ? profile_clock::now()
                                 : profile_clock::time_point{};
        for (double current : motor_output.phase_current_a)
        {
            if (!std::isfinite(current))
                throw std::runtime_error("non-finite motor current");
            maximum_current_a_ = std::max(maximum_current_a_, std::abs(current));
        }
        if (!std::isfinite(motor_output.mechanical_speed_rpm))
            throw std::runtime_error("non-finite motor speed");
        if (context.time_s >= kSimulationDurationS - 50.0e-3)
        {
            final_window_speed_sum_ += motor_output.mechanical_speed_rpm;
            ++final_window_count_;
        }
    }

    /** Sample circuit outputs and synchronously dispatch any ADC interrupt. */
    void sample_peripheral_inputs(
        const gmp::csp::cctl::simulation_step_context &context) override
    {
        if (pwm_output_[0].adc_trigger)
        {
            if (pwm_output_[1].adc_trigger || pwm_output_[2].adc_trigger)
                throw std::runtime_error("more than one ePWM module drives ADC SOC");
            if (mcu_.output_enabled() &&
                !mcs::cctl_xplt::mcu_simulation::all_low_sides_conducting(
                    pwm_output_))
                throw std::runtime_error(
                    "ADC SOC occurred outside the three-low-side sampling window");

            mcs::cctl_xplt::adc_pin_voltages adc_inputs;
            adc_inputs.dc_link_voltage = inverter_output_.VADC_VDC;
            adc_inputs.phase_voltage = {inverter_output_.VADC_VA,
                                        inverter_output_.VADC_VB,
                                        inverter_output_.VADC_VC};
            adc_inputs.phase_current = {inverter_output_.VADC_IA,
                                        inverter_output_.VADC_IB,
                                        inverter_output_.VADC_IC};
            mcu_.stage_adc_inputs(adc_inputs);
            mcu_.trigger_adc([this, &context] {
                adc_interrupt(context.time_s, context.runtime);
            });
        }
        if (sample_profile_step_)
        {
            const auto profile_end = profile_clock::now();
            profile_peripheral_ns_ += elapsed_ns(profile_begin_,
                                                 profile_after_peripheral_);
            profile_circuit_ns_ += elapsed_ns(profile_after_peripheral_,
                                              profile_after_circuit_);
            profile_motor_ns_ += elapsed_ns(profile_after_circuit_,
                                            profile_after_motor_);
            profile_housekeeping_ns_ += elapsed_ns(profile_after_motor_,
                                                   profile_end);
            ++profile_samples_;
        }
    }

    /** Validate end-of-run scheduling, current, and closed-loop speed. */
    void finalize_circuit() override
    {
        if (final_window_count_ == 0U)
            throw std::runtime_error("final speed window is empty");
        mean_final_speed_rpm_ =
            final_window_speed_sum_ / static_cast<double>(final_window_count_);

        const std::size_t expected_controller_steps = static_cast<std::size_t>(
            kSimulationDurationS / kControlStepS + 0.5);
        const std::size_t expected_user_code_steps = static_cast<std::size_t>(
            kSimulationDurationS * CCTL_SIM_USER_CODE_FREQUENCY_HZ + 0.5);
        if (!ever_enabled_ || controller_steps_ != expected_controller_steps ||
            mcu_.adc_trigger_count() != expected_controller_steps ||
            user_code_scheduler_.total_executions() != expected_user_code_steps)
            throw std::runtime_error("controller scheduling or CiA402 enable failed");
        if (maximum_current_a_ > 20.0)
            throw std::runtime_error("phase current exceeded the expected range");
        if (mean_final_speed_rpm_ < 260.0 || mean_final_speed_rpm_ > 340.0)
            throw std::runtime_error(
                "closed-loop speed did not approach the 300 rpm command");
    }

    /** Peripheral shutdown requires no work after circuit validation. */
    void finalize_peripherals() override
    {
    }

    /** Hosted chip shutdown is owned by the common CSP. */
    void finalize_embedded_chip() override
    {
    }

    /** Print project-specific plant and controller summary values. */
    void print_summary(std::ostream &stream) const
    {
        stream << std::setprecision(10)
               << "PMSM drive topology:\n"
               << "  main circuit backend=" << PmsmCircuit::matrix_backend
               << '/' << PmsmCircuit::matrix_storage
               << ", DC bus=" << CCTL_SIM_DC_BUS_V << " V\n"
               << "  motor integration order="
               << CCTL_SIM_PMSM_INTEGRATION_ORDER << " at plant step\n"
               << "  plant/control step: " << kPlantStepS << " s / "
               << kControlStepS << " s, ADC SOC CMPB="
               << CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT
               << "\n  controller steps=" << controller_steps_
               << ", ADC triggers=" << mcu_.adc_trigger_count()
               << ", user-code steps="
               << user_code_scheduler_.total_executions()
               << ", PWM enabled=" << ever_enabled_
               << "\n  mean/final speed=" << mean_final_speed_rpm_ << " / "
               << motor_.output.mechanical_speed_rpm
               << " rpm\n  maximum phase current=" << maximum_current_a_
               << " A, final torque=" << motor_.output.electromagnetic_torque_nm
               << " N*m\n";
        if (profile_enabled_ && profile_samples_ != 0U)
        {
            const double samples = static_cast<double>(profile_samples_);
            const double peripheral = profile_peripheral_ns_ / samples;
            const double circuit = profile_circuit_ns_ / samples;
            const double motor = profile_motor_ns_ / samples;
            const double housekeeping = profile_housekeeping_ns_ / samples;
            const double total = peripheral + circuit + motor + housekeeping;
            stream << "  sampled profile: peripheral=" << peripheral
                   << " ns, circuit=" << circuit << " ns, motor=" << motor
                   << " ns, housekeeping=" << housekeeping << " ns\n"
                   << "  controller profile: "
                   << (profile_controller_samples_ == 0U
                           ? 0.0
                           : static_cast<double>(profile_controller_ns_) /
                                 static_cast<double>(profile_controller_samples_))
                   << " ns/ISR; sampled plant total=" << total << " ns\n";
        }
    }

  private:
    /** Execute the ADC-complete ISR and publish one decimated CSV record. */
    void adc_interrupt(double time_s,
                       gmp::csp::cctl::simulation_runtime &runtime)
    {
        const auto profile_begin = profile_enabled_ ? profile_clock::now()
                                                    : profile_clock::time_point{};
        if (!mcu_.adc_interrupt_pending())
            throw std::runtime_error("ADC interrupt dispatched without completion");
        mcu_.transfer_adc_results_to_controller();
        mcu_.sample_encoder(motor_.output.mechanical_angle_rad);

        cctl_adc_interrupt();
        ++controller_steps_;

        mcu_.transfer_pwm_from_controller();
        ever_enabled_ = ever_enabled_ || mcu_.output_enabled();
        mcu_.acknowledge_adc_interrupt();

        const simulation_record record = make_record(time_s);
        runtime.interface_transfer(&record, sizeof(record));
        if (profile_enabled_)
        {
            profile_controller_ns_ += elapsed_ns(profile_begin, profile_clock::now());
            ++profile_controller_samples_;
        }
    }

    /** Build one trivially-copyable observation for the CSP SPSC ring. */
    simulation_record make_record(double time_s) const
    {
        simulation_record record;
        record.time_s = time_s;
        record.enabled = mcu_.output_enabled() ? 1U : 0U;
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
    cctl::pmsm_cs<sim_real_gt> motor_;
    mcs::cctl_xplt::mcu_simulation mcu_;
    gmp::csp::cctl::compute_budget_scheduler user_code_scheduler_{
        kPlantStepS, CCTL_SIM_USER_CODE_FREQUENCY_HZ};
    mcs::cctl_xplt::epwm_outputs pwm_output_{};
    PmsmCircuit::Inputs inverter_input_{};
    PmsmCircuit::Outputs inverter_output_{};
    double maximum_current_a_{};
    double final_window_speed_sum_{};
    double mean_final_speed_rpm_{};
    std::size_t final_window_count_{};
    std::size_t controller_steps_{};
    bool ever_enabled_{};
    using profile_clock = std::chrono::steady_clock;
    static constexpr std::size_t kProfileSamplePeriod = 4096U; // sparse hot-path sampling
    static std::uint64_t elapsed_ns(profile_clock::time_point begin,
                                    profile_clock::time_point end) noexcept
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)
                .count());
    }
    bool profile_enabled_{};
    bool sample_profile_step_{};
    profile_clock::time_point profile_begin_{};
    profile_clock::time_point profile_after_peripheral_{};
    profile_clock::time_point profile_after_circuit_{};
    profile_clock::time_point profile_after_motor_{};
    std::uint64_t profile_peripheral_ns_{};
    std::uint64_t profile_circuit_ns_{};
    std::uint64_t profile_motor_ns_{};
    std::uint64_t profile_housekeeping_ns_{};
    std::uint64_t profile_controller_ns_{};
    std::size_t profile_samples_{};
    std::size_t profile_controller_samples_{};
};

/** Serialize one observation record on the asynchronous file worker. */
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

/** Keep project callbacks valid until gmp_csp_exit() finalizes the runtime. */
std::unique_ptr<pmsm_drive_topology> configured_topology;
std::unique_ptr<gmp::csp::cctl::simulation_system> configured_system;

} // namespace

extern "C"
{

/**
 * @brief Apply project initialization after setup_peripheral() and ctl_init().
 */
void init(void)
{
    flag_enable_adc_calibrator = 0;
    for (time_gt &delay : cia402_sm.minimum_transit_delay)
        delay = 0;

    gmp::csp::cctl::build_information build;
    build.backend = CCTL_SIM_MATRIX_BACKEND_NAME;
    build.storage = PmsmCircuit::matrix_storage;
    build.configuration = CCTL_SIM_BUILD_CONFIGURATION;
    build.optimized = CCTL_SIM_OPTIMIZED_BUILD != 0;
    gmp::csp::cctl::configure_build_information(std::move(build));

    const gmp::csp::cctl::command_line_options &options =
        gmp::csp::cctl::command_line();
    if (options.print_build_info)
        return;

    configured_topology =
        std::make_unique<pmsm_drive_topology>(options.profile_enabled);
    configured_system = std::make_unique<gmp::csp::cctl::simulation_system>(
        *configured_topology, *configured_topology, *configured_topology);

    gmp::csp::cctl::simulation_config config;
    config.total_steps = static_cast<std::size_t>(
        kSimulationDurationS / kPlantStepS + 0.5);
    config.plant_step_s = kPlantStepS;
    config.record_size = sizeof(simulation_record);
    config.output_ring_bytes = CCTL_SIM_OUTPUT_RING_BYTES;
    config.output_batch_bytes = CCTL_SIM_OUTPUT_BATCH_BYTES;
    config.progress_interval_ms = CCTL_SIM_PROGRESS_INTERVAL_MS;
    config.step_chunk_size = CCTL_SIM_STEP_CHUNK_STEPS;
    config.output_header = kCsvHeader;
    config.console_title = "GMP CCTL Motor Simulation Kit";
    if (CCTL_SIM_OPTIMIZED_BUILD == 0)
        config.console_title +=
            "\n\n[PERFORMANCE WARNING] This is an unoptimized "
            CCTL_SIM_BUILD_CONFIGURATION
            " build. Use build_test.bat or configure CMake with Release.";
    config.execution_label =
        std::string(CCTL_SIM_MATRIX_BACKEND_NAME) + "/" +
        PmsmCircuit::matrix_storage + "  build=" +
        CCTL_SIM_BUILD_CONFIGURATION + "  optimized=" +
        (CCTL_SIM_OPTIMIZED_BUILD != 0 ? "yes" : "no");
    config.console_bar_width = 64U;
    config.pause_on_exit = CCTL_SIM_PAUSE_ON_EXIT != 0;

    gmp::csp::cctl::simulation_callbacks callbacks;
    callbacks.initialize = [] { configured_system->initialize(); };
    callbacks.step = [](std::size_t index, double time_s,
                        gmp::csp::cctl::simulation_runtime &host) {
        configured_system->step(index, time_s, host);
    };
    callbacks.finalize = [] { configured_system->finalize(); };
    callbacks.write_record = write_record;
    callbacks.print_summary = [](std::ostream &stream) {
        configured_topology->print_summary(stream);
    };
    gmp::csp::cctl::configure_simulation(std::move(config),
                                         std::move(callbacks));
}

/** @brief Project background hook; the finite simulation runs in the CSP. */
void mainloop(void)
{
}

} // extern "C"
