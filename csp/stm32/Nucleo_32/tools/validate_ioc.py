#!/usr/bin/env python3
"""Validate a Nucleo-32 IOC against its SDPE board entity and pin table."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Report:
    target: Path
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.errors.append(message)


def find_repo_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "ctl/hardware_preset/sdpe_schemas").is_dir() and (
            candidate / "csp/stm32"
        ).is_dir():
            return candidate
    raise RuntimeError("cannot locate the GMP Pro repository root")


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_ioc(path: Path) -> dict[str, str]:
    settings: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{line_number}: invalid IOC setting")
        key, value = line.split("=", 1)
        settings[key] = value
    return settings


def pin_signal(ioc: dict[str, str], pin: str) -> str | None:
    """Return a pin signal while ignoring CubeMX package-function suffixes."""
    for key, value in ioc.items():
        if not key.endswith(".Signal"):
            continue
        key_pin = key.removesuffix(".Signal").split("-", 1)[0]
        key_pin = key_pin.replace("\\ ", " ").split("(", 1)[0].strip()
        if key_pin == pin:
            if value.startswith("GPXTI"):
                return ioc.get(f"SH.{value}.0", value)
            return value
    return None


def selected_option(requirement: dict, macro: str) -> str:
    for option in requirement.get("option_macros", []):
        if option.get("macro") == macro:
            return str(option.get("value"))
    raise KeyError(f"missing option macro {macro}")


def peripheral_from_handle(handle: str, prefix: str) -> str:
    match = re.fullmatch(r"\(&h([a-z]+)(\d+)\)", handle)
    if not match or match.group(1) != prefix:
        raise ValueError(f"unsupported {prefix} handle expression: {handle}")
    return f"{prefix.upper()}{match.group(2)}"


def ioc_ips(ioc: dict[str, str]) -> set[str]:
    count = int(ioc.get("Mcu.IPNb", "0"))
    return {ioc.get(f"Mcu.IP{index}", "") for index in range(count)}


def check_pwm(report: Report, ioc: dict[str, str], timer: str) -> None:
    prefix = f"{timer}."
    for channel in range(1, 4):
        report.require(
            ioc.get(f"{timer}.Channel-PWM\\ Generation{channel}\\ CH{channel}\\ CH{channel}N")
            == f"TIM_CHANNEL_{channel}",
            f"{timer} is missing complementary PWM channel {channel}",
        )
    report.require(
        ioc.get(prefix + "CounterMode", "").startswith("TIM_COUNTERMODE_CENTERALIGNED"),
        f"{timer} is not center aligned",
    )
    report.require(ioc.get(prefix + "DeadTime") is not None, f"{timer} has no dead time")
    report.require(
        ioc.get(prefix + "TIM_MasterOutputTrigger") == "TIM_TRGO_OC4REF"
        or ioc.get(prefix + "TIM_MasterOutputTrigger2") == "TIM_TRGO2_OC4REF",
        f"{timer} does not trigger from OC4REF",
    )


def check_adc_feedback(
    report: Report, ioc: dict[str, str], entity: dict, pin_doc: str
) -> None:
    parameters = entity["parameters"]
    count = int(parameters["adc_fb_count"])
    regular_dma = bool(int(parameters.get("adc_regular_dma", "0")))
    fixed_sequence = regular_dma and ioc.get("ADC1.Sequencer") == "NOT_FULLY_CONFIGURABLE"
    selected_regular_channels = sorted(
        int(channel)
        for channel in re.findall(r"ADC_CHANNEL_(\d+)", ioc.get("ADC1.SelectedChannel", ""))
    )
    report.require(count >= 6, "board entity exposes fewer than six ADC feedback channels")
    seen_pins: set[str] = set()
    for index in range(count):
        pin = parameters[f"adc_fb{index}_pin"]
        handle = parameters[f"adc_fb{index}_handle"]
        rank_token = parameters[f"adc_fb{index}_rank"]
        adc = peripheral_from_handle(handle, "adc")
        if regular_dma:
            rank = str(int(rank_token) + 1) if rank_token.isdigit() else ""
            report.require(
                rank_token == str(index),
                f"FB{index} has invalid regular-DMA index {rank_token}",
            )
        else:
            match = re.fullmatch(r"ADC_INJECTED_RANK_([1-4])", rank_token)
            report.require(match is not None, f"FB{index} has invalid injected rank {rank_token}")
            if match is None:
                continue
            rank = match.group(1)
        signal = pin_signal(ioc, pin) or ""
        if signal.startswith("ADCx_IN"):
            shared_prefix = f"SH.{signal}."
            signal = next(
                (
                    value
                    for key, value in ioc.items()
                    if key.startswith(shared_prefix) and value.startswith(f"{adc}_IN")
                ),
                signal,
            )
            signal = signal.split(",", 1)[0]
        signal_match = re.fullmatch(rf"{adc}_INP?(\d+)", signal)
        report.require(signal_match is not None, f"FB{index} {pin} is not routed to {adc}")
        if signal_match:
            if regular_dma:
                channel = signal_match.group(1)
                if fixed_sequence:
                    report.require(
                        index < len(selected_regular_channels)
                        and selected_regular_channels[index] == int(channel),
                        f"FB{index} does not match the fixed ADC sequence at {signal}",
                    )
                else:
                    selected_rank = (
                        ioc.get(f"{adc}.Rank-{channel}#ChannelRegularConversion")
                        or ioc.get(f"{adc}.Rank-{channel}\\#ChannelRegularConversion")
                    )
                    report.require(
                        selected_rank == rank,
                        f"FB{index} {adc} regular rank {rank} does not select {signal}",
                    )
            else:
                selected_channel = (
                    ioc.get(f"{adc}.Rank{rank}_Channel")
                    or ioc.get(f"{adc}.InjectedChannel-{rank}#ChannelInjectedConversion")
                    or ioc.get(f"{adc}.InjectedChannel-{rank}\\#ChannelInjectedConversion")
                )
                report.require(
                    selected_channel == f"ADC_CHANNEL_{signal_match.group(1)}",
                    f"FB{index} {adc} injected rank {rank} does not select {signal}",
                )
        report.require(pin not in seen_pins, f"ADC feedback pin {pin} is duplicated")
        report.require(pin in pin_doc, f"ADC feedback pin {pin} is absent from pin_assign.md")
        seen_pins.add(pin)


def check_target(repo: Path, board_dir: Path) -> Report:
    ioc_path = next(iter(sorted(board_dir.glob("*.ioc"))), None)
    requirement_path = board_dir / "sdpe_mgr/sdpe_requirement.json"
    report = Report(board_dir)
    if ioc_path is None:
        report.errors.append("no IOC file found")
        return report
    if not requirement_path.is_file():
        report.errors.append("missing sdpe_mgr/sdpe_requirement.json")
        return report

    ioc = load_ioc(ioc_path)
    requirement = load_json(requirement_path)
    hardware = requirement.get("hardware", [])
    report.require(len(hardware) == 1, "target must select exactly one board entity")
    if len(hardware) != 1:
        return report
    entity_id = hardware[0].get("entity", "")
    entity_path = repo / f"ctl/hardware_preset/sdpe_src/mcu_board/{entity_id}.json"
    report.require(entity_path.is_file(), f"missing board entity {entity_id}")
    if not entity_path.is_file():
        return report
    entity = load_json(entity_path)
    parameters = entity["parameters"]
    regular_dma = bool(int(parameters.get("adc_regular_dma", "0")))
    trigger_bridge = bool(int(parameters.get("adc_trigger_bridge", "0")))
    pin_doc_path = board_dir / "pin_assign.md"
    report.require(pin_doc_path.is_file(), "missing pin_assign.md")
    pin_doc = pin_doc_path.read_text(encoding="utf-8") if pin_doc_path.is_file() else ""

    report.require(entity.get("schema") == "stm32_nucleo_32_board", "wrong SDPE schema")
    report.require(
        ioc.get("Mcu.UserName") == parameters["chip"],
        "IOC MCU does not match the SDPE board entity",
    )
    expected_ioc = (repo / parameters["ioc_file"]).resolve()
    report.require(ioc_path.resolve() == expected_ioc, "entity ioc_file points elsewhere")
    report.require(ioc.get("Mcu.Package") == "LQFP32", "target is not a 32-pin LQFP MCU")
    report.require(ioc.get("ProjectManager.LastFirmware") == "false", "LastFirmware must be false")
    report.require(int(parameters["has_base"]) == 1, "board does not declare base capability")
    report.require(
        int(parameters["has_control"]) == 1,
        "board does not declare control capability",
    )

    ips = ioc_ips(ioc)
    dma_ip = "GPDMA1" if ioc.get("Mcu.Family") == "STM32H5" else "DMA"
    required_ips = {"ADC1", dma_ip, "GPIO", "I2C1", "NVIC", "RCC", "SYS", "USART2"}
    if not regular_dma:
        required_ips.add("ADC2")
    if trigger_bridge:
        required_ips.add(parameters["adc_trigger_timer_instance"])
    # GPIO is represented by pin settings rather than an Mcu.IP entry in CubeMX.
    required_ips.remove("GPIO")
    report.require(required_ips <= ips, f"missing IOC peripherals: {sorted(required_ips - ips)}")

    pwm_selection = selected_option(requirement, "GMP_NUCLEO_PWM_TIMER_SELECTION")
    qep_selection = selected_option(requirement, "GMP_NUCLEO_QEP_TIMER_SELECTION")
    report.require(pwm_selection in {"1", "8"}, "PWM timer selection must be 1 or 8")
    report.require(qep_selection in {"3", "4"}, "QEP timer selection must be 3 or 4")
    check_pwm(report, ioc, f"TIM{pwm_selection}")
    # Validate every timer that the board entity advertises as switchable. Some
    # LQFP64 families cannot route both complete advanced timers concurrently.
    for timer_option in entity.get("option_sets", {}).get(
        "PWM_TIMER_SELECTION", [pwm_selection]
    ):
        check_pwm(report, ioc, f"TIM{timer_option}")

    for channel in (1, 2):
        report.require(
            any(
                value.startswith(f"TIM{qep_selection}_CH{channel},Encoder_Interface")
                for key, value in ioc.items()
                if key.startswith(f"SH.S_TIM{qep_selection}_CH{channel}.")
            ),
            f"TIM{qep_selection} CH{channel} is not an encoder input",
        )
    report.require(
        ioc.get(f"TIM{qep_selection}.EncoderMode") == "TIM_ENCODERMODE_TI12",
        f"TIM{qep_selection} does not count both encoder inputs (TI12)",
    )
    if int(parameters.get("qep_software_index", "0")):
        qep_z_port = parameters[f"qep_tim{qep_selection}_z_port"]
        qep_z_mask = parameters[f"qep_tim{qep_selection}_z_pin"]
        qep_z_number = qep_z_mask.removeprefix("GPIO_PIN_")
        qep_z_pin = f"P{qep_z_port[-1]}{qep_z_number}"
        report.require(
            pin_signal(ioc, qep_z_pin) == f"GPIO_EXTI{qep_z_number}",
            f"software QEP index is not routed to {qep_z_pin} EXTI",
        )
    else:
        report.require(
            any(
                value.startswith(f"TIM{qep_selection}_ETR,Encoder_Interface_w_index")
                for key, value in ioc.items()
                if key.startswith(f"SH.S_TIM{qep_selection}_ETR.")
            ),
            f"TIM{qep_selection} has no native index input",
        )

    check_adc_feedback(report, ioc, entity, pin_doc)
    expected_trigger = parameters[f"pwm_tim{pwm_selection}_adc_trigger"]
    supported_triggers = {
        parameters["pwm_tim1_adc_trigger"],
        parameters["pwm_tim8_adc_trigger"],
    }
    runtime_binding = (
        repo / "csp/stm32/Nucleo_32/src/xplt/xplt.peripheral.c"
    ).read_text(encoding="utf-8")
    if regular_dma:
        report.require(
            ioc.get("ADC1.ExternalTrigConv") == expected_trigger,
            "ADC1 regular scan is not triggered by the selected PWM timer",
        )
        report.require(
            ioc.get("ADC1.DMAContinuousRequests") == "ENABLE"
            and ioc.get("Dma.ADC1.0.Mode") == "DMA_CIRCULAR",
            "ADC1 regular scan DMA is not continuous and circular",
        )
        if trigger_bridge:
            bridge_timer = parameters["adc_trigger_timer_instance"]
            report.require(
                ioc.get(f"{bridge_timer}.TIM_SlaveMode") == "TIM_SLAVEMODE_RESET"
                and ioc.get(f"{bridge_timer}.TIM_MasterOutputTrigger")
                == "TIM_TRGO_UPDATE",
                f"{bridge_timer} does not relay PWM TRGO to the ADC",
            )
            report.require(
                any(
                    key.startswith(f"VP_{bridge_timer}_VS_ClockSourceITR")
                    and value == "TriggerSource_ITR0"
                    for key, value in ioc.items()
                ),
                f"{bridge_timer} trigger bridge is not sourced from TIM1 ITR0",
            )
    else:
        for adc in ("ADC1", "ADC2"):
            configured_trigger = ioc.get(f"{adc}.ExternalTrigInjecConv")
            report.require(
                configured_trigger in supported_triggers,
                f"{adc} injected trigger is not supplied by TIM1 or TIM8",
            )
            if configured_trigger != expected_trigger:
                report.require(
                    "xplt_select_adc_trigger" in runtime_binding
                    and "GMP_NUCLEO_PWM_ADC_TRIGGER" in runtime_binding,
                    f"{adc} needs runtime trigger rebinding for TIM{pwm_selection}",
                )
                report.warnings.append(
                    f"{adc} IOC defaults to {configured_trigger}; shared xplt rebinds it to {expected_trigger}"
                )
    if ioc.get("Mcu.Family") == "STM32H5":
        report.require(
            any(key.startswith("NVIC.ADC1_IRQn") for key in ioc),
            "ADC1 interrupt is not enabled",
        )
        report.require(
            ioc.get("GPDMA1.REQUEST_GPDMACH1") == "GPDMA1_REQUEST_USART2_RX"
            and ioc.get("GPDMA1.CIRCULARMODE_GPDMACH1") == "ENABLE"
            and ioc.get("GPDMA1.DESTINC_GPDMACH1") == "DMA_DINC_INCREMENTED",
            "USART2 RX GPDMA is not circular",
        )
        report.require(
            ioc.get("GPDMA1.REQUEST_GPDMACH0") == "GPDMA1_REQUEST_USART2_TX"
            and ioc.get("GPDMA1.DIRECTION_GPDMACH0") == "DMA_MEMORY_TO_PERIPH"
            and ioc.get("GPDMA1.SRCINC_GPDMACH0") == "DMA_SINC_INCREMENTED",
            "USART2 TX GPDMA is missing or invalid",
        )
    else:
        if not regular_dma:
            report.require("ADC1_2_IRQn" in " ".join(ioc), "ADC1/2 interrupt is not enabled")
        rx_prefix = next(
            (key.removesuffix(".Mode") for key in ioc if re.fullmatch(r"Dma\.USART2_RX\.\d+\.Mode", key)),
            "",
        )
        tx_prefix = next(
            (key.removesuffix(".Mode") for key in ioc if re.fullmatch(r"Dma\.USART2_TX\.\d+\.Mode", key)),
            "",
        )
        report.require(
            bool(rx_prefix) and ioc.get(rx_prefix + ".Mode") == "DMA_CIRCULAR",
            "USART2 RX DMA is not circular",
        )
        report.require(
            bool(tx_prefix)
            and ioc.get(tx_prefix + ".Direction") == "DMA_MEMORY_TO_PERIPH"
            and ioc.get(tx_prefix + ".Mode") == "DMA_NORMAL",
            "USART2 TX DMA is missing or invalid",
        )
    report.require(pin_signal(ioc, "PA2") == "USART2_TX", "VCP TX is not PA2/USART2_TX")
    report.require(pin_signal(ioc, "PA3") == "USART2_RX", "VCP RX is not PA3/USART2_RX")
    i2c_scl_pin = parameters["i2c_scl_pin"]
    i2c_sda_pin = parameters["i2c_sda_pin"]
    report.require(
        pin_signal(ioc, i2c_scl_pin) == "I2C1_SCL",
        f"I2C SCL is not {i2c_scl_pin}",
    )
    report.require(
        pin_signal(ioc, i2c_sda_pin) == "I2C1_SDA",
        f"I2C SDA is not {i2c_sda_pin}",
    )
    report.require(pin_signal(ioc, "PB8") == "GPIO_Output", "status LED is not on PB8")

    report.require(
        ioc.get("RCC.PLLSourceVirtual") == "RCC_PLLSOURCE_HSI",
        "the control topology must use HSI so PF0 is free for TIM1_CH3N",
    )
    pwm_pins = {
        "PA8": "S_TIM1_CH1",
        "PA9": "S_TIM1_CH2",
        "PA10": "S_TIM1_CH3",
        "PA11": "TIM1_CH1N",
        "PA12": "TIM1_CH2N",
        "PF0": "TIM1_CH3N",
    }
    for pin, signal in pwm_pins.items():
        report.require(
            pin_signal(ioc, pin) == signal,
            f"three-phase PWM pin {pin} is not routed to {signal}",
        )

    if int(parameters["has_dac"]):
        report.require("DAC1" in ips and pin_signal(ioc, "PA4") == "COMP_DAC11_group", "DAC capability mismatch")
    if int(parameters["has_can"]):
        can_rx_pin = parameters["can_rx_pin"]
        can_tx_pin = parameters["can_tx_pin"]
        report.require(
            "FDCAN1" in ips
            and pin_signal(ioc, can_rx_pin) == "FDCAN1_RX"
            and pin_signal(ioc, can_tx_pin) == "FDCAN1_TX",
            "FDCAN capability mismatch",
        )
        if int(parameters.get("can_has_stby", "0")):
            can_stby_port = parameters["can_stby_port"]
            can_stby_number = parameters["can_stby_pin"].removeprefix("GPIO_PIN_")
            can_stby_pin = f"P{can_stby_port[-1]}{can_stby_number}"
            report.require(
                pin_signal(ioc, can_stby_pin) == "GPIO_Output",
                "CAN transceiver standby pin is not a GPIO output",
            )

    documented_pins = {
        "PA2", "PA3", "PB8", i2c_scl_pin, i2c_sda_pin,
    }
    if int(parameters["has_dac"]):
        documented_pins.add(parameters["dac_pin"])
    if int(parameters["has_can"]):
        documented_pins.update((parameters["can_rx_pin"], parameters["can_tx_pin"]))
    for pin in sorted(documented_pins):
        report.require(pin in pin_doc, f"{pin} is absent from pin_assign.md")

    serialized = json.dumps(requirement) + json.dumps(entity) + "\n".join(ioc.values())
    report.require(
        re.search(r"[A-Za-z]:[/\\]", serialized) is None,
        "configuration contains an absolute Windows path",
    )
    if ioc.get("board") == "custom":
        report.warnings.append(
            "CubeMX board is 'custom'; ST-Link VCP and connector routing rely on the reviewed pin table"
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("targets", nargs="*", type=Path, help="board directories to validate")
    args = parser.parse_args()
    script_dir = Path(__file__).resolve().parent
    repo = find_repo_root(script_dir)
    targets = args.targets or sorted(
        path.parent.parent
        for path in script_dir.parent.glob("*/sdpe_mgr/sdpe_requirement.json")
    )
    if not targets:
        print("no Nucleo-32 board targets found", file=sys.stderr)
        return 2

    failed = False
    for target in targets:
        board_dir = target if target.is_absolute() else (Path.cwd() / target)
        report = check_target(repo, board_dir.resolve())
        state = "FAIL" if report.errors else "PASS"
        print(f"[{state}] {report.target.relative_to(repo)}")
        for warning in report.warnings:
            print(f"  warning: {warning}")
        for error in report.errors:
            print(f"  error: {error}")
        failed = failed or bool(report.errors)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
