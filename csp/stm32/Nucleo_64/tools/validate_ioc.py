#!/usr/bin/env python3
"""Validate a Nucleo-64 IOC against its SDPE board entity and pin table."""

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
        ioc.get(prefix + "TIM_MasterOutputTrigger") == "TIM_TRGO_OC4REF",
        f"{timer} does not trigger from OC4REF",
    )


def check_adc_feedback(
    report: Report, ioc: dict[str, str], entity: dict, pin_doc: str
) -> None:
    parameters = entity["parameters"]
    count = int(parameters["adc_fb_count"])
    report.require(count >= 6, "board entity exposes fewer than six ADC feedback channels")
    seen_pins: set[str] = set()
    for index in range(count):
        pin = parameters[f"adc_fb{index}_pin"]
        handle = parameters[f"adc_fb{index}_handle"]
        rank_token = parameters[f"adc_fb{index}_rank"]
        adc = peripheral_from_handle(handle, "adc")
        match = re.fullmatch(r"ADC_INJECTED_RANK_([1-4])", rank_token)
        report.require(match is not None, f"FB{index} has invalid injected rank {rank_token}")
        if match is None:
            continue
        rank = match.group(1)
        signal = ioc.get(f"{pin}.Signal", "")
        signal_match = re.fullmatch(rf"{adc}_IN(\d+)", signal)
        report.require(signal_match is not None, f"FB{index} {pin} is not routed to {adc}")
        if signal_match:
            report.require(
                ioc.get(f"{adc}.Rank{rank}_Channel")
                == f"ADC_CHANNEL_{signal_match.group(1)}",
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
    pin_doc_path = board_dir / "pin_assign.md"
    report.require(pin_doc_path.is_file(), "missing pin_assign.md")
    pin_doc = pin_doc_path.read_text(encoding="utf-8") if pin_doc_path.is_file() else ""

    report.require(entity.get("schema") == "stm32_nucleo_64_board", "wrong SDPE schema")
    report.require(
        ioc.get("Mcu.UserName") == parameters["chip"],
        "IOC MCU does not match the SDPE board entity",
    )
    expected_ioc = (repo / parameters["ioc_file"]).resolve()
    report.require(ioc_path.resolve() == expected_ioc, "entity ioc_file points elsewhere")
    report.require(ioc.get("Mcu.Package") == "LQFP64", "target is not a 64-pin LQFP MCU")
    report.require(ioc.get("ProjectManager.LastFirmware") == "false", "LastFirmware must be false")

    ips = ioc_ips(ioc)
    required_ips = {"ADC1", "ADC2", "DMA", "GPIO", "I2C1", "NVIC", "RCC", "SYS", "USART2"}
    # GPIO is represented by pin settings rather than an Mcu.IP entry in CubeMX.
    required_ips.remove("GPIO")
    report.require(required_ips <= ips, f"missing IOC peripherals: {sorted(required_ips - ips)}")

    pwm_selection = selected_option(requirement, "GMP_NUCLEO_PWM_TIMER_SELECTION")
    qep_selection = selected_option(requirement, "GMP_NUCLEO_QEP_TIMER_SELECTION")
    report.require(pwm_selection in {"1", "8"}, "PWM timer selection must be 1 or 8")
    report.require(qep_selection in {"3", "4"}, "QEP timer selection must be 3 or 4")
    check_pwm(report, ioc, f"TIM{pwm_selection}")
    # Both alternatives must remain complete so changing SDPE needs no IOC edit.
    check_pwm(report, ioc, "TIM1")
    check_pwm(report, ioc, "TIM8")

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
        repo / "csp/stm32/Nucleo_64/src/xplt/xplt.peripheral.c"
    ).read_text(encoding="utf-8")
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
    report.require("ADC1_2_IRQn" in " ".join(ioc), "ADC1/2 interrupt is not enabled")

    report.require(
        ioc.get("Dma.USART2_RX.1.Mode") == "DMA_CIRCULAR",
        "USART2 RX DMA is not circular",
    )
    report.require(
        ioc.get("Dma.USART2_TX.2.Direction") == "DMA_MEMORY_TO_PERIPH"
        and ioc.get("Dma.USART2_TX.2.Mode") == "DMA_NORMAL",
        "USART2 TX DMA is missing or invalid",
    )
    report.require(ioc.get("PA2.Signal") == "USART2_TX", "VCP TX is not PA2/USART2_TX")
    report.require(ioc.get("PA3.Signal") == "USART2_RX", "VCP RX is not PA3/USART2_RX")
    report.require(ioc.get("PB8.Signal") == "I2C1_SCL", "I2C SCL is not PB8")
    report.require(ioc.get("PB9.Signal") == "I2C1_SDA", "I2C SDA is not PB9")
    report.require(ioc.get("PA5.Signal") == "GPIO_Output", "status LED is not on PA5")

    if int(parameters["has_dac"]):
        report.require("DAC1" in ips and ioc.get("PA4.Signal") == "COMP_DAC11_group", "DAC capability mismatch")
    if int(parameters["has_can"]):
        report.require(
            "FDCAN1" in ips
            and ioc.get("PA11.Signal") == "FDCAN1_RX"
            and ioc.get("PA12.Signal") == "FDCAN1_TX",
            "FDCAN capability mismatch",
        )

    for pin in ("PA2", "PA3", "PA4", "PA5", "PA11", "PA12", "PB8", "PB9"):
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
        print("no Nucleo-64 board targets found", file=sys.stderr)
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
