#!/usr/bin/env python3
"""Validate Nucleo-144 control and mandatory Ethernet board contracts."""

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


def repo_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "ctl/hardware_preset/sdpe_schemas").is_dir():
            return candidate
    raise RuntimeError("cannot locate GMP Pro repository root")


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_ioc(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{number}: invalid IOC setting")
        key, value = line.split("=", 1)
        result[key] = value
    return result


def ips(ioc: dict[str, str]) -> set[str]:
    return {
        ioc.get(f"Mcu.IP{index}", "")
        for index in range(int(ioc.get("Mcu.IPNb", "0")))
    }


def pin_signal(ioc: dict[str, str], pin: str) -> str:
    direct = ""
    for key, value in ioc.items():
        if not key.endswith(".Signal"):
            continue
        key_pin = key.removesuffix(".Signal").split("-", 1)[0]
        key_pin = key_pin.replace("\\ ", " ").split("(", 1)[0].strip()
        if key_pin == pin:
            direct = value
            break
    if direct.startswith("ADCx_"):
        return next(
            (
                value.split(",", 1)[0]
                for key, value in ioc.items()
                if key.startswith(f"SH.{direct}.") and value.startswith("ADC")
            ),
            direct,
        )
    if direct.startswith("GPXTI"):
        return next(
            (
                value.split(",", 1)[0]
                for key, value in ioc.items()
                if key.startswith(f"SH.{direct}.") and value.startswith("GPIO_EXTI")
            ),
            direct,
        )
    return direct


def option(requirement: dict, macro: str) -> str:
    return next(
        str(item["value"])
        for item in requirement.get("option_macros", [])
        if item.get("macro") == macro
    )


def parse_ipv4(value: str) -> tuple[int, int, int, int] | None:
    try:
        octets = tuple(int(item) for item in value.split("."))
    except ValueError:
        return None
    return octets if len(octets) == 4 else None


def check_target(root: Path, board: Path) -> Report:
    report = Report(board)
    ioc_files = sorted(board.glob("*.ioc"))
    requirement_file = board / "sdpe_mgr/sdpe_requirement.json"
    pin_file = board / "pin_assign.md"
    report.require(len(ioc_files) == 1, "target must contain exactly one IOC file")
    report.require(requirement_file.is_file(), "missing SDPE requirement")
    report.require(pin_file.is_file(), "missing pin_assign.md")
    if len(ioc_files) != 1 or not requirement_file.is_file():
        return report

    ioc_file = ioc_files[0]
    ioc = read_ioc(ioc_file)
    requirement = read_json(requirement_file)
    hardware = requirement.get("hardware", [])
    report.require(len(hardware) == 1, "target must select exactly one board entity")
    if len(hardware) != 1:
        return report
    entity_file = root / "ctl/hardware_preset/sdpe_src/mcu_board" / (
        hardware[0]["entity"] + ".json"
    )
    report.require(entity_file.is_file(), "selected board entity does not exist")
    if not entity_file.is_file():
        return report
    entity = read_json(entity_file)
    p = entity["parameters"]
    pin_doc = pin_file.read_text(encoding="utf-8")

    report.require(entity.get("schema") == "stm32_nucleo_144_board", "wrong schema")
    report.require(ioc.get("Mcu.UserName") == p["chip"], "IOC MCU/entity mismatch")
    report.require(ioc.get("Mcu.Package") == "LQFP144", "MCU is not LQFP144")
    report.require(
        ioc_file.resolve() == (root / p["ioc_file"]).resolve(),
        "entity IOC path points elsewhere",
    )
    report.require(
        ioc.get("ProjectManager.LastFirmware") == "false",
        "CubeMX firmware version must be pinned",
    )

    pwm = f"TIM{option(requirement, 'GMP_NUCLEO_PWM_TIMER_SELECTION')}"
    qep = f"TIM{option(requirement, 'GMP_NUCLEO_QEP_TIMER_SELECTION')}"
    mandatory = {"ADC1", "DMA", "ETH", "I2C1", "LWIP", pwm, qep, "USART3"}
    report.require(mandatory <= ips(ioc), f"missing IOC IPs: {sorted(mandatory - ips(ioc))}")

    for channel in range(1, 4):
        report.require(
            ioc.get(
                f"{pwm}.Channel-PWM\\ Generation{channel}\\ CH{channel}\\ CH{channel}N"
            )
            == f"TIM_CHANNEL_{channel}",
            f"{pwm} complementary PWM channel {channel} is missing",
        )
    report.require(
        ioc.get(f"{pwm}.CounterMode", "").startswith("TIM_COUNTERMODE_CENTERALIGNED"),
        f"{pwm} is not center aligned",
    )
    report.require(ioc.get(f"{pwm}.DeadTime") is not None, f"{pwm} has no dead time")
    report.require(
        ioc.get(f"{pwm}.TIM_MasterOutputTrigger2") == "TIM_TRGO2_OC4REF"
        or ioc.get(f"{pwm}.TIM_MasterOutputTrigger") == "TIM_TRGO_OC4REF",
        f"{pwm} OC4 does not trigger ADC sampling",
    )
    report.require(
        ioc.get(f"{qep}.EncoderMode") == "TIM_ENCODERMODE_TI12",
        f"{qep} is not in TI12 encoder mode",
    )
    for channel in (1, 2):
        report.require(
            any(
                value.startswith(f"{qep}_CH{channel},Encoder_Interface")
                for key, value in ioc.items()
                if key.startswith(f"SH.S_{qep}_CH{channel}.")
            ),
            f"{qep} CH{channel} encoder input is missing",
        )

    report.require(int(p["adc_fb_count"]) >= 6, "fewer than six ADC feedback pins")
    for index in range(int(p["adc_fb_count"])):
        pin = p[f"adc_fb{index}_pin"]
        signal = pin_signal(ioc, pin)
        match = re.fullmatch(r"ADC1_INP?(\d+)", signal)
        report.require(match is not None, f"FB{index} {pin} is not routed to ADC1")
        if match:
            channel_key = f"ADC1.Channel-{index}\\#ChannelRegularConversion"
            rank_key = f"ADC1.Rank-{index}\\#ChannelRegularConversion"
            report.require(
                ioc.get(channel_key) == f"ADC_CHANNEL_{match.group(1)}",
                f"FB{index} channel ordering does not match {pin}",
            )
            report.require(
                ioc.get(rank_key) == str(index + 1),
                f"FB{index} rank ordering is invalid",
            )
        report.require(pin in pin_doc, f"{pin} is absent from pin_assign.md")
    report.require(
        ioc.get("ADC1.ExternalTrigConv") == p["pwm_tim1_adc_trigger"],
        "ADC1 is not triggered from TIM1 TRGO2",
    )
    report.require(
        ioc.get("ADC1.ConversionDataManagement") == "ADC_CONVERSIONDATA_DMA_CIRCULAR"
        and ioc.get("Dma.ADC1.0.Mode") == "DMA_CIRCULAR",
        "ADC1 circular DMA is not configured",
    )

    uart = p["dl_uart_instance"]
    report.require(
        pin_signal(ioc, p["dl_tx_pin"]) == f"{uart}_TX",
        "ST-Link VCP TX routing mismatch",
    )
    report.require(
        pin_signal(ioc, p["dl_rx_pin"]) == f"{uart}_RX",
        "ST-Link VCP RX routing mismatch",
    )
    rx_dma = next(
        (prefix.removesuffix(".Mode") for prefix in ioc if re.fullmatch(rf"Dma\.{uart}_RX\.\d+\.Mode", prefix)),
        "",
    )
    tx_dma = next(
        (prefix.removesuffix(".Mode") for prefix in ioc if re.fullmatch(rf"Dma\.{uart}_TX\.\d+\.Mode", prefix)),
        "",
    )
    report.require(bool(rx_dma) and ioc.get(rx_dma + ".Mode") == "DMA_CIRCULAR", "VCP RX DMA is not circular")
    report.require(
        bool(tx_dma)
        and ioc.get(tx_dma + ".Direction") == "DMA_MEMORY_TO_PERIPH"
        and ioc.get(tx_dma + ".Mode") == "DMA_NORMAL",
        "VCP TX DMA is invalid",
    )
    report.require(pin_signal(ioc, p["i2c_scl_pin"]) == "I2C1_SCL", "I2C SCL mismatch")
    report.require(pin_signal(ioc, p["i2c_sda_pin"]) == "I2C1_SDA", "I2C SDA mismatch")
    led_pin = f"P{p['status_led_port'][-1]}{p['status_led_pin'].removeprefix('GPIO_PIN_')}"
    report.require(pin_signal(ioc, led_pin) == "GPIO_Output", "status LED routing mismatch")

    eth_pins = {
        "eth_ref_clk_pin": "ETH_REF_CLK",
        "eth_mdio_pin": "ETH_MDIO",
        "eth_crs_dv_pin": "ETH_CRS_DV",
        "eth_mdc_pin": "ETH_MDC",
        "eth_rxd0_pin": "ETH_RXD0",
        "eth_rxd1_pin": "ETH_RXD1",
        "eth_tx_en_pin": "ETH_TX_EN",
        "eth_txd0_pin": "ETH_TXD0",
        "eth_txd1_pin": "ETH_TXD1",
    }
    for field_name, signal in eth_pins.items():
        pin = p[field_name]
        report.require(pin_signal(ioc, pin) == signal, f"{pin} is not {signal}")
        report.require(pin in pin_doc, f"{pin} is absent from pin_assign.md")
    report.require(ioc.get("ETH.MediaInterface") == p["eth_media_interface"], "Ethernet is not RMII")
    expected_mac = ":".join(p[f"eth_mac{index}"].upper() for index in range(6))
    report.require(ioc.get("ETH.MACAddr", "").replace("\\:", ":").upper() == expected_mac, "MAC mismatch")
    report.require(ioc.get("LWIP.LWIP_DHCP") == "0", "acceptance target must use static IPv4")
    expected_ip = tuple(int(p[f"eth_ipv4_{index}"]) for index in range(4))
    expected_mask = tuple(int(p[f"eth_netmask_{index}"]) for index in range(4))
    expected_gateway = tuple(int(p[f"eth_gateway_{index}"]) for index in range(4))
    report.require(parse_ipv4(ioc.get("LWIP.IP_ADDRESS", "")) == expected_ip, "IPv4 address mismatch")
    report.require(parse_ipv4(ioc.get("LWIP.NETMASK_ADDRESS", "")) == expected_mask, "IPv4 netmask mismatch")
    report.require(parse_ipv4(ioc.get("LWIP.GATEWAY_ADDRESS", "")) == expected_gateway, "IPv4 gateway mismatch")
    report.require(ioc.get("LWIP0.BSP.component") == p["eth_phy_driver"], "PHY driver mismatch")

    serialized = json.dumps(requirement) + json.dumps(entity) + "\n".join(ioc.values())
    report.require(re.search(r"[A-Za-z]:[/\\]", serialized) is None, "configuration contains an absolute path")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("targets", nargs="*", type=Path)
    args = parser.parse_args()
    root = repo_root(Path(__file__).resolve())
    targets = args.targets or sorted(
        item.parent.parent for item in (root / "csp/stm32/Nucleo_144").glob("*/sdpe_mgr/sdpe_requirement.json")
    )
    failed = False
    for target in targets:
        board = target.resolve()
        report = check_target(root, board)
        print(f"[{'FAIL' if report.errors else 'PASS'}] {board.relative_to(root)}")
        for warning in report.warnings:
            print(f"  warning: {warning}")
        for error in report.errors:
            print(f"  error: {error}")
        failed |= bool(report.errors)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
