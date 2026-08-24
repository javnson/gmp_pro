#!/usr/bin/env python3
"""Host-side coefficient, injection, and frequency-response utilities.

The tool deliberately separates controller math from the physical transport.
It emits an AXI register image and the exact 16-bit SPI frames used by the TD
bridge, while AMD/Xilinx software can write the same byte addresses directly.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from pathlib import Path
from typing import Iterable, Sequence

DATA_WIDTH = 32
FRAC_WIDTH = 28
MAX_SECTIONS = 4
COEFF_BASE = 0x040
COEFF_STRIDE = 0x14
COEFF_NAMES = ("b0", "b1", "b2", "a1", "a2")


def quantize(value: float, width: int = DATA_WIDTH, frac: int = FRAC_WIDTH) -> int:
    """Round and saturate a floating value to a signed fixed-point word."""
    scaled = int(round(value * (1 << frac)))
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    return min(max(scaled, lo), hi)


def unsigned_word(value: int, width: int = DATA_WIDTH) -> int:
    return value & ((1 << width) - 1)


def normalize_sos(rows: Iterable[Sequence[float]]) -> list[list[float]]:
    normalized: list[list[float]] = []
    for index, row in enumerate(rows):
        if len(row) == 6:
            b0, b1, b2, a0, a1, a2 = map(float, row)
            if a0 == 0.0:
                raise ValueError(f"SOS row {index} has a0=0")
            normalized.append([b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0])
        elif len(row) == 5:
            normalized.append(list(map(float, row)))
        else:
            raise ValueError(f"SOS row {index} must contain 5 normalized or 6 scipy-format values")
    if len(normalized) > MAX_SECTIONS:
        raise ValueError(f"hardware supports at most {MAX_SECTIONS} SOS sections")
    return normalized


def transfer_function_to_sos(numerator: Sequence[float], denominator: Sequence[float]) -> list[list[float]]:
    try:
        from scipy.signal import tf2sos  # type: ignore
    except ImportError as exc:
        raise RuntimeError("transfer-function conversion requires scipy; use --sos-json otherwise") from exc
    return normalize_sos(tf2sos(numerator, denominator).tolist())


def register_writes(sos: Sequence[Sequence[float]]) -> list[dict[str, object]]:
    rows = normalize_sos(sos)
    writes: list[dict[str, object]] = []
    for section, coefficients in enumerate(rows):
        for coefficient, value in enumerate(coefficients):
            address = COEFF_BASE + section * COEFF_STRIDE + coefficient * 4
            minimum = -(1 << (DATA_WIDTH - 1)) / (1 << FRAC_WIDTH)
            maximum = ((1 << (DATA_WIDTH - 1)) - 1) / (1 << FRAC_WIDTH)
            if not minimum <= value <= maximum:
                raise ValueError(
                    f"s{section}.{COEFF_NAMES[coefficient]}={value} is outside "
                    f"Q{DATA_WIDTH-FRAC_WIDTH-1}.{FRAC_WIDTH} range [{minimum}, {maximum}]"
                )
            signed = quantize(value)
            writes.append({
                "address": address,
                "name": f"s{section}.{COEFF_NAMES[coefficient]}",
                "float": value,
                "value": unsigned_word(signed),
            })
    writes.extend([
        {"address": 0x020, "name": "active_sections", "value": len(rows)},
        {"address": 0x024, "name": "commit", "value": 1},
    ])
    return writes


def spi_frames_for_write(byte_address: int, value: int) -> list[list[int]]:
    if byte_address & 3:
        raise ValueError("AXI register addresses must be 32-bit aligned")
    halfword = byte_address // 2
    if halfword + 1 > 0x7F:
        raise ValueError("address is outside the TD SPI bridge window")
    return [
        [((halfword & 0x7F) << 8), value & 0xFFFF],
        [(((halfword + 1) & 0x7F) << 8), (value >> 16) & 0xFFFF],
    ]


def write_register_image(path: Path, sos: Sequence[Sequence[float]]) -> None:
    writes = register_writes(sos)
    for item in writes:
        item["spi_transactions"] = spi_frames_for_write(
            int(item["address"]), int(item["value"])
        )
    payload = {
        "format": {"width": DATA_WIDTH, "fractional_bits": FRAC_WIDTH},
        "difference_equation": "y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2",
        "writes": writes,
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def make_log_chirp(sample_rate: float, duration: float, f_start: float, f_stop: float,
                   amplitude: float) -> list[float]:
    if not (0 < f_start < f_stop < sample_rate / 2):
        raise ValueError("require 0 < f_start < f_stop < sample_rate/2")
    count = int(round(sample_rate * duration))
    rate = math.log(f_stop / f_start) / duration
    return [
        amplitude * math.sin(2 * math.pi * f_start * (math.exp(rate * n / sample_rate) - 1) / rate)
        for n in range(count)
    ]


def write_injection(csv_path: Path, binary_path: Path | None, samples: Sequence[float], sample_rate: float) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "injection_float", "injection_q3_28"))
        for index, value in enumerate(samples):
            writer.writerow((index / sample_rate, value, quantize(value)))
    if binary_path:
        with binary_path.open("wb") as stream:
            for value in samples:
                stream.write(struct.pack("<I", unsigned_word(quantize(value))))


def analyze_response(input_path: Path, output_path: Path, sample_rate: float) -> None:
    try:
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError("frequency-response analysis requires numpy") from exc
    injected, measured = [], []
    with input_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {"input", "output"}.issubset(reader.fieldnames):
            raise ValueError("measurement CSV requires input and output columns")
        for row in reader:
            injected.append(float(row["input"]))
            measured.append(float(row["output"]))
    if len(injected) < 8 or len(injected) != len(measured):
        raise ValueError("measurement requires equal input/output arrays of at least 8 samples")
    window = np.hanning(len(injected))
    x = np.fft.rfft(np.asarray(injected) * window)
    y = np.fft.rfft(np.asarray(measured) * window)
    frequency = np.fft.rfftfreq(len(injected), 1.0 / sample_rate)
    valid = np.abs(x) > max(float(np.max(np.abs(x))) * 1e-9, 1e-15)
    transfer = np.zeros_like(y, dtype=complex)
    transfer[valid] = y[valid] / x[valid]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("frequency_hz", "magnitude_db", "phase_deg"))
        for f, h, keep in zip(frequency, transfer, valid):
            if keep:
                writer.writerow((float(f), 20 * math.log10(max(abs(h), 1e-30)),
                                 math.degrees(math.atan2(h.imag, h.real))))


def parse_floats(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    compile_cmd = sub.add_parser("compile", help="compile TF or SOS into register writes")
    source = compile_cmd.add_mutually_exclusive_group(required=True)
    source.add_argument("--sos-json", type=Path)
    source.add_argument("--numerator", help="comma-separated transfer-function numerator")
    compile_cmd.add_argument("--denominator", help="comma-separated denominator (required with numerator)")
    compile_cmd.add_argument("--output", type=Path, required=True)

    inject_cmd = sub.add_parser("make-injection", help="create logarithmic chirp samples")
    inject_cmd.add_argument("--sample-rate", type=float, required=True)
    inject_cmd.add_argument("--duration", type=float, required=True)
    inject_cmd.add_argument("--f-start", type=float, required=True)
    inject_cmd.add_argument("--f-stop", type=float, required=True)
    inject_cmd.add_argument("--amplitude", type=float, default=0.1)
    inject_cmd.add_argument("--csv", type=Path, required=True)
    inject_cmd.add_argument("--binary", type=Path)

    analyze_cmd = sub.add_parser("analyze", help="estimate frequency response from a CSV")
    analyze_cmd.add_argument("--input", type=Path, required=True)
    analyze_cmd.add_argument("--sample-rate", type=float, required=True)
    analyze_cmd.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.command == "compile":
        if args.sos_json:
            raw = json.loads(args.sos_json.read_text(encoding="utf-8"))
            rows = raw["sos"] if isinstance(raw, dict) else raw
            sos = normalize_sos(rows)
        else:
            if not args.denominator:
                parser.error("--denominator is required with --numerator")
            sos = transfer_function_to_sos(parse_floats(args.numerator), parse_floats(args.denominator))
        write_register_image(args.output, sos)
    elif args.command == "make-injection":
        samples = make_log_chirp(args.sample_rate, args.duration, args.f_start,
                                 args.f_stop, args.amplitude)
        write_injection(args.csv, args.binary, samples, args.sample_rate)
    else:
        analyze_response(args.input, args.output, args.sample_rate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
