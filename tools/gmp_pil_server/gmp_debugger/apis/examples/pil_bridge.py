"""Run the headless GMP Simulink-to-Data-Link PIL bridge."""

from __future__ import annotations

import argparse
from pathlib import Path

from apis import (
    GmpDatalinkClient,
    PilApi,
    PilBridge,
    PilConfiguration,
    PilTraceWriter,
)


def main() -> int:
    """Load SDPE, open the target serial port, and serve PIL steps."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdpe", required=True, type=Path, help="Target sdpe_requirement.json")
    parser.add_argument("--port", required=True, help="Target Data Link serial port")
    parser.add_argument("--trace", type=Path, help="Optional output CSV trace")
    parser.add_argument("--maximum-steps", type=int, help="Stop after this many successful steps")
    args = parser.parse_args()

    config = PilConfiguration.from_sdpe(args.sdpe)
    trace_stream = None
    try:
        if args.trace is not None:
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            trace_stream = args.trace.open("w", encoding="utf-8", newline="")
        with GmpDatalinkClient(
            args.port,
            config.serial_baudrate,
            timeout=config.mcu_timeout_s,
            retries=0,
        ) as client:
            trace = PilTraceWriter(trace_stream, config) if trace_stream is not None else None
            bridge = PilBridge(PilApi(client.transport, config), trace=trace)
            completed = bridge.run(maximum_steps=args.maximum_steps)
        print(f"Completed {completed} PIL steps at BUILD_LEVEL {config.build_level}.")
        return 0
    finally:
        if trace_stream is not None:
            trace_stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
