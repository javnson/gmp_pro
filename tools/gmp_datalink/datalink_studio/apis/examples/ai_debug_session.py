"""Example headless GMP Data Link hardware-debugging session."""

from __future__ import annotations

import argparse
from pathlib import Path

from apis import GmpDatalinkClient, ScopeConfiguration, ScopeTriggerMode


def main() -> None:
    """Discover resources, inspect target data, and acquire one waveform."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--output", type=Path, default=Path("scope_frame.csv"))
    args = parser.parse_args()

    with GmpDatalinkClient(args.port, args.baudrate) as client:
        parameters = client.tunables.discover()
        print("Tunable table:")
        for parameter in parameters:
            print(
                f"  [{parameter.item_id}] {parameter.name}: "
                f"{parameter.data_type.name}, {parameter.permission.name}"
            )
        print("Current values:", client.tunables.read_all())

        regions = client.memory.discover()
        print("Memory regions:")
        for region in regions:
            print(
                f"  [{region.region_id}] {region.name}: "
                f"0x{region.address:08X}, {region.byte_length} bytes"
            )
        if regions:
            print("First 16 bytes:", client.memory.read_region(regions[0], byte_length=16).hex(" "))

        scopes = client.scope.discover()
        if not scopes:
            print("The target did not report a Scope resource.")
            return
        frame = client.scope.capture(
            scopes[0],
            ScopeConfiguration(
                mode=ScopeTriggerMode.IMMEDIATE,
                trigger_position_percent=50.0,
                sample_divider=0,
            ),
        )
        frame.save_csv(args.output)
        print(
            f"Saved generation {frame.generation}, {len(frame.channels)} channels, "
            f"{len(frame.time_seconds)} samples/channel to {args.output}."
        )


if __name__ == "__main__":
    main()
