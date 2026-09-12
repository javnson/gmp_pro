"""Example headless GMP Data Link hardware-debugging session."""

from __future__ import annotations

import argparse
from pathlib import Path

from apis import (
    GmpDatalinkClient,
    ScopeConfiguration,
    ScopeTriggerMode,
    TcpDataLinkTransport,
    UdpDataLinkTransport,
)


def main() -> None:
    """Discover resources, inspect target data, and acquire one waveform."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--protocol", choices=("serial", "tcp", "udp"), default="serial"
    )
    parser.add_argument("--port", help="Serial port, for example COM5")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--host", default="192.168.137.2")
    parser.add_argument("--network-port", type=int)
    parser.add_argument("--timeout", type=float, default=0.8)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument(
        "--skip-capture", action="store_true", help="Only discover DL facilities"
    )
    parser.add_argument("--output", type=Path, default=Path("scope_frame.csv"))
    args = parser.parse_args()

    if args.protocol == "serial":
        if not args.port:
            parser.error("--port is required for the serial protocol")
        client = GmpDatalinkClient(
            args.port,
            args.baudrate,
            timeout=args.timeout,
            retries=args.retries,
        )
    else:
        default_port = 50001 if args.protocol == "tcp" else 50002
        transport_type = (
            TcpDataLinkTransport if args.protocol == "tcp" else UdpDataLinkTransport
        )
        transport = transport_type(
            args.host,
            args.network_port or default_port,
            timeout=args.timeout,
            retries=args.retries,
        )
        client = GmpDatalinkClient(transport=transport)

    with client:
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
        if not scopes or args.skip_capture:
            if scopes:
                print("Scope resources discovered; capture was skipped.")
            else:
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
