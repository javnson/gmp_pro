"""Sustained Scope and reconnect stress test for the F28388D GMP DL example."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(
    0, str(REPOSITORY_ROOT / "tools" / "gmp_datalink" / "datalink_studio")
)

from apis import (
    GmpDatalinkClient,
    ScopeConfiguration,
    ScopeTriggerMode,
    TcpDataLinkTransport,
    UdpDataLinkTransport,
)


def make_client(args: argparse.Namespace) -> GmpDatalinkClient:
    """Create a fresh client so reconnect tests also exercise target cleanup."""
    if args.protocol == "serial":
        return GmpDatalinkClient(
            args.port,
            args.baudrate,
            timeout=args.timeout,
            retries=args.retries,
        )
    transport_type = (
        TcpDataLinkTransport if args.protocol == "tcp" else UdpDataLinkTransport
    )
    return GmpDatalinkClient(
        transport=transport_type(
            args.host,
            args.network_port,
            timeout=args.timeout,
            retries=args.retries,
        )
    )


def verify_resources(client: GmpDatalinkClient) -> tuple[object, object | None]:
    """Exercise all example facilities and return the first Scope/Memory item."""
    parameters = client.tunables.discover()
    if len(parameters) != 3:
        raise RuntimeError(f"Expected three tunables; target reported {len(parameters)}.")
    values = client.tunables.read_all()
    if len(values) != len(parameters):
        raise RuntimeError("Tunable readback count does not match discovery.")
    regions = client.memory.discover()
    if regions:
        client.memory.read_region(regions[0], byte_length=min(16, regions[0].byte_length))
    scopes = client.scope.discover()
    if len(scopes) != 1:
        raise RuntimeError(f"Expected one Scope; target reported {len(scopes)}.")
    return scopes[0], regions[0] if regions else None


def stress_captures(client: GmpDatalinkClient, count: int, report_every: int) -> None:
    """Continuously re-arm/read Scope while interleaving other DL facilities."""
    if count == 0:
        return
    scope, memory = verify_resources(client)
    configuration = ScopeConfiguration(
        mode=ScopeTriggerMode.IMMEDIATE,
        trigger_position_percent=50.0,
        sample_divider=0,
    )
    started = time.monotonic()
    previous_generation: int | None = None
    for completed, frame in enumerate(
        client.scope.iter_captures(
            scope,
            configuration,
            count=count,
            timeout=5.0,
            poll_interval=0.01,
        ),
        start=1,
    ):
        if previous_generation is not None:
            expected = (previous_generation + 1) & 0xFFFFFFFF
            if frame.generation != expected:
                raise RuntimeError(
                    f"Scope generation jumped from {previous_generation} "
                    f"to {frame.generation} at capture {completed}."
                )
        previous_generation = frame.generation
        if len(frame.channels) != 2 or any(
            len(channel) != frame.resource.depth for channel in frame.channels
        ):
            raise RuntimeError(f"Malformed Scope frame at capture {completed}.")
        if completed % 10 == 0:
            client.tunables.read_all()
            if memory is not None:
                client.memory.read_region(
                    memory, byte_length=min(16, memory.byte_length)
                )
        if completed % report_every == 0 or completed == count:
            elapsed = time.monotonic() - started
            print(
                f"Scope stress: {completed}/{count}, generation "
                f"{frame.generation}, {elapsed:.1f} s",
                flush=True,
            )


def stress_reconnects(args: argparse.Namespace) -> None:
    """Repeatedly create/close clients and verify the target accepts the next one."""
    for completed in range(1, args.reconnect_cycles + 1):
        with make_client(args) as client:
            verify_resources(client)
        # Let the 1 ms NO_SYS scheduler consume FIN before issuing the next
        # SYN.  This is still a rapid reconnect test without depending on the
        # desktop TCP stack's first-SYN retransmission interval.
        time.sleep(args.reconnect_delay)
        if completed % args.report_every == 0 or completed == args.reconnect_cycles:
            print(
                f"Reconnect stress: {completed}/{args.reconnect_cycles}", flush=True
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", choices=("serial", "tcp", "udp"), required=True)
    parser.add_argument("--port", help="Serial port, for example COM74")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--host", default="192.168.137.2")
    parser.add_argument("--network-port", type=int)
    parser.add_argument("--captures", type=int, default=200)
    parser.add_argument("--reconnect-cycles", type=int, default=0)
    parser.add_argument("--report-every", type=int, default=10)
    parser.add_argument("--reconnect-delay", type=float, default=0.02)
    parser.add_argument("--timeout", type=float, default=0.8)
    parser.add_argument("--retries", type=int, default=2)
    args = parser.parse_args()
    if (
        args.captures < 0
        or args.reconnect_cycles < 0
        or args.report_every <= 0
        or args.reconnect_delay < 0.0
    ):
        parser.error(
            "counts and --reconnect-delay must be non-negative; "
            "--report-every must be positive"
        )
    if args.protocol == "serial" and not args.port:
        parser.error("--port is required for serial")
    if args.network_port is None:
        args.network_port = 50001 if args.protocol == "tcp" else 50002
    if args.protocol != "tcp" and args.reconnect_cycles:
        parser.error("reconnect stress is defined only for TCP")

    if args.captures:
        with make_client(args) as client:
            stress_captures(client, args.captures, args.report_every)
        if args.reconnect_cycles:
            time.sleep(args.reconnect_delay)
    stress_reconnects(args)
    print(
        f"DL stress passed: {args.captures} captures, "
        f"{args.reconnect_cycles} reconnects.",
        flush=True,
    )


if __name__ == "__main__":
    main()
