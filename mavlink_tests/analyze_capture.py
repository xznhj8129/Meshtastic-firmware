#!/usr/bin/env python3
"""Break a capture down message by message: rates, gaps, queue state, drops.

Totals hide what a constrained link actually does. This prints a per-second timeline beside
per-message-type rates and the node-side queue and relay counters, so it is visible whether a
link settles, oscillates, or stays locked up.

Usage:

    python3 mavlink_tests/analyze_capture.py mavlink_tests/reports/px4-sitl-mesh-capture-<stamp>
"""

from __future__ import annotations

import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

STRIP_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def load_events(report: Path) -> list[dict]:
    path = report / "events.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def node_counters(path: Path) -> list[tuple[float, int, int, int, int]]:
    """Extract (elapsed, txGood, txRelay, rxGood, rxBad) samples from a node console log."""
    out = []
    if not path.exists():
        return out
    pattern = re.compile(r"txGood=(\d+),txRelay=(\d+),rxGood=(\d+),rxBad=(\d+)")
    for line in path.read_text(errors="replace").splitlines():
        stamp = re.match(r"\[[^]]*\] \[ *([0-9.]+)\]", line)
        found = pattern.search(STRIP_ANSI.sub("", line))
        if stamp and found:
            out.append((float(stamp.group(1)), *(int(g) for g in found.groups())))
    return out


def count_lines(path: Path, needle: str) -> int:
    if not path.exists():
        return 0
    return sum(1 for line in path.read_text(errors="replace").splitlines() if needle in line)


def main() -> int:
    report = Path(sys.argv[1])
    events = load_events(report)
    summary = json.loads((report / "capture.json").read_text())

    rx = [e for e in events if e["event"] == "mavlink_rx_message"]
    tx = [e for e in events if e["event"] == "mavlink_tx_message"]
    probes = [e for e in events if e["event"] == "probe_send"]
    duration = max((e["elapsed_s"] for e in events), default=0.0)

    print(f"# {report.name}")
    cfg = summary["configuration"]
    print(f"window {duration:.1f}s   probes {len(probes)}   rx frames {len(rx)}   tx frames {len(tx)}")
    print(f"px4 max_rate_bps={cfg.get('max_rate_bps')} probe_interval={cfg.get('probe_interval_s')}s\n")

    print("## received frames by type")
    print(f"{'type':<22}{'count':>6}{'per sec':>9}{'mean gap':>10}{'max gap':>9}")
    by_type: dict[str, list[float]] = defaultdict(list)
    for e in rx:
        by_type[e["message_type"]].append(e["elapsed_s"])
    for name, times in sorted(by_type.items(), key=lambda kv: -len(kv[1])):
        gaps = [b - a for a, b in zip(times, times[1:])]
        mean = sum(gaps) / len(gaps) if gaps else 0.0
        widest = max(gaps) if gaps else 0.0
        print(f"{name:<22}{len(times):>6}{len(times)/duration:>9.2f}{mean:>10.2f}{widest:>9.2f}")

    print("\n## per-second timeline  (rx frames | tx frames | types)")
    buckets: dict[int, Counter] = defaultdict(Counter)
    tx_buckets: Counter = Counter()
    for e in rx:
        buckets[int(e["elapsed_s"])][e["message_type"]] += 1
    for e in tx:
        tx_buckets[int(e["elapsed_s"])] += 1
    for second in range(int(duration) + 1):
        got = buckets.get(second, Counter())
        sent = tx_buckets.get(second, 0)
        if not got and not sent:
            continue
        detail = " ".join(f"{k}x{v}" if v > 1 else k for k, v in got.most_common())
        print(f"  t={second:>3}s  rx={sum(got.values()):>2}  tx={sent:>2}  {detail}")

    for label, filename in (("air", "air-node.log"), ("ground", "ground-node.log")):
        path = report / filename
        samples = node_counters(path)
        if not samples:
            continue
        first, last = samples[0], samples[-1]
        span = last[0] - first[0] or 1.0
        print(f"\n## {label} node over {span:.0f}s of console")
        print(f"  txGood  +{last[1]-first[1]:<5} ({(last[1]-first[1])/span:.2f}/s)")
        print(f"  txRelay +{last[2]-first[2]:<5} ({(last[2]-first[2])/span:.2f}/s)   "
              f"relays are rebroadcasts of the other node, pure waste with two nodes")
        print(f"  rxGood  +{last[3]-first[3]:<5}   rxBad +{last[4]-first[4]}")
        print(f"  TX queue full : {count_lines(path, 'TX queue is full')}")
        print(f"  ToPhone full  : {count_lines(path, 'ToPhone queue is full')}  (no phone attached; ignore)")
        print(f"  busyRx defer  : {count_lines(path, 'Can not send yet, busyRx')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
