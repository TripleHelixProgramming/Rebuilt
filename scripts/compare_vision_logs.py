#!/usr/bin/env python3
"""
Compare vision processing between two WPILOG files (real vs simulation replay).

Analyzes:
- Vision inputs (poseObservations, tagIds, connected status)
- Vision outputs (ObservationScore, RobotPosesAccepted, RobotPosesRejected)
- Timing differences
- Score distributions
"""

import array
import mmap
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any

import msgpack

# --- WPILib DataLog Reader (from wpilibsuite/allwpilib) ---

floatStruct = struct.Struct("<f")
doubleStruct = struct.Struct("<d")

kControlStart = 0
kControlFinish = 1
kControlSetMetadata = 2


class StartRecordData:
    def __init__(self, entry: int, name: str, type_: str, metadata: str):
        self.entry = entry
        self.name = name
        self.type = type_
        self.metadata = metadata


class DataLogRecord:
    def __init__(self, entry: int, timestamp: int, data):
        self.entry = entry
        self.timestamp = timestamp
        self.data = data

    def isControl(self) -> bool:
        return self.entry == 0

    def _getControlType(self) -> int:
        return self.data[0]

    def isStart(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) >= 17
            and self._getControlType() == kControlStart
        )

    def isFinish(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) == 5
            and self._getControlType() == kControlFinish
        )

    def getStartData(self) -> StartRecordData:
        if not self.isStart():
            raise TypeError("not a start record")
        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        name, pos = self._readInnerString(5)
        type_, pos = self._readInnerString(pos)
        metadata = self._readInnerString(pos)[0]
        return StartRecordData(entry, name, type_, metadata)

    def getBoolean(self) -> bool:
        if len(self.data) != 1:
            raise TypeError("not a boolean")
        return self.data[0] != 0

    def getInteger(self) -> int:
        if len(self.data) != 8:
            raise TypeError("not an integer")
        return int.from_bytes(self.data, byteorder="little", signed=True)

    def getDouble(self) -> float:
        if len(self.data) != 8:
            raise TypeError("not a double")
        return doubleStruct.unpack(self.data)[0]

    def getString(self) -> str:
        return str(self.data, encoding="utf-8")

    def getMsgPack(self):
        return msgpack.unpackb(self.data, strict_map_key=False)

    def getIntegerArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not an integer array")
        arr = array.array("q")  # 8-byte signed
        arr.frombytes(self.data)
        return arr

    def getDoubleArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not a double array")
        arr = array.array("d")
        arr.frombytes(self.data)
        return arr

    def _readInnerString(self, pos: int) -> Tuple[str, int]:
        size = int.from_bytes(self.data[pos:pos + 4], byteorder="little", signed=False)
        end = pos + 4 + size
        if end > len(self.data):
            raise TypeError("invalid string size")
        return str(self.data[pos + 4:end], encoding="utf-8"), end


class DataLogIterator:
    def __init__(self, buf, pos: int):
        self.buf = buf
        self.pos = pos

    def __iter__(self):
        return self

    def _readVarInt(self, pos: int, length: int) -> int:
        val = 0
        for i in range(length):
            val |= self.buf[pos + i] << (i * 8)
        return val

    def __next__(self) -> DataLogRecord:
        if len(self.buf) < (self.pos + 4):
            raise StopIteration
        entryLen = (self.buf[self.pos] & 0x3) + 1
        sizeLen = ((self.buf[self.pos] >> 2) & 0x3) + 1
        timestampLen = ((self.buf[self.pos] >> 4) & 0x7) + 1
        headerLen = 1 + entryLen + sizeLen + timestampLen
        if len(self.buf) < (self.pos + headerLen):
            raise StopIteration
        entry = self._readVarInt(self.pos + 1, entryLen)
        size = self._readVarInt(self.pos + 1 + entryLen, sizeLen)
        timestamp = self._readVarInt(self.pos + 1 + entryLen + sizeLen, timestampLen)
        if len(self.buf) < (self.pos + headerLen + size):
            raise StopIteration
        record = DataLogRecord(
            entry,
            timestamp,
            self.buf[self.pos + headerLen:self.pos + headerLen + size],
        )
        self.pos += headerLen + size
        return record


class DataLogReader:
    def __init__(self, buf):
        self.buf = buf

    def __bool__(self):
        return self.isValid()

    def isValid(self) -> bool:
        return (
            len(self.buf) >= 12
            and self.buf[:6] == b"WPILOG"
            and self.getVersion() >= 0x0100
        )

    def getVersion(self) -> int:
        if len(self.buf) < 12:
            return 0
        return int.from_bytes(self.buf[6:8], byteorder="little", signed=False)

    def __iter__(self):
        extraHeaderSize = int.from_bytes(self.buf[8:12], byteorder="little", signed=False)
        return DataLogIterator(self.buf, 12 + extraHeaderSize)


# --- Vision Log Analysis ---

@dataclass
class VisionEntry:
    """Stores time-series data for a vision-related log entry."""
    name: str
    type: str
    timestamps: List[float]
    values: List[Any]


def parse_wpilog(filepath: str, include_all: bool = False) -> Dict[str, VisionEntry]:
    """Parse a WPILOG file and extract vision-related entries."""
    entries: Dict[int, StartRecordData] = {}
    vision_data: Dict[str, VisionEntry] = {}

    with open(filepath, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            raise ValueError(f"Invalid log file: {filepath}")

        for record in reader:
            timestamp_sec = record.timestamp / 1_000_000.0

            if record.isStart():
                data = record.getStartData()
                entries[data.entry] = data
                # Pre-create vision entries (including AdvantageKit prefixed names)
                if "Vision" in data.name or "vision" in data.name or include_all:
                    vision_data[data.name] = VisionEntry(
                        name=data.name,
                        type=data.type,
                        timestamps=[],
                        values=[]
                    )
            elif record.isFinish():
                pass
            elif record.isControl():
                pass
            else:
                # Data record
                entry = entries.get(record.entry)
                if entry is None:
                    continue
                if entry.name not in vision_data:
                    continue

                try:
                    value = decode_value(record, entry.type)
                    vision_data[entry.name].timestamps.append(timestamp_sec)
                    vision_data[entry.name].values.append(value)
                except Exception as e:
                    pass  # Skip malformed records

        mm.close()

    return vision_data


def normalize_key(key: str) -> str:
    """Remove AdvantageKit prefixes from entry names for comparison."""
    prefixes = ["/RealOutputs/", "/ReplayOutputs/", "/AdvantageKit/"]
    for prefix in prefixes:
        if key.startswith(prefix):
            return key[len(prefix):]
    return key.lstrip("/")


def decode_value(record: DataLogRecord, type_: str) -> Any:
    """Decode a data record based on its type."""
    if type_ == "double":
        return record.getDouble()
    elif type_ == "int64":
        return record.getInteger()
    elif type_ == "boolean":
        return record.getBoolean()
    elif type_ in ("string", "json"):
        return record.getString()
    elif type_ == "double[]":
        return list(record.getDoubleArray())
    elif type_ == "int64[]":
        return list(record.getIntegerArray())
    elif type_.startswith("struct:") or type_.startswith("structarray:"):
        # AdvantageKit struct types - return raw bytes length as indicator
        return len(record.data)
    else:
        # Unknown type, return raw bytes length
        return len(record.data)


def compute_statistics(values: List[float]) -> Dict[str, float]:
    """Compute basic statistics for a list of float values."""
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None, "sum": None}

    values = [v for v in values if v is not None and isinstance(v, (int, float))]
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None, "sum": None}

    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
        "sum": sum(values)
    }


def compare_time_series(name: str, real: VisionEntry, sim: VisionEntry) -> Dict:
    """Compare two time-series entries."""
    result = {
        "name": name,
        "real_count": len(real.values),
        "sim_count": len(sim.values),
        "count_diff": len(sim.values) - len(real.values),
    }

    # For numeric types, compute statistics
    if real.type == "double" and real.values:
        real_stats = compute_statistics(real.values)
        sim_stats = compute_statistics(sim.values)
        result["real_stats"] = real_stats
        result["sim_stats"] = sim_stats

        if real_stats["mean"] is not None and sim_stats["mean"] is not None:
            result["mean_diff"] = sim_stats["mean"] - real_stats["mean"]
            result["mean_diff_pct"] = (
                (sim_stats["mean"] - real_stats["mean"]) / real_stats["mean"] * 100
                if real_stats["mean"] != 0 else None
            )

    return result


def generate_report(real_path: str, sim_path: str) -> str:
    """Generate a detailed comparison report."""
    print(f"Parsing real log: {real_path}")
    real_data = parse_wpilog(real_path)
    print(f"  Found {len(real_data)} vision-related entries")

    print(f"Parsing sim log: {sim_path}")
    sim_data = parse_wpilog(sim_path)
    print(f"  Found {len(sim_data)} vision-related entries")

    # Build normalized lookup for easier comparison
    # Map normalized name -> (real_key, sim_key)
    normalized_map: Dict[str, Tuple[Optional[str], Optional[str]]] = {}
    for k in real_data.keys():
        norm = normalize_key(k)
        if norm not in normalized_map:
            normalized_map[norm] = (k, None)
        else:
            normalized_map[norm] = (k, normalized_map[norm][1])

    for k in sim_data.keys():
        norm = normalize_key(k)
        if norm not in normalized_map:
            normalized_map[norm] = (None, k)
        else:
            normalized_map[norm] = (normalized_map[norm][0], k)

    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("VISION PROCESSING COMPARISON REPORT")
    report_lines.append("=" * 80)
    report_lines.append("")
    report_lines.append(f"Real log: {real_path}")
    report_lines.append(f"Sim log:  {sim_path}")
    report_lines.append("")

    # List all vision entries found
    all_keys = sorted(set(real_data.keys()) | set(sim_data.keys()))

    report_lines.append("-" * 80)
    report_lines.append("VISION ENTRIES OVERVIEW")
    report_lines.append("-" * 80)
    report_lines.append(f"{'Entry Name':<55} {'Real':>8} {'Sim':>8} {'Diff':>8}")
    report_lines.append("-" * 80)

    for key in all_keys:
        real_count = len(real_data[key].values) if key in real_data else 0
        sim_count = len(sim_data[key].values) if key in sim_data else 0
        diff = sim_count - real_count
        diff_str = f"+{diff}" if diff > 0 else str(diff)
        report_lines.append(f"{key:<55} {real_count:>8} {sim_count:>8} {diff_str:>8}")

    report_lines.append("")

    # Detailed analysis of key metrics
    report_lines.append("-" * 80)
    report_lines.append("DETAILED METRIC ANALYSIS")
    report_lines.append("-" * 80)
    report_lines.append("")

    # ObservationScore analysis - find keys with different prefixes
    real_score_key = None
    sim_score_key = None
    for k in real_data.keys():
        if "ObservationScore" in k:
            real_score_key = k
            break
    for k in sim_data.keys():
        if "ObservationScore" in k and "Replay" in k:
            sim_score_key = k
            break
    # Fallback to RealOutputs in sim if no ReplayOutputs
    if sim_score_key is None:
        for k in sim_data.keys():
            if "ObservationScore" in k:
                sim_score_key = k
                break

    score_key = "Vision/Summary/ObservationScore"
    if real_score_key and sim_score_key:
        report_lines.append("## Observation Scores")
        report_lines.append(f"  Real key: {real_score_key}")
        report_lines.append(f"  Sim key:  {sim_score_key}")
        report_lines.append("")

        real_scores = [v for v in real_data[real_score_key].values if isinstance(v, (int, float))]
        sim_scores = [v for v in sim_data[sim_score_key].values if isinstance(v, (int, float))]

        real_stats = compute_statistics(real_scores)
        sim_stats = compute_statistics(sim_scores)

        report_lines.append(f"  Real observations accepted: {real_stats['count']}")
        report_lines.append(f"  Sim observations accepted:  {sim_stats['count']}")
        report_lines.append(f"  Difference: {sim_stats['count'] - real_stats['count']} ({'+' if sim_stats['count'] >= real_stats['count'] else ''}{(sim_stats['count'] - real_stats['count']) / max(real_stats['count'], 1) * 100:.1f}%)")
        report_lines.append("")

        if real_stats['mean'] is not None:
            report_lines.append(f"  Real score statistics:")
            report_lines.append(f"    Min:  {real_stats['min']:.4f}")
            report_lines.append(f"    Max:  {real_stats['max']:.4f}")
            report_lines.append(f"    Mean: {real_stats['mean']:.4f}")

        if sim_stats['mean'] is not None:
            report_lines.append(f"  Sim score statistics:")
            report_lines.append(f"    Min:  {sim_stats['min']:.4f}")
            report_lines.append(f"    Max:  {sim_stats['max']:.4f}")
            report_lines.append(f"    Mean: {sim_stats['mean']:.4f}")

        if real_stats['mean'] is not None and sim_stats['mean'] is not None:
            mean_diff = sim_stats['mean'] - real_stats['mean']
            report_lines.append(f"  Mean score difference: {mean_diff:+.4f} ({mean_diff / real_stats['mean'] * 100:+.1f}%)")

        report_lines.append("")

        # Score distribution comparison
        if real_scores and sim_scores:
            report_lines.append("  Score distribution (buckets of 0.1):")
            buckets = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
            report_lines.append(f"    {'Range':<12} {'Real':>8} {'Sim':>8} {'Diff':>8}")
            for i in range(len(buckets) - 1):
                lo, hi = buckets[i], buckets[i + 1]
                real_in_bucket = sum(1 for s in real_scores if lo <= s < hi)
                sim_in_bucket = sum(1 for s in sim_scores if lo <= s < hi)
                diff = sim_in_bucket - real_in_bucket
                report_lines.append(f"    [{lo:.1f}, {hi:.1f})   {real_in_bucket:>8} {sim_in_bucket:>8} {diff:>+8}")
            report_lines.append("")

    # Per-camera analysis
    report_lines.append("## Per-Camera Analysis")
    report_lines.append("")

    for cam_idx in range(4):
        cam_prefix = f"Vision/Camera{cam_idx}"
        cam_entries = [k for k in all_keys if k.startswith(cam_prefix)]

        if cam_entries:
            report_lines.append(f"  Camera {cam_idx}:")
            for key in sorted(cam_entries):
                real_count = len(real_data[key].values) if key in real_data else 0
                sim_count = len(sim_data[key].values) if key in sim_data else 0
                short_name = key.replace(cam_prefix + "/", "")
                report_lines.append(f"    {short_name:<30} Real: {real_count:>6}, Sim: {sim_count:>6}")
            report_lines.append("")

    # Input analysis (from AdvantageKit processInputs)
    report_lines.append("## Vision Inputs (from AdvantageKit processInputs)")
    report_lines.append("")

    input_keys = [k for k in all_keys if "/poseObservations" in k or "/tagIds" in k or "/connected" in k]
    for key in sorted(input_keys):
        real_count = len(real_data[key].values) if key in real_data else 0
        sim_count = len(sim_data[key].values) if key in sim_data else 0
        report_lines.append(f"  {key:<55}")
        report_lines.append(f"    Real samples: {real_count}, Sim samples: {sim_count}")

        # For connected status, show uptime
        if "/connected" in key and key in real_data:
            real_connected = sum(1 for v in real_data[key].values if v) if real_data[key].values else 0
            sim_connected = sum(1 for v in sim_data[key].values if v) if key in sim_data and sim_data[key].values else 0
            real_pct = real_connected / max(len(real_data[key].values), 1) * 100
            sim_pct = sim_connected / max(len(sim_data[key].values), 1) if key in sim_data else 0
            report_lines.append(f"    Real connected: {real_pct:.1f}%, Sim connected: {sim_pct:.1f}%")
        report_lines.append("")

    # Time range analysis
    report_lines.append("## Time Range Analysis")
    report_lines.append("")

    if real_score_key and real_data[real_score_key].timestamps:
        real_ts = real_data[real_score_key].timestamps
        report_lines.append(f"  Real log time range: {real_ts[0]:.2f}s to {real_ts[-1]:.2f}s ({real_ts[-1] - real_ts[0]:.2f}s duration)")

    if sim_score_key and sim_data[sim_score_key].timestamps:
        sim_ts = sim_data[sim_score_key].timestamps
        report_lines.append(f"  Sim log time range:  {sim_ts[0]:.2f}s to {sim_ts[-1]:.2f}s ({sim_ts[-1] - sim_ts[0]:.2f}s duration)")

    report_lines.append("")

    # RealOutputs vs ReplayOutputs comparison (within sim file)
    report_lines.append("## RealOutputs vs ReplayOutputs (Code Change Analysis)")
    report_lines.append("  This compares the ORIGINAL behavior (RealOutputs) vs NEW code (ReplayOutputs)")
    report_lines.append("  within the simulation replay log.")
    report_lines.append("")

    sim_real_score_key = None
    sim_replay_score_key = None
    for k in sim_data.keys():
        if "ObservationScore" in k:
            if "RealOutputs" in k:
                sim_real_score_key = k
            elif "ReplayOutputs" in k:
                sim_replay_score_key = k

    if sim_real_score_key and sim_replay_score_key:
        sim_real_scores = [v for v in sim_data[sim_real_score_key].values if isinstance(v, (int, float))]
        sim_replay_scores = [v for v in sim_data[sim_replay_score_key].values if isinstance(v, (int, float))]

        report_lines.append(f"  Original code (RealOutputs):  {len(sim_real_scores)} observations")
        report_lines.append(f"  New code (ReplayOutputs):     {len(sim_replay_scores)} observations")
        report_lines.append(f"  Difference: {len(sim_replay_scores) - len(sim_real_scores)} observations")
        report_lines.append("")

        if sim_real_scores:
            real_stats = compute_statistics(sim_real_scores)
            report_lines.append(f"  Original code score statistics:")
            report_lines.append(f"    Min:  {real_stats['min']:.4f}")
            report_lines.append(f"    Max:  {real_stats['max']:.4f}")
            report_lines.append(f"    Mean: {real_stats['mean']:.4f}")

        if sim_replay_scores:
            replay_stats = compute_statistics(sim_replay_scores)
            report_lines.append(f"  New code score statistics:")
            report_lines.append(f"    Min:  {replay_stats['min']:.4f}")
            report_lines.append(f"    Max:  {replay_stats['max']:.4f}")
            report_lines.append(f"    Mean: {replay_stats['mean']:.4f}")

        if sim_real_scores and sim_replay_scores:
            mean_change = replay_stats['mean'] - real_stats['mean']
            report_lines.append("")
            report_lines.append(f"  SCORE CHANGE: {mean_change:+.4f} ({mean_change / real_stats['mean'] * 100:+.1f}%)")
            report_lines.append("")

            # Distribution comparison
            report_lines.append("  Score distribution comparison:")
            buckets = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
            report_lines.append(f"    {'Range':<12} {'Original':>10} {'New':>10} {'Change':>10}")
            for i in range(len(buckets) - 1):
                lo, hi = buckets[i], buckets[i + 1]
                orig = sum(1 for s in sim_real_scores if lo <= s < hi)
                new = sum(1 for s in sim_replay_scores if lo <= s < hi)
                change = new - orig
                report_lines.append(f"    [{lo:.1f}, {hi:.1f})   {orig:>10} {new:>10} {change:>+10}")

    report_lines.append("")

    # Summary findings
    report_lines.append("=" * 80)
    report_lines.append("SUMMARY FINDINGS")
    report_lines.append("=" * 80)
    report_lines.append("")

    findings = []

    if real_score_key and sim_score_key:
        real_count = len(real_data[real_score_key].values)
        sim_count = len(sim_data[sim_score_key].values)

        if sim_count == real_count:
            findings.append("- Observation counts MATCH exactly between real and sim")
        elif sim_count > real_count:
            findings.append(f"- Sim has MORE accepted observations than real ({sim_count} vs {real_count}, +{sim_count - real_count})")
        else:
            findings.append(f"- Sim has FEWER accepted observations than real ({sim_count} vs {real_count}, {sim_count - real_count})")

        real_scores = [v for v in real_data[real_score_key].values if isinstance(v, (int, float))]
        sim_scores = [v for v in sim_data[sim_score_key].values if isinstance(v, (int, float))]

        if real_scores and sim_scores:
            real_mean = sum(real_scores) / len(real_scores)
            sim_mean = sum(sim_scores) / len(sim_scores)

            if abs(sim_mean - real_mean) < 0.01:
                findings.append(f"- Mean scores are SIMILAR (real: {real_mean:.4f}, sim: {sim_mean:.4f})")
            elif sim_mean > real_mean:
                findings.append(f"- Sim has HIGHER mean score (real: {real_mean:.4f}, sim: {sim_mean:.4f}, +{sim_mean - real_mean:.4f})")
            else:
                findings.append(f"- Sim has LOWER mean score (real: {real_mean:.4f}, sim: {sim_mean:.4f}, {sim_mean - real_mean:.4f})")

    # Check for entries only in one log
    only_in_real = [k for k in real_data.keys() if k not in sim_data]
    only_in_sim = [k for k in sim_data.keys() if k not in real_data]

    if only_in_real:
        findings.append(f"- {len(only_in_real)} entries only in real log")
    if only_in_sim:
        findings.append(f"- {len(only_in_sim)} entries only in sim log")

    # Input vs output analysis
    for cam in range(4):
        real_pose_key = f"/Vision/Camera{cam}/PoseObservations"
        real_tag_key = f"/Vision/Camera{cam}/TagIds"
        if real_pose_key in real_data and real_pose_key in sim_data:
            real_poses = len(real_data[real_pose_key].values)
            sim_poses = len(sim_data[real_pose_key].values)
            if real_poses != sim_poses:
                findings.append(f"- Camera {cam} input mismatch: {real_poses} real vs {sim_poses} sim PoseObservations")

    # Score threshold analysis
    if real_score_key and sim_score_key:
        real_scores = [v for v in real_data[real_score_key].values if isinstance(v, (int, float))]
        sim_scores = [v for v in sim_data[sim_score_key].values if isinstance(v, (int, float))]

        min_score = 0.02  # VisionConstants.minScore
        real_above_min = sum(1 for s in real_scores if s > min_score)
        sim_above_min = sum(1 for s in sim_scores if s > min_score)

        findings.append(f"- Original code: {real_above_min}/{len(real_scores)} scored above minScore (0.02)")
        findings.append(f"- New code: {sim_above_min}/{len(sim_scores)} scored above minScore (0.02)")

    for finding in findings:
        report_lines.append(finding)

    report_lines.append("")
    report_lines.append("=" * 80)

    return "\n".join(report_lines)


def main():
    if len(sys.argv) != 3:
        print("Usage: compare_vision_logs.py <real_log.wpilog> <sim_log.wpilog>")
        sys.exit(1)

    real_path = sys.argv[1]
    sim_path = sys.argv[2]

    report = generate_report(real_path, sim_path)
    print(report)


if __name__ == "__main__":
    main()
