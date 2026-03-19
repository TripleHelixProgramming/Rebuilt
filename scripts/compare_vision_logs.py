#!/usr/bin/env python3
"""
Compare vision processing between two WPILOG files (real vs simulation replay).

Analyzes:
- Vision inputs (poseObservations, tagIds, connected status)
- Vision outputs (ObservationScore, RobotPosesAccepted, RobotPosesRejected)
- Timing differences
- Score distributions
- False positive detection heuristics (pose jumps, velocity consistency)
"""

import array
import mmap
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any, Set

import msgpack

# Constants for analysis
MAX_REASONABLE_VELOCITY_MPS = 7.5  # ~1.5x typical FRC max speed
TIMESTAMP_MATCH_TOLERANCE = 0.001  # 1ms tolerance for matching timestamps
POSE_JUMP_THRESHOLD_METERS = 0.5  # Flag jumps larger than this

# Struct sizes (WPILib serialization)
# Pose3d = Translation3d (3 doubles) + Rotation3d as Quaternion (4 doubles) = 56 bytes
POSE3D_STRUCT_SIZE = 56  # 7 doubles * 8 bytes


@dataclass
class Pose3d:
    """Decoded Pose3d from WPILib struct format."""
    x: float
    y: float
    z: float
    qw: float  # Quaternion w component
    qx: float  # Quaternion x component
    qy: float  # Quaternion y component
    qz: float  # Quaternion z component

    def distance_2d(self, other: 'Pose3d') -> float:
        """2D distance (ignoring Z) between two poses."""
        dx = self.x - other.x
        dy = self.y - other.y
        return (dx * dx + dy * dy) ** 0.5

    def distance_3d(self, other: 'Pose3d') -> float:
        """3D distance between two poses."""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5


def decode_pose3d(data: bytes, offset: int = 0) -> Optional[Pose3d]:
    """Decode a single Pose3d from raw bytes at the given offset."""
    if len(data) < offset + POSE3D_STRUCT_SIZE:
        return None
    try:
        # WPILib serializes as: x, y, z (Translation3d), then qw, qx, qy, qz (Quaternion)
        values = struct.unpack_from('<7d', data, offset)
        return Pose3d(
            x=values[0], y=values[1], z=values[2],
            qw=values[3], qx=values[4], qy=values[5], qz=values[6]
        )
    except struct.error:
        return None


def decode_pose3d_array(data: bytes) -> List[Pose3d]:
    """Decode an array of Pose3d structs from raw bytes."""
    poses = []
    offset = 0
    while offset + POSE3D_STRUCT_SIZE <= len(data):
        pose = decode_pose3d(data, offset)
        if pose:
            poses.append(pose)
        offset += POSE3D_STRUCT_SIZE
    return poses

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
    elif "Pose3d" in type_ and type_.startswith("structarray:"):
        # Decode Pose3d array
        return decode_pose3d_array(record.data)
    elif type_.startswith("struct:") or type_.startswith("structarray:"):
        # Other struct types - return raw bytes for potential future decoding
        return record.data
    else:
        # Unknown type, return raw bytes
        return record.data


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


# --- False Positive Analysis Functions ---


@dataclass
class TimestampedScore:
    """A score with its timestamp for matching across code versions."""
    timestamp: float
    score: float


@dataclass
class PoseJump:
    """Records a suspicious pose discontinuity."""
    timestamp: float
    prev_timestamp: float
    distance_m: float
    implied_velocity_mps: float
    score: float


@dataclass
class FalsePositiveAnalysis:
    """Results of false positive detection heuristics."""
    # Observations old code accepted but new code rejected
    newly_rejected_count: int
    newly_rejected_timestamps: List[float]

    # Pose discontinuities in accepted trajectory
    pose_jumps: List[PoseJump]
    max_jump_distance: float
    jumps_above_threshold: int

    # Velocity consistency violations
    impossible_velocities: List[PoseJump]
    max_implied_velocity: float

    # Score comparison for observations both accepted
    both_accepted_count: int
    score_improvements: List[Tuple[float, float, float]]  # (timestamp, old_score, new_score)
    mean_score_change_when_both_accepted: float


def find_timestamp_matches(
    timestamps_a: List[float],
    timestamps_b: List[float],
    tolerance: float = TIMESTAMP_MATCH_TOLERANCE
) -> Tuple[Set[int], Set[int], List[Tuple[int, int]]]:
    """
    Find matching timestamps between two lists.

    Returns:
        - indices in A that have no match in B
        - indices in B that have no match in A
        - list of (index_a, index_b) pairs that match
    """
    matched_a: Set[int] = set()
    matched_b: Set[int] = set()
    matches: List[Tuple[int, int]] = []

    # Sort B indices by timestamp for efficient searching
    sorted_b = sorted(enumerate(timestamps_b), key=lambda x: x[1])

    for i, ts_a in enumerate(timestamps_a):
        # Binary search for closest match in B
        best_j = None
        best_diff = float('inf')

        for j, ts_b in sorted_b:
            diff = abs(ts_a - ts_b)
            if diff < best_diff:
                best_diff = diff
                best_j = j
            elif ts_b > ts_a + tolerance:
                # No point searching further
                break

        if best_j is not None and best_diff <= tolerance:
            if best_j not in matched_b:  # Each B can only match once
                matched_a.add(i)
                matched_b.add(best_j)
                matches.append((i, best_j))

    unmatched_a = set(range(len(timestamps_a))) - matched_a
    unmatched_b = set(range(len(timestamps_b))) - matched_b

    return unmatched_a, unmatched_b, matches


def analyze_pose_trajectory_from_poses(
    timestamps: List[float],
    pose_arrays: List[List[Pose3d]],
    scores: Optional[List[float]] = None
) -> Tuple[List[PoseJump], List[PoseJump]]:
    """
    Analyze accepted pose trajectory for discontinuities.

    Looks at consecutive accepted poses and flags:
    1. Large position jumps (> POSE_JUMP_THRESHOLD_METERS)
    2. Implied velocities that exceed MAX_REASONABLE_VELOCITY_MPS

    Args:
        timestamps: List of timestamps for each pose array
        pose_arrays: List of Pose3d arrays (one per logged frame)
        scores: Optional scores to include in results

    Returns:
        - List of pose jumps above threshold
        - List of impossible velocity violations
    """
    pose_jumps: List[PoseJump] = []
    velocity_violations: List[PoseJump] = []

    # Flatten all poses with their timestamps
    all_poses: List[Tuple[float, Pose3d, float]] = []
    for i, (ts, poses) in enumerate(zip(timestamps, pose_arrays)):
        if not isinstance(poses, list):
            continue
        score = scores[i] if scores and i < len(scores) else 0.0
        for pose in poses:
            if isinstance(pose, Pose3d):
                all_poses.append((ts, pose, score))

    # Sort by timestamp
    all_poses.sort(key=lambda x: x[0])

    # Analyze consecutive poses
    for i in range(1, len(all_poses)):
        prev_ts, prev_pose, prev_score = all_poses[i - 1]
        curr_ts, curr_pose, curr_score = all_poses[i]

        dt = curr_ts - prev_ts
        if dt <= 0.001:  # Skip near-simultaneous observations
            continue

        distance = prev_pose.distance_2d(curr_pose)
        velocity = distance / dt

        # Check for large jumps
        if distance > POSE_JUMP_THRESHOLD_METERS:
            pose_jumps.append(PoseJump(
                timestamp=curr_ts,
                prev_timestamp=prev_ts,
                distance_m=distance,
                implied_velocity_mps=velocity,
                score=curr_score
            ))

        # Check for impossible velocities
        if velocity > MAX_REASONABLE_VELOCITY_MPS:
            velocity_violations.append(PoseJump(
                timestamp=curr_ts,
                prev_timestamp=prev_ts,
                distance_m=distance,
                implied_velocity_mps=velocity,
                score=curr_score
            ))

    return pose_jumps, velocity_violations


def analyze_false_positives(
    sim_data: Dict[str, 'VisionEntry']
) -> Optional[FalsePositiveAnalysis]:
    """
    Analyze simulation replay data for potential false positives.

    Compares RealOutputs (original code) vs ReplayOutputs (new code) to identify:
    1. Observations old code accepted but new code rejected (potential false positives caught)
    2. Score changes for observations both codes accepted

    IMPORTANT CAVEATS:
    - This is HEURISTIC analysis, not ground truth
    - "Newly rejected" observations may include both:
      - True false positives (correctly rejected bad data)
      - False negatives (incorrectly rejected good data)
    - Without ground truth robot position, we cannot definitively classify

    Returns None if required data is not available.
    """
    # Find score keys
    real_score_key = None
    replay_score_key = None

    for k in sim_data.keys():
        if "ObservationScore" in k:
            if "RealOutputs" in k:
                real_score_key = k
            elif "ReplayOutputs" in k:
                replay_score_key = k

    if not real_score_key or not replay_score_key:
        return None

    real_entry = sim_data[real_score_key]
    replay_entry = sim_data[replay_score_key]

    # Get timestamps and scores
    real_timestamps = real_entry.timestamps
    real_scores = [v for v in real_entry.values if isinstance(v, (int, float))]
    replay_timestamps = replay_entry.timestamps
    replay_scores = [v for v in replay_entry.values if isinstance(v, (int, float))]

    if len(real_timestamps) != len(real_scores):
        real_timestamps = real_timestamps[:len(real_scores)]
    if len(replay_timestamps) != len(replay_scores):
        replay_timestamps = replay_timestamps[:len(replay_scores)]

    # Find timestamp matches
    unmatched_real, _unmatched_replay, matches = find_timestamp_matches(
        real_timestamps, replay_timestamps
    )

    # Newly rejected = old code accepted (has timestamp) but new code rejected (no matching timestamp)
    newly_rejected_timestamps = [real_timestamps[i] for i in unmatched_real]

    # Score comparison for matched observations
    score_improvements: List[Tuple[float, float, float]] = []
    for i_real, i_replay in matches:
        old_score = real_scores[i_real]
        new_score = replay_scores[i_replay]
        ts = real_timestamps[i_real]
        score_improvements.append((ts, old_score, new_score))

    mean_change = 0.0
    if score_improvements:
        changes = [new - old for _, old, new in score_improvements]
        mean_change = sum(changes) / len(changes)

    # Analyze pose trajectories for false positives (bad data that was accepted)
    # Look for RobotPosesAccepted in both RealOutputs and ReplayOutputs
    pose_jumps: List[PoseJump] = []
    velocity_violations: List[PoseJump] = []

    # Analyze OLD code's accepted poses (looking for false positives it let through)
    real_poses_key = None
    replay_poses_key = None
    for k in sim_data.keys():
        if "RobotPosesAccepted" in k:
            if "RealOutputs" in k:
                real_poses_key = k
            elif "ReplayOutputs" in k:
                replay_poses_key = k

    # Analyze old code's accepted trajectory
    if real_poses_key and real_poses_key in sim_data:
        entry = sim_data[real_poses_key]
        # Decode bytes to Pose3d arrays
        pose_arrays: List[List[Pose3d]] = []
        timestamps_for_poses: List[float] = []
        for i, val in enumerate(entry.values):
            if isinstance(val, bytes) and len(val) > 0:
                decoded = decode_pose3d_array(val)
                if decoded:
                    pose_arrays.append(decoded)
                    if i < len(entry.timestamps):
                        timestamps_for_poses.append(entry.timestamps[i])
            elif isinstance(val, list) and val and isinstance(val[0], Pose3d):
                pose_arrays.append(val)
                if i < len(entry.timestamps):
                    timestamps_for_poses.append(entry.timestamps[i])

        if pose_arrays:
            old_jumps, old_violations = analyze_pose_trajectory_from_poses(
                timestamps_for_poses,
                pose_arrays,
                None  # No per-pose scores available
            )
            pose_jumps.extend(old_jumps)
            velocity_violations.extend(old_violations)

    return FalsePositiveAnalysis(
        newly_rejected_count=len(unmatched_real),
        newly_rejected_timestamps=newly_rejected_timestamps,
        pose_jumps=pose_jumps,
        max_jump_distance=max((j.distance_m for j in pose_jumps), default=0.0),
        jumps_above_threshold=sum(1 for j in pose_jumps if j.distance_m > POSE_JUMP_THRESHOLD_METERS),
        impossible_velocities=velocity_violations,
        max_implied_velocity=max((v.implied_velocity_mps for v in velocity_violations), default=0.0),
        both_accepted_count=len(matches),
        score_improvements=score_improvements,
        mean_score_change_when_both_accepted=mean_change
    )


def generate_false_positive_report(analysis: FalsePositiveAnalysis) -> List[str]:
    """Generate report section for false positive analysis."""
    lines = []
    lines.append("")
    lines.append("=" * 80)
    lines.append("FALSE POSITIVE ANALYSIS (HEURISTIC)")
    lines.append("=" * 80)
    lines.append("")
    lines.append("IMPORTANT CAVEATS:")
    lines.append("  - This analysis uses HEURISTICS, not ground truth")
    lines.append("  - Without knowing the robot's actual position, we cannot definitively")
    lines.append("    classify observations as 'false positives' or 'false negatives'")
    lines.append("  - The following are INDICATORS that warrant investigation, not proof")
    lines.append("")

    # Section 1: Newly rejected observations
    lines.append("-" * 80)
    lines.append("1. OBSERVATIONS OLD CODE ACCEPTED BUT NEW CODE REJECTED")
    lines.append("-" * 80)
    lines.append("")
    lines.append(f"  Count: {analysis.newly_rejected_count}")
    lines.append("")
    lines.append("  Interpretation:")
    lines.append("    These observations passed the old filter but failed the new filter.")
    lines.append("    They could be:")
    lines.append("      a) TRUE FALSE POSITIVES: Bad data the old code incorrectly accepted")
    lines.append("      b) FALSE NEGATIVES: Good data the new code incorrectly rejects")
    lines.append("")
    lines.append("    Without ground truth, we cannot determine which category each falls into.")
    lines.append("")

    if analysis.newly_rejected_timestamps:
        # Show time distribution
        ts = analysis.newly_rejected_timestamps
        lines.append(f"  Time range: {min(ts):.2f}s to {max(ts):.2f}s")

        # Bucket by 30-second intervals
        if ts:
            min_t = min(ts)
            bucket_size = 30.0
            buckets: Dict[int, int] = defaultdict(int)
            for t in ts:
                bucket = int((t - min_t) / bucket_size)
                buckets[bucket] += 1

            lines.append("")
            lines.append("  Distribution over time (30-second buckets):")
            for b in sorted(buckets.keys())[:10]:  # Show first 10 buckets
                start = min_t + b * bucket_size
                lines.append(f"    {start:6.1f}s - {start + bucket_size:6.1f}s: {buckets[b]:>4} rejected")
            if len(buckets) > 10:
                lines.append(f"    ... and {len(buckets) - 10} more buckets")

    lines.append("")

    # Section 2: Score comparison for both-accepted
    lines.append("-" * 80)
    lines.append("2. SCORE COMPARISON FOR OBSERVATIONS BOTH CODES ACCEPTED")
    lines.append("-" * 80)
    lines.append("")
    lines.append(f"  Observations accepted by both: {analysis.both_accepted_count}")
    lines.append(f"  Mean score change (new - old): {analysis.mean_score_change_when_both_accepted:+.4f}")
    lines.append("")

    if analysis.score_improvements:
        # Analyze score changes
        improvements = [(ts, old, new) for ts, old, new in analysis.score_improvements if new > old]
        degradations = [(ts, old, new) for ts, old, new in analysis.score_improvements if new < old]
        unchanged = [(ts, old, new) for ts, old, new in analysis.score_improvements if abs(new - old) < 0.001]

        lines.append(f"  Score increased (new > old): {len(improvements)} ({100*len(improvements)/len(analysis.score_improvements):.1f}%)")
        lines.append(f"  Score decreased (new < old): {len(degradations)} ({100*len(degradations)/len(analysis.score_improvements):.1f}%)")
        lines.append(f"  Score unchanged: {len(unchanged)} ({100*len(unchanged)/len(analysis.score_improvements):.1f}%)")
        lines.append("")

        if improvements:
            avg_improvement = sum(new - old for _, old, new in improvements) / len(improvements)
            lines.append(f"  Average improvement when score increased: +{avg_improvement:.4f}")
        if degradations:
            avg_degradation = sum(new - old for _, old, new in degradations) / len(degradations)
            lines.append(f"  Average degradation when score decreased: {avg_degradation:.4f}")

        lines.append("")
        lines.append("  Interpretation:")
        lines.append("    Score increases indicate the new code assigns higher confidence to")
        lines.append("    observations it considers high-quality. This is expected behavior.")
        lines.append("    Score decreases are rare and may indicate edge cases worth reviewing.")

    lines.append("")

    # Section 3: Pose trajectory analysis (actual false positive detection)
    lines.append("-" * 80)
    lines.append("3. POSE TRAJECTORY ANALYSIS (FALSE POSITIVE DETECTION)")
    lines.append("-" * 80)
    lines.append("")
    lines.append("  This analyzes ACCEPTED poses for suspicious patterns that indicate")
    lines.append("  bad data that should have been rejected (true false positives).")
    lines.append("")

    if analysis.pose_jumps or analysis.impossible_velocities:
        lines.append(f"  Pose jumps > {POSE_JUMP_THRESHOLD_METERS}m: {len(analysis.pose_jumps)}")
        lines.append(f"  Impossible velocities > {MAX_REASONABLE_VELOCITY_MPS} m/s: {len(analysis.impossible_velocities)}")
        lines.append(f"  Max jump distance: {analysis.max_jump_distance:.2f}m")
        lines.append(f"  Max implied velocity: {analysis.max_implied_velocity:.1f} m/s")
        lines.append("")

        if analysis.impossible_velocities:
            lines.append("  SUSPICIOUS OBSERVATIONS (impossible velocity):")
            # Show worst violations
            sorted_violations = sorted(analysis.impossible_velocities,
                                       key=lambda x: x.implied_velocity_mps, reverse=True)
            for v in sorted_violations[:10]:
                lines.append(f"    t={v.timestamp:.2f}s: {v.distance_m:.2f}m in {(v.timestamp - v.prev_timestamp)*1000:.0f}ms = {v.implied_velocity_mps:.1f} m/s")
            if len(sorted_violations) > 10:
                lines.append(f"    ... and {len(sorted_violations) - 10} more")
            lines.append("")

        lines.append("  Interpretation:")
        lines.append("    High velocity violations indicate poses that jumped impossibly fast.")
        lines.append("    These are likely FALSE POSITIVES - bad data that was accepted.")
        if analysis.impossible_velocities:
            lines.append("    The new filter's velocity consistency check should catch these.")
    else:
        lines.append("  No pose data decoded (RobotPosesAccepted may not be logged).")
        lines.append("  To enable this analysis, set kLogAcceptedPoses = true in VisionConstants.java")

    lines.append("")

    # Section 4: Limitations
    lines.append("-" * 80)
    lines.append("4. LIMITATIONS OF THIS ANALYSIS")
    lines.append("-" * 80)
    lines.append("")
    lines.append("  What we CANNOT determine from logs alone:")
    lines.append("    - Ground truth robot position (no external reference)")
    lines.append("    - Why specific observations were rejected (per-test scores not logged)")
    lines.append("")
    lines.append("  To enable deeper analysis, consider:")
    lines.append("    - Setting kLogRejectedPoses = true to see rejected pose values")
    lines.append("    - Logging individual test scores for rejected observations")
    lines.append("    - Recording ground truth position data for validation")
    lines.append("")

    # Section 5: Recommendations
    lines.append("-" * 80)
    lines.append("5. RECOMMENDATIONS FOR VALIDATION")
    lines.append("-" * 80)
    lines.append("")
    lines.append("  To validate the new filter is not rejecting good data:")
    lines.append("")
    lines.append("  a) VISUAL INSPECTION:")
    lines.append("     Open both logs in AdvantageScope and compare pose trajectories")
    lines.append("     Look for cases where the old trajectory is smoother/more accurate")
    lines.append("")
    lines.append("  b) GROUND TRUTH COMPARISON:")
    lines.append("     If field position markers are available, compare accepted poses")
    lines.append("     to known positions at specific match times")
    lines.append("")
    lines.append("  c) COMPETITION TESTING:")
    lines.append("     Run both filter versions in practice matches and compare")
    lines.append("     autonomous accuracy and teleop behavior")
    lines.append("")

    return lines


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

    # False positive analysis
    fp_analysis = analyze_false_positives(sim_data)
    if fp_analysis:
        report_lines.extend(generate_false_positive_report(fp_analysis))

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
