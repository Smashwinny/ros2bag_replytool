"""Canonical hashes for bit-exact ESKF replay snapshots."""
import hashlib
import math
import struct


def _number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("snapshot contains a non-finite number")
    return 0.0 if number == 0.0 else number


def _double_bytes(values):
    return b"".join(struct.pack(">d", _number(value)) for value in values)


def _canonical(value):
    if value is None:
        return b"n"
    if isinstance(value, bool):
        return b"b1" if value else b"b0"
    if isinstance(value, int):
        encoded = str(value).encode("ascii")
        return b"i" + struct.pack(">I", len(encoded)) + encoded
    if isinstance(value, float):
        return b"f" + struct.pack(">d", _number(value))
    if isinstance(value, str):
        encoded = value.encode("utf-8")
        return b"s" + struct.pack(">I", len(encoded)) + encoded
    if isinstance(value, list):
        return b"l" + struct.pack(">I", len(value)) + b"".join(
            _canonical(item) for item in value)
    if isinstance(value, dict):
        items = sorted(value.items())
        return b"d" + struct.pack(">I", len(items)) + b"".join(
            _canonical(str(key)) + _canonical(item) for key, item in items)
    raise ValueError(f"unsupported snapshot value: {type(value).__name__}")


def snapshot_hashes(snapshot):
    state = snapshot["nominal_state_pvqwxyz_bg_ba"]
    covariance = snapshot["covariance_row_major"]
    trajectory = snapshot["output_trajectory"]
    if len(state) != 16:
        raise ValueError("nominal state must contain 16 doubles")
    if len(covariance) != 225:
        raise ValueError("covariance must contain 225 doubles")
    trajectory_bytes = bytearray()
    for row in trajectory:
        if len(row) != 9:
            raise ValueError("trajectory row must contain ordinal and 8 doubles")
        ordinal = int(row[0])
        if ordinal < 0 or ordinal > (1 << 64) - 1:
            raise ValueError("trajectory ordinal is outside uint64")
        trajectory_bytes.extend(struct.pack(">Q", ordinal))
        trajectory_bytes.extend(_double_bytes(row[1:]))
    return {
        "state_sha256": hashlib.sha256(_canonical([
            state, snapshot["node_state_summary"]])).hexdigest(),
        "covariance_sha256": hashlib.sha256(_double_bytes(covariance)).hexdigest(),
        "trajectory_sha256": hashlib.sha256(trajectory_bytes).hexdigest(),
    }


def compare_three(runs):
    if len(runs) != 3:
        raise ValueError("bit-exact audit requires exactly three runs")
    keys = ("state_sha256", "covariance_sha256", "trajectory_sha256")
    matches = {key: len({run[key] for run in runs}) == 1 for key in keys}
    return {"bit_exact": all(matches.values()), "matches": matches}


def _first_difference(left, right, path="$"):
    if type(left) is not type(right):
        return path
    if isinstance(left, dict):
        if sorted(left) != sorted(right):
            return path + ".keys"
        for key in sorted(left):
            found = _first_difference(left[key], right[key], f"{path}.{key}")
            if found:
                return found
        return None
    if isinstance(left, list):
        if len(left) != len(right):
            return path + ".length"
        for index, (a, b) in enumerate(zip(left, right)):
            found = _first_difference(a, b, f"{path}[{index}]")
            if found:
                return found
        return None
    if isinstance(left, float):
        return None if struct.pack(">d", _number(left)) == struct.pack(">d", _number(right)) else path
    return None if left == right else path


def first_differences(snapshots):
    if len(snapshots) != 3:
        raise ValueError("bit-exact audit requires exactly three snapshots")
    categories = {
        "state": lambda value: [value["nominal_state_pvqwxyz_bg_ba"],
                                value["node_state_summary"]],
        "covariance": lambda value: value["covariance_row_major"],
        "trajectory": lambda value: value["output_trajectory"],
    }
    result = {}
    for name, select in categories.items():
        reference = select(snapshots[0])
        result[name] = next((
            _first_difference(reference, select(snapshot), f"$.{name}")
            for snapshot in snapshots[1:]
            if _first_difference(reference, select(snapshot), f"$.{name}")), None)
    return result
