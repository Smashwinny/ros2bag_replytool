#!/usr/bin/env python3
"""Content-addressed guard for expensive replay-only build artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


SCHEMA = "rosbag_replay_build_artifacts/v1"


def hash_paths(paths: list[Path]) -> str:
    digest = hashlib.sha256()
    files: list[tuple[str, Path]] = []
    for root in paths:
        resolved = root.resolve()
        if not resolved.exists():
            raise FileNotFoundError(resolved)
        if resolved.is_file():
            files.append((str(resolved), resolved))
            continue
        for item in resolved.rglob("*"):
            if (item.is_file() and ".git" not in item.parts and
                    "__pycache__" not in item.parts):
                files.append((str(item.resolve()), item.resolve()))
    for name, item in sorted(files):
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        with item.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def load_state(path: Path) -> dict | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) and value.get("schema") == SCHEMA else None


def artifacts_valid(state: dict | None, source_hash: str,
                    artifacts: list[Path]) -> bool:
    if state is None or state.get("source_sha256") != source_hash:
        return False
    if not all(path.is_file() for path in artifacts):
        return False
    return state.get("artifact_sha256") == hash_paths(artifacts)


def write_state(path: Path, source_hash: str, artifacts: list[Path]) -> None:
    payload = {
        "schema": SCHEMA,
        "source_sha256": source_hash,
        "artifact_sha256": hash_paths(artifacts),
        "artifacts": [str(item.resolve()) for item in artifacts],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--source", type=Path, action="append", default=[])
    parser.add_argument("--artifact", type=Path, action="append", default=[])
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not args.source or not args.artifact or not command:
        parser.error("--source, --artifact and a command after -- are required")

    source_hash = hash_paths(args.source)
    if artifacts_valid(load_state(args.state), source_hash, args.artifact):
        print(f"ESKF replay build unchanged: reuse artifacts ({source_hash[:12]})")
        return 0

    print("ESKF replay inputs changed or build state is missing; rebuilding...")
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        return completed.returncode
    missing = [str(path) for path in args.artifact if not path.is_file()]
    if missing:
        raise RuntimeError("build completed without artifacts: " + ", ".join(missing))
    write_state(args.state, source_hash, args.artifact)
    print(f"ESKF replay build recorded ({source_hash[:12]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
