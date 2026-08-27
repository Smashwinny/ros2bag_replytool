#!/usr/bin/env python3
"""Persistent, content-addressed display cache for completed ESKF replays."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
from typing import Iterable


SCHEMA = "eskf_persistent_display_cache/v1"
INDEX_SCHEMA = 1


def reset_build_database(path: Path) -> None:
    """Remove an interrupted SQLite build and its journal sidecars."""
    for candidate in (path, Path(f"{path}-wal"), Path(f"{path}-shm")):
        candidate.unlink(missing_ok=True)


def stale_build_cache_directories(cache_root: Path, bag_dir: Path,
                                  target_yaml: Path,
                                  current_build_state: dict) -> list[Path]:
    """Find complete caches for the same inputs but an older replay build."""
    bag_path = str(bag_dir.resolve())
    target_path = str(target_yaml.resolve())
    stale = []
    if not cache_root.is_dir():
        return stale
    for manifest_path in cache_root.glob("*/manifest.json"):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            identity = manifest["identity"]
            same_inputs = (
                identity["bag"]["path"] == bag_path and
                identity["target_yaml"]["path"] == target_path)
            old_build = identity["build_state"] != current_build_state
        except (OSError, KeyError, TypeError, json.JSONDecodeError):
            continue
        if (manifest.get("schema") == SCHEMA and manifest.get("complete") and
                same_inputs and old_build):
            stale.append(manifest_path.parent)
    return sorted(stale)


def remove_cache_directories(cache_root: Path,
                             directories: Iterable[Path]) -> int:
    """Remove only direct, content-addressed children of the cache root."""
    root = cache_root.resolve()
    removed = 0
    for directory in directories:
        candidate = directory.resolve()
        if (candidate.parent != root or len(candidate.name) != 64 or
                any(character not in "0123456789abcdef"
                    for character in candidate.name)):
            raise ValueError(f"unsafe cache directory: {candidate}")
        if candidate.is_dir():
            shutil.rmtree(candidate)
            removed += 1
    return removed


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bag_signature(bag_dir: Path) -> dict:
    bag_dir = bag_dir.resolve()
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        raise FileNotFoundError(metadata)
    storage = []
    for item in sorted(bag_dir.iterdir()):
        if item.is_file() and item.name != "metadata.yaml":
            stat = item.stat()
            storage.append({"name": item.name, "size": stat.st_size,
                            "mtime_ns": stat.st_mtime_ns,
                            "sha256": _sha256_file(item)})
    return {"path": str(bag_dir), "metadata_sha256": _sha256_file(metadata),
            "storage": storage}


def cache_fingerprint(bag_dir: Path, target_yaml: Path,
                      build_state: Path) -> tuple[str, dict]:
    identity = {
        "schema": SCHEMA,
        "bag": bag_signature(bag_dir),
        "target_yaml": {"path": str(target_yaml.resolve()),
                        "sha256": _sha256_file(target_yaml)},
        "build_state": json.loads(build_state.read_text(encoding="utf-8")),
    }
    canonical = json.dumps(identity, separators=(",", ":"),
                           sort_keys=True).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest(), identity


def file_edge_signature(path: Path) -> dict:
    size = path.stat().st_size
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        head = stream.read(min(size, 1024 * 1024))
        if size > len(head):
            stream.seek(max(0, size - 1024 * 1024))
            tail = stream.read(1024 * 1024)
        else:
            tail = b""
    digest.update(head)
    digest.update(tail)
    return {"size": size, "edge_sha256": digest.hexdigest()}


def create_database(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path)
    connection.executescript("""
      PRAGMA journal_mode=WAL;
      PRAGMA synchronous=NORMAL;
      CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT NOT NULL);
      CREATE TABLE process_index(
        stamp_ns INTEGER NOT NULL, file_offset INTEGER NOT NULL,
        byte_length INTEGER NOT NULL, event TEXT NOT NULL,
        sequence INTEGER NOT NULL);
      CREATE INDEX process_time_idx ON process_index(stamp_ns, file_offset);
      CREATE TABLE trajectory(
        kind TEXT NOT NULL, stamp_ns INTEGER NOT NULL,
        x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
        qw REAL NOT NULL, qx REAL NOT NULL, qy REAL NOT NULL, qz REAL NOT NULL);
      CREATE INDEX trajectory_time_idx ON trajectory(kind, stamp_ns);
    """)
    connection.execute("INSERT INTO metadata VALUES(?, ?)",
                       ("index_schema", str(INDEX_SCHEMA)))
    return connection


def _record_stamp_ns(record: dict) -> int | None:
    value = record.get("source_time_s")
    if not isinstance(value, (int, float)) or value <= 0:
        value = record.get("ros_time_s")
    if not isinstance(value, (int, float)) or value <= 0:
        return None
    return int(round(float(value) * 1_000_000_000))


def index_process_log(connection: sqlite3.Connection,
                      process_log: Path) -> tuple[int, int]:
    indexed = poses = 0
    last_position: dict[str, tuple[float, float]] = {}
    with process_log.open("rb") as stream:
        while True:
            offset = stream.tell()
            line = stream.readline()
            if not line:
                break
            try:
                record = json.loads(line)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            stamp_ns = _record_stamp_ns(record)
            if stamp_ns is None:
                continue
            event = str(record.get("event", ""))
            sequence = int(record.get("sequence", -1))
            connection.execute(
                "INSERT INTO process_index VALUES(?,?,?,?,?)",
                (stamp_ns, offset, len(line), event, sequence))
            indexed += 1
            if indexed % 10000 == 0:
                connection.commit()
            if event != "pose_published":
                continue
            position = record.get("published_position_wb")
            orientation = record.get("published_orientation_wb_wxyz")
            if (not isinstance(position, list) or len(position) < 3 or
                    not isinstance(orientation, list) or len(orientation) < 4):
                continue
            kind = ("eskf" if record.get("filter_anchored") is True
                    else "eskf_provisional")
            x, y = float(position[0]), float(position[1])
            previous = last_position.get(kind)
            if (previous is not None and
                    (x - previous[0]) ** 2 + (y - previous[1]) ** 2 < 0.02 ** 2):
                continue
            connection.execute(
                "INSERT INTO trajectory VALUES(?,?,?,?,?,?,?,?,?)",
                (kind, stamp_ns, x, y, float(position[2]),
                 *map(float, orientation[:4])))
            last_position[kind] = (x, y)
            poses += 1
    connection.commit()
    return indexed, poses


def insert_trajectory(connection: sqlite3.Connection, kind: str,
                      poses: Iterable[tuple]) -> int:
    rows, previous = [], None
    for pose in poses:
        current = (float(pose[1]), float(pose[2]))
        if (previous is not None and
                (current[0] - previous[0]) ** 2 +
                (current[1] - previous[1]) ** 2 < 0.02 ** 2):
            continue
        rows.append((kind, *pose))
        previous = current
    connection.executemany(
        "INSERT INTO trajectory VALUES(?,?,?,?,?,?,?,?,?)", rows)
    connection.commit()
    return len(rows)


def nearest_process_record(database: Path, process_log: Path,
                           stamp_ns: int) -> dict | None:
    with sqlite3.connect(database) as connection:
        row = connection.execute(
            "SELECT file_offset,byte_length FROM process_index "
            "WHERE stamp_ns<=? ORDER BY stamp_ns DESC,file_offset DESC LIMIT 1",
            (stamp_ns,)).fetchone()
    if row is None:
        return None
    with process_log.open("rb") as stream:
        stream.seek(row[0])
        return json.loads(stream.read(row[1]))


def write_manifest(cache_dir: Path, fingerprint: str, identity: dict,
                   process_log: Path, counts: dict) -> Path:
    manifest = {
        "schema": SCHEMA, "complete": True, "fingerprint": fingerprint,
        "identity": identity, "process_log": str(process_log.resolve()),
        "process_log_signature": file_edge_signature(process_log),
        "database": "index.sqlite3", "counts": counts,
    }
    path = cache_dir / "manifest.json"
    temporary = cache_dir / "manifest.json.tmp"
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    temporary.replace(path)
    return path


def load_valid_manifest(cache_root: Path, fingerprint: str) -> dict | None:
    path = cache_root / fingerprint / "manifest.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
        process_log = Path(manifest["process_log"])
        database = path.parent / manifest["database"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        return None
    if (manifest.get("schema") != SCHEMA or not manifest.get("complete") or
            manifest.get("fingerprint") != fingerprint or
            not process_log.is_file() or not database.is_file() or
            file_edge_signature(process_log) != manifest.get("process_log_signature")):
        return None
    manifest["manifest_path"] = str(path)
    manifest["database_path"] = str(database)
    return manifest
