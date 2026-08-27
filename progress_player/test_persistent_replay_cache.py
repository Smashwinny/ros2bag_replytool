#!/usr/bin/env python3
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

from persistent_replay_cache import (
    cache_fingerprint, create_database, index_process_log, insert_trajectory,
    load_valid_manifest, nearest_process_record, reset_build_database,
    write_manifest)


class PersistentReplayCacheTest(unittest.TestCase):
    def test_reset_build_database_removes_sqlite_sidecars(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            database = Path(directory) / "index.sqlite3.building"
            sidecars = [database, Path(f"{database}-wal"),
                        Path(f"{database}-shm")]
            for path in sidecars:
                path.write_bytes(b"stale")
            reset_build_database(database)
            self.assertFalse(any(path.exists() for path in sidecars))

    def test_manifest_binds_bag_target_build_and_process_log(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory)
            bag, cache = root / "bag", root / "cache"
            bag.mkdir()
            (bag / "metadata.yaml").write_text("metadata-v1", encoding="utf-8")
            (bag / "data.db3").write_bytes(b"bag-v1")
            target, build, process = root / "target.yaml", root / "build.json", root / "log"
            target.write_text("target-v1", encoding="utf-8")
            build.write_text('{"artifact_sha256":"a"}', encoding="utf-8")
            process.write_text("sealed-log", encoding="utf-8")
            fingerprint, identity = cache_fingerprint(bag, target, build)
            cache_dir = cache / fingerprint
            cache_dir.mkdir(parents=True)
            database = cache_dir / "index.sqlite3"
            create_database(database).close()
            write_manifest(cache_dir, fingerprint, identity, process, {})
            self.assertIsNotNone(load_valid_manifest(cache, fingerprint))
            process.write_text("changed-log", encoding="utf-8")
            self.assertIsNone(load_valid_manifest(cache, fingerprint))
            target.write_text("target-v2", encoding="utf-8")
            self.assertNotEqual(cache_fingerprint(bag, target, build)[0],
                                fingerprint)

    def test_indexes_all_process_records_and_trajectories(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory)
            log, database = root / "process.jsonl", root / "index.sqlite3"
            records = [
                {"event": "diagnostic", "source_time_s": 10.0,
                 "sequence": 1, "value": "all-process-data"},
                {"event": "pose_published", "source_time_s": 10.5,
                 "sequence": 2, "filter_anchored": True,
                 "published_position_wb": [1, 2, 0],
                 "published_orientation_wb_wxyz": [1, 0, 0, 0]},
            ]
            log.write_text("".join(json.dumps(item) + "\n" for item in records),
                           encoding="utf-8")
            connection = create_database(database)
            self.assertEqual(index_process_log(connection, log), (2, 1))
            self.assertEqual(insert_trajectory(connection, "legacy", [
                (10_000_000_000, 3, 4, 0, 1, 0, 0, 0)]), 1)
            connection.close()
            self.assertEqual(nearest_process_record(
                database, log, 10_100_000_000)["value"], "all-process-data")
            with sqlite3.connect(database) as check:
                self.assertEqual(check.execute(
                    "SELECT count(*) FROM trajectory").fetchone()[0], 2)


if __name__ == "__main__":
    unittest.main()
