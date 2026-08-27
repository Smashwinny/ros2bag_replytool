#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest

from build_artifact_guard import (
    SCHEMA, artifacts_valid, hash_paths, load_state, write_state)


class BuildArtifactGuardTest(unittest.TestCase):
    def test_unchanged_source_and_artifact_reuse(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory)
            source, artifact, state = (
                root / "source.cpp", root / "node", root / "state.json")
            source.write_text("source-v1", encoding="utf-8")
            artifact.write_text("binary-v1", encoding="utf-8")
            source_hash = hash_paths([source])
            write_state(state, source_hash, [artifact])
            self.assertTrue(artifacts_valid(
                load_state(state), source_hash, [artifact]))

    def test_source_or_artifact_change_invalidates(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory)
            source, artifact, state = (
                root / "source.cpp", root / "node", root / "state.json")
            source.write_text("source-v1", encoding="utf-8")
            artifact.write_text("binary-v1", encoding="utf-8")
            source_hash = hash_paths([source])
            write_state(state, source_hash, [artifact])
            source.write_text("source-v2", encoding="utf-8")
            self.assertFalse(artifacts_valid(
                load_state(state), hash_paths([source]), [artifact]))
            source.write_text("source-v1", encoding="utf-8")
            artifact.write_text("binary-v2", encoding="utf-8")
            self.assertFalse(artifacts_valid(
                load_state(state), source_hash, [artifact]))

    def test_invalid_state_fails_closed(self):
        with tempfile.TemporaryDirectory(
                dir=Path(__file__).parents[1] / "tmp") as directory:
            state = Path(directory) / "state.json"
            state.write_text(json.dumps({"schema": SCHEMA + "-old"}),
                             encoding="utf-8")
            self.assertIsNone(load_state(state))


if __name__ == "__main__":
    unittest.main()
