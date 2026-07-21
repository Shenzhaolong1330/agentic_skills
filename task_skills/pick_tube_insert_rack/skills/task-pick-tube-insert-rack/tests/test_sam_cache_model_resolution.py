from __future__ import annotations

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
import sam_cache_service  # noqa: E402


def test_existing_local_model_path_does_not_call_hugging_face(
    tmp_path: Path,
    monkeypatch,
) -> None:
    model_dir = tmp_path / "sam-model"
    model_dir.mkdir()

    def unexpected_download(**_kwargs):
        raise AssertionError("snapshot_download must not run for a local model path")

    monkeypatch.setattr(
        "huggingface_hub.snapshot_download",
        unexpected_download,
    )

    resolved = sam_cache_service.resolve_sam_model_source(
        str(model_dir),
        allow_model_download=False,
    )

    assert resolved == str(model_dir.resolve())


def test_hub_model_resolution_is_local_only_by_default(
    tmp_path: Path,
    monkeypatch,
) -> None:
    snapshot = tmp_path / "snapshot"
    snapshot.mkdir()
    calls: list[dict] = []

    def fake_snapshot_download(**kwargs):
        calls.append(kwargs)
        return str(snapshot)

    monkeypatch.setattr(
        "huggingface_hub.snapshot_download",
        fake_snapshot_download,
    )

    resolved = sam_cache_service.resolve_sam_model_source(
        "facebook/sam-vit-base",
        allow_model_download=False,
    )

    assert resolved == str(snapshot.resolve())
    assert calls == [
        {
            "repo_id": "facebook/sam-vit-base",
            "local_files_only": True,
        }
    ]
