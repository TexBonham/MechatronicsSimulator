"""REC invariant tier: no run archive in git (REC-021). REC-022 lives in `test_run_twice_determinism.py`."""

import subprocess
from pathlib import Path

import pytest

REPOSITORY = Path(__file__).resolve().parents[2]
ARCHIVE_SUFFIXES = (".h5", ".hdf5")


def _git(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["git", "-C", str(REPOSITORY), *arguments], capture_output=True, text=True, check=False)


@pytest.mark.req("REC-021")
def test_rec_021() -> None:
    """No run archive shall be tracked in git: none is tracked, and the ignore rules keep new ones out."""
    listing = _git("ls-files")
    assert listing.returncode == 0, listing.stderr
    tracked = [name for name in listing.stdout.splitlines() if name.lower().endswith(ARCHIVE_SUFFIXES)]
    assert tracked == []
    for name in ("run.h5", "runs/nested/run.hdf5"):
        assert _git("check-ignore", "--no-index", "-q", name).returncode == 0, name
