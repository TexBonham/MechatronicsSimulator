#!/usr/bin/env bash
# One C++ build on this machine at a time (round 4 rule): configure, build and test a preset under a machine-wide
# lock. The presets carry "jobs": 2 for CI; on this machine the script passes MECHSIM_JOBS (default 8, owner
# ruling 2026-11-12: the WSL runner has 12 cores to itself) to the build, ctest and the extension build. Usage:
#   tools/locked_build.sh core debug [ctest args...]        # e.g. -R 'signals|stepper'
#   tools/locked_build.sh firmware host [ctest args...]
#   tools/locked_build.sh sync                              # uv sync (builds the extension) under the same lock
#   tools/locked_build.sh run uv run pytest -q ...          # any command under the lock (uv run rebuilds the
#                                                           # extension after C++ edits, so the first uv run after
#                                                           # a C++ change goes through here)
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lock="${MECHSIM_BUILD_LOCK:-/tmp/mechsim-build.lock}"
jobs="${MECHSIM_JOBS:-8}"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
tree="${1:?tree: core, firmware or sync}"
shift || true
case "$tree" in
    core) dir="$root" ;;
    firmware) dir="$root/firmware" ;;
    sync)
        exec flock "$lock" bash -c "cd '$root' && uv sync"
        ;;
    run)
        cd "$root"
        exec flock "$lock" "$@"
        ;;
    *) echo "unknown tree: $tree" >&2; exit 2 ;;
esac
preset="${1:?preset name}"
shift || true
ctest_args="$(printf ' %q' "$@")"
exec flock "$lock" bash -c "
    set -euo pipefail
    cd '$dir'
    if [ ! -f 'build/$preset/CMakeCache.txt' ]; then cmake --preset '$preset'; fi
    cmake --build --preset '$preset' -j '$jobs'
    ctest --preset '$preset' -j '$jobs'$ctest_args
"
