#!/usr/bin/env bash
# One C++ build on this machine at a time (round 4 rule): configure, build and test a preset under a machine-wide
# lock. Parallelism comes from the presets ("jobs": 2). Usage:
#   tools/locked_build.sh core debug [ctest args...]        # e.g. -R 'signals|stepper'
#   tools/locked_build.sh firmware host [ctest args...]
#   tools/locked_build.sh sync                              # uv sync (builds the extension) under the same lock
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lock="${MECHSIM_BUILD_LOCK:-/tmp/mechsim-build.lock}"
tree="${1:?tree: core, firmware or sync}"
shift || true
case "$tree" in
    core) dir="$root" ;;
    firmware) dir="$root/firmware" ;;
    sync)
        exec flock "$lock" bash -c "cd '$root' && uv sync"
        ;;
    *) echo "unknown tree: $tree" >&2; exit 2 ;;
esac
preset="${1:?preset name}"
shift || true
exec flock "$lock" bash -c "
    set -euo pipefail
    cd '$dir'
    if [ ! -f 'build/$preset/CMakeCache.txt' ]; then cmake --preset '$preset'; fi
    cmake --build --preset '$preset'
    ctest --preset '$preset' $*
"
