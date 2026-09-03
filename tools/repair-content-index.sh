#!/usr/bin/env bash
# Repair the carla-content index and materialise the LFS files.
#
# Needed only if a "git checkout" in this repo was interrupted: the index is
# then left with every entry staged as deleted, and "git lfs pull" quite
# correctly does nothing because git believes there are no files to check out.
set -u
CARLA_ROOT="${CARLA_ROOT:-/c/carla-ue58/carla}"
C="${DEST:-$CARLA_ROOT/Unreal/CarlaUnreal/Content/Carla}"
cd "$C" || exit 1

echo "=== 1/3 reset index to HEAD (working tree untouched) ==="
git reset --mixed HEAD || exit 1

echo "=== 2/3 reapply sparse patterns ==="
sed 's/^/    /' .git/info/sparse-checkout
git sparse-checkout reapply || exit 1

echo "=== 3/3 lfs pull ==="
git lfs pull || exit 1

echo
echo "DONE"
echo "  files    : $(find . -type f -not -path './.git/*' | wc -l)"
echo "  pointers : $(grep -rl --include='*.uasset' -m1 'git-lfs.github.com/spec' . 2>/dev/null | wc -l) still unresolved"
