#!/usr/bin/env bash
# Fetch carla-content without the World Partition actor files.
#
# The repo tracks 513,074 LFS files; 468,594 of them (91%) live under
# __ExternalActors__/ and __ExternalObjects__/ - per-actor World Partition data
# for CARLA's OWN towns. We are loading the City Sample map instead, so none of
# it is needed. Excluding it leaves 44,383 files / 46 GB:
#
#     Static          42,563 files   33.35 GB   meshes, textures, vehicles
#     Maps               853 files   10.63 GB   CARLA's own towns
#     HDMaps               8 files    1.51 GB
#     HoudiniEngine      169 files    0.37 GB
#     Blueprints         682 files    0.07 GB   CarlaGameMode, vehicles, sensors
#     Animations          72 files    0.04 GB
#
# Only Blueprints and Static are strictly needed to run the City Sample as a
# CARLA server; Maps/HDMaps are kept so CARLA's own towns still work, which
# makes it possible to A/B the two. Add them to the exclusions below to save
# 12 GB if you never want them.
#
# Two separate exclusions are required and they are NOT the same thing:
#   sparse-checkout    stops git writing those paths to the working tree
#   lfs.fetchexclude   stops git-lfs DOWNLOADING them in the first place
# Without the second, lfs still pulls all 513k objects and fills the disk.
set -u
set -e
trap 'echo "FAILED at line $LINENO" >&2' ERR

# Everything is overridable from the environment so this is not tied to one
# machine. CARLA_ROOT is the only thing most people need to set.
#
#   CARLA_ROOT=/d/src/carla LFS_CACHE=/d/lfs bash tools/fetch-carla-content.sh
#
# LFS_CACHE matters on a small system drive: git-lfs keeps a full second copy of
# every object it downloads, so point it at a roomy volume.
# CARLA_DIR is what the PowerShell scripts and the README use; accept both so
# one exported variable covers the whole pipeline.
CARLA_ROOT="${CARLA_ROOT:-${CARLA_DIR:-/c/carla-ue58/carla}}"
DEST="${DEST:-$CARLA_ROOT/Unreal/CarlaUnreal/Content/Carla}"
URL="${CONTENT_URL:-https://bitbucket.org/carla-simulator/carla-content.git}"
BRANCH="${CONTENT_BRANCH:-ue58-dev-carla}"
CACHE="${LFS_CACHE:-$CARLA_ROOT/.lfs-cache}"
EXCLUDE="__ExternalActors__/**,__ExternalObjects__/**"

mkdir -p "$(dirname "$DEST")"

if [ ! -d "$DEST/.git" ]; then
  echo "=== 1/4 clone (metadata only, LFS deferred) ==="
  # SKIP_SMUDGE: check out LFS pointer files, download nothing yet.
  GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --no-checkout --sparse \
      -b "$BRANCH" "$URL" "$DEST" || exit 1
else
  echo "=== 1/4 clone: already present, reusing ==="
fi

cd "$DEST" || exit 1

echo "=== 2/4 config ==="
git config lfs.storage            "$CACHE"
git config lfs.concurrenttransfers 16
git config lfs.fetchexclude       "$EXCLUDE"
git config core.longpaths         true
echo "  lfs.storage      = $(git config lfs.storage)"
echo "  lfs.fetchexclude = $(git config lfs.fetchexclude)"

echo "=== 3/4 sparse checkout ==="
# Write the pattern file directly instead of "git sparse-checkout set".
# Under Git Bash / MSYS, any argument that looks like an absolute POSIX path is
# rewritten to a Windows one, so '!/__ExternalActors__/' silently becomes
# '!C:/Program Files/Git/__ExternalActors__/' and excludes nothing. Writing the
# file is immune to that.
git sparse-checkout init --no-cone || exit 1
cat > .git/info/sparse-checkout <<'EOP'
/*
!/__ExternalActors__/
!/__ExternalObjects__/
EOP
echo "  patterns:"; sed 's/^/    /' .git/info/sparse-checkout
git sparse-checkout reapply || exit 1
GIT_LFS_SKIP_SMUDGE=1 git checkout "$BRANCH" || exit 1
echo "  pointer files on disk: $(find . -type f -not -path './.git/*' | wc -l)"

echo "=== 4/4 lfs pull ==="
git lfs pull || exit 1

echo
echo "DONE  files: $(find . -type f -not -path './.git/*' | wc -l)"
du -sh . 2>/dev/null | tail -1
