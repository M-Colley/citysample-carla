#!/usr/bin/env bash
# Fetch carla-content without the World Partition actor files.
#
# The 1.0 tag has 45,019 LFS files (49.66 GB) after excluding CARLA's own
# World Partition actors and objects. The LFS cache stores another 49.61 GB
# when it is empty. LFS_CACHE can point to a separate drive.
#
# Two separate exclusions are required and they are NOT the same thing:
#   sparse-checkout    stops git writing those paths to the working tree
#   lfs.fetchexclude   stops git-lfs DOWNLOADING them in the first place
# Both exclusions are needed to omit the unused assets from the download.
set -euo pipefail
trap 'echo "FAILED at line $LINENO" >&2' ERR

# Everything is overridable from the environment so this is not tied to one
# machine. CARLA_ROOT is the only thing most people need to set.
#
#   CARLA_ROOT=/d/src/carla LFS_CACHE=/d/lfs bash tools/fetch-carla-content.sh
#
# LFS_CACHE matters on a small system drive: git-lfs keeps a full second copy of
# every object it downloads, so point it at a roomy volume.
# CARLA_DIR is what the PowerShell scripts and the README use; prefer it over
# CARLA_ROOT, which CARLA's binary package documentation often points at a
# non-Git shipping install.
CARLA_ROOT=$(cd "${CARLA_DIR:-${CARLA_ROOT:-/c/carla-ue58/carla}}" && pwd -P)
DEST="${DEST:-$CARLA_ROOT/Unreal/CarlaUnreal/Content/Carla}"
URL="${CONTENT_URL:-https://bitbucket.org/carla-simulator/carla-content.git}"
CACHE="${LFS_CACHE:-$CARLA_ROOT/.lfs-cache}"
EXCLUDE="__ExternalActors__/**,__ExternalObjects__/**"
CARLA_1_0_REV=3ea166118d2c3c094b13b15c5b7a66a9a279998d
CONTENT_1_0_REV=55228c1ccca1b698e5ad8c638c576d523bcdebb9

case "${1:-}" in
  ''|--check) ;;
  *) echo "usage: $0 [--check]" >&2; exit 2 ;;
esac

source_rev=$(git -C "$CARLA_ROOT" rev-parse --verify HEAD)
branch="${CONTENT_BRANCH:-ue58-dev-carla}"
checkout="$branch"
if [[ "$source_rev" == "$CARLA_1_0_REV" ]]; then
  branch=1.0
  checkout="$CONTENT_1_0_REV"
fi

check_release_revision() {
  if [[ "$source_rev" != "$CARLA_1_0_REV" ]]; then
    return
  fi
  local current
  if [[ ! -e "$DEST/.git" ]] || ! current=$(git -C "$DEST" rev-parse --verify HEAD); then
    echo "CARLA 1.0 content checkout missing at $DEST. Run $0." >&2
    exit 1
  fi
  if [[ "$current" != "$CONTENT_1_0_REV" ]]; then
    echo "CARLA 1.0 needs content $CONTENT_1_0_REV. Found $current at $DEST." >&2
    echo "Use a separate CARLA checkout or move the existing content aside before fetching." >&2
    exit 1
  fi
}

check_game_mode() {
  local game_mode="$DEST/Blueprints/Game/CarlaGameMode.uasset"
  if [[ ! -s "$game_mode" ]] || LC_ALL=C grep -aq '^version https://git-lfs.github.com/spec/v1$' "$game_mode"; then
    echo "CARLA game mode asset missing or unresolved at $game_mode. Run $0." >&2
    exit 1
  fi
}

if [[ "${1:-}" == '--check' ]]; then
  check_release_revision
  check_game_mode
  exit 0
fi

mkdir -p "$(dirname "$DEST")"

if [[ ! -e "$DEST/.git" ]]; then
  echo "=== 1/4 clone (metadata only, LFS deferred) ==="
  GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --no-checkout \
      -b "$branch" "$URL" "$DEST"
else
  echo "=== 1/4 clone: already present, reusing ==="
fi
check_release_revision

DEST=$(cd "$DEST" && pwd -P)
cd "$DEST"

echo "=== 2/4 config ==="
git config lfs.storage            "$CACHE"
git config lfs.concurrenttransfers 16
git config lfs.fetchexclude       "$EXCLUDE"
git config core.longpaths         true
echo "  lfs.storage      = $(git config lfs.storage)"
echo "  lfs.fetchexclude = $(git config lfs.fetchexclude)"

echo "=== 3/4 sparse checkout ==="
# Read patterns on stdin so Git Bash passes the leading slashes literally.
GIT_LFS_SKIP_SMUDGE=1 git sparse-checkout set --no-cone --stdin <<'EOP'
/*
!/__ExternalActors__/
!/__ExternalObjects__/
EOP
GIT_LFS_SKIP_SMUDGE=1 git checkout "$checkout"

echo "=== 4/4 lfs pull ==="
git lfs pull
check_game_mode

echo "CARLA content ready at $DEST."
