#!/usr/bin/env bash
# Repair standalone git after removing wbr_mujoco submodule link.
# Preserves working-tree files; only resets git metadata to match origin.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ORIGIN="${MUJOCO_INTERFACE_ORIGIN:-git@github.com:CosmosMount/mujoco_interface.git}"
PREFERRED_BRANCH="${MUJOCO_INTERFACE_BRANCH:-}"

echo "==> mujoco_interface git repair (root: $ROOT)"

if [ ! -d .git ]; then
  git init
fi

# Drop stale submodule remotes / wrong names
git remote remove upstream 2>/dev/null || true
if git remote get-url origin &>/dev/null; then
  git remote set-url origin "$ORIGIN"
else
  git remote add origin "$ORIGIN"
fi

echo "==> fetch origin"
git fetch origin

pick_branch() {
  if [ -n "$PREFERRED_BRANCH" ] && git show-ref --verify --quiet "refs/remotes/origin/$PREFERRED_BRANCH"; then
    echo "$PREFERRED_BRANCH"
    return
  fi
  if git show-ref --verify --quiet refs/remotes/origin/architecture; then
    echo architecture
    return
  fi
  if git show-ref --verify --quiet refs/remotes/origin/main; then
    echo main
    return
  fi
  git remote show origin | sed -n 's/.*HEAD branch: //p' | head -1
}

BR="$(pick_branch)"
if [ -z "$BR" ]; then
  echo "error: could not determine default branch from origin" >&2
  exit 1
fi

echo "==> checkout $BR (track origin/$BR, keep working tree)"
git checkout -B "$BR" "origin/$BR"
git branch --set-upstream-to="origin/$BR" "$BR"

echo
echo "==> done"
git status -sb
echo "---"
git remote -v
echo "---"
git branch -vv
