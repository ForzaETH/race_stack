#!/bin/bash

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Needed if there is a stale image before the persistance changes
IMAGE="race_stack:humble_${USER}"
if ! docker run --rm --pull never --entrypoint test "$IMAGE" -d "/home/$USER/.vscode-server" 2>/dev/null; then
    echo "Image $IMAGE is missing or predates the persistence volumes, rebuilding..."
    make -C "$REPO_DIR" build || exit 1
fi

# Personal compose overrides
cp -n "$REPO_DIR/.devcontainer/docker-compose.local.template.yaml" \
      "$REPO_DIR/.devcontainer/docker-compose.local.yaml"

bash "$REPO_DIR/.devcontainer/xauth_setup.sh"


