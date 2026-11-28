#!/bin/bash

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Personal compose overrides
cp -n "$REPO_DIR/.devcontainer/docker-compose.local.template.yaml" \
      "$REPO_DIR/.devcontainer/docker-compose.local.yaml"

bash "$REPO_DIR/.devcontainer/xauth_setup.sh"


