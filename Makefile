-include .env

export HOST_UID HOST_GID

CONTAINER := forzaeth_devcontainer_$(HOST_UID)
LOCAL_COMPOSE := .devcontainer/docker-compose.local.yaml
XAUTH_DIR := $(HOME)/.docker_xauth

.PHONY: setup prepare launch rebuild x11 help

help:
	@echo "make setup    Prepare the host folders and .env (once per machine)"
	@echo "make launch   Build the images, start the containers and open a shell"
	@echo "make rebuild  Same as launch, with a fresh container"
	@echo "make x11      Give the container this session's display (e.g. after ssh -X)"

setup:
	@mkdir -p ../race_stack_maps ../race_stack_bags ../race_stack_deps

	@echo "Exporting environment variables to .env..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter RACECAR_VERSION (SIM for laptops, NUCx for cars) [SIM]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-SIM}" >> .env
	@printf "Does this machine run VIO with a RealSense camera? (true/false) [false]: " && read use_vio && echo "USE_VIO=$${use_vio:-false}" >> .env && \
	if [ "$${use_vio:-false}" = "true" ]; then \
		printf "Which RealSense camera model? (d435/d455) [d435]: " && read camera_model && echo "VIO_CAMERA_MODEL=$${camera_model:-d435}" >> .env; \
	fi
	@echo "HOST_UID=$$(id -u)" >> .env
	@echo "HOST_GID=$$(id -g)" >> .env
	@# Per-user ports
	@echo "FOXGLOVE_PORT=$$((8765 + $$(id -u) % 1000))" >> .env
	@echo "NOVNC_X11_PORT=$$((6000 + $$(id -u)))" >> .env
	@echo "NOVNC_WEB_PORT=$$((20000 + $$(id -u)))" >> .env

	@echo "Selecting base image..."
	@if [ -f /etc/nv_tegra_release ]; then \
		echo "Jetson detected"; \
		echo "BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0" >> .env; \
		echo "DOCKER_RUNTIME=nvidia" >> .env; \
	else \
		echo "BASE_IMAGE=ros:humble-ros-base" >> .env; \
	fi

	@echo "====================================================================================="
	@echo "Setup complete! Open VS Code, type CTRL + SHIFT + P and select 'Reopen in Container'."
	@echo "====================================================================================="

# This is also used by VSCode
prepare: x11
	@[ -f $(LOCAL_COMPOSE) ] || cp .devcontainer/docker-compose.local.template.yaml $(LOCAL_COMPOSE)
	@# Build when online, else keep using the existing images
	@docker compose -f docker-compose.yaml -f .devcontainer/docker-compose.build.yaml build \
		|| echo "WARNING: image build failed (no internet?), using the existing images"
	@# Zenoh router (shared by users)
	@docker compose -p race_stack_shared up -d --no-recreate zenoh_router

launch: prepare
	@docker compose -f docker-compose.yaml -f $(LOCAL_COMPOSE) up -d $(UP_FLAGS)
	@# Once per new container, like VS Code; the marker is in the container, so it is gone when that is recreated
	@docker exec $(CONTAINER) test -f /tmp/.post_create_done \
		|| docker exec -it $(CONTAINER) bash /ws/src/race_stack/.install_utils/post_create_command.sh
	@docker exec -it $(CONTAINER) bash

build: UP_FLAGS := --force-recreate
build: launch

# Display access for GUI applications, both locally (xhost) and over ssh -X (xauth)
x11:
	@mkdir -p $(XAUTH_DIR) && touch $(XAUTH_DIR)/xauth
	@if [ "$$(uname -s)" = Linux ] && [ -n "$$DISPLAY" ] && command -v xhost >/dev/null; then \
		xhost +SI:localuser:$$(id -un) >/dev/null 2>&1 || true; \
	fi
	@if [ -n "$$DISPLAY" ] && command -v xauth >/dev/null; then \
		xauth nlist $$DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $(XAUTH_DIR)/xauth nmerge - 2>/dev/null || true; \
	fi
	@# Sessions without a display (MacOS, ssh without X forwarding) use the noVNC display
	@if [ "$$(uname -s)" = Linux ] && [ -n "$$DISPLAY" ]; then echo "$$DISPLAY"; else echo ":$(HOST_UID)"; fi > $(XAUTH_DIR)/display
	@echo "Container display: $$(cat $(XAUTH_DIR)/display) (in already open terminals: export DISPLAY=$$(cat $(XAUTH_DIR)/display))"
