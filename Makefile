MAPS_DIR := ../race_stack_maps
BAGS_DIR := ../race_stack_bags
DEPS_DIR := ../race_stack_deps

CONTAINER := forzaeth_devcontainer_$(shell id -u)
LOCAL_COMPOSE := .devcontainer/docker-compose.local.yaml
COMPOSE := docker compose -f docker-compose.yaml -f $(LOCAL_COMPOSE)

XAUTH_DIR := $(HOME)/.docker_xauth

.PHONY: setup launch x11 x11-refresh help

help:
	@echo "Usage: make setup"
	@echo "Prepares the host folders and environment variables for the VS Code DevContainer setup."
	@echo "       make launch"
	@echo "Starts the containers without VS Code and opens a shell (runs the post-create setup on first start)."
	@echo "       make x11"
	@echo "Run from an ssh -X terminal: gives the running container access to this session's display."

setup:
	@mkdir -p $(MAPS_DIR) $(BAGS_DIR) $(DEPS_DIR)

	@echo "Exporting environment variables to .env..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter RACECAR_VERSION (SIM for laptops, NUCx for cars) [SIM]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-SIM}" >> .env
	@printf "Does this machine run VIO with a RealSense camera? (true/false) [false]: " && read use_vio && echo "USE_VIO=$${use_vio:-false}" >> .env
	@echo "HOST_UID=$$(id -u)" >> .env
	@echo "HOST_GID=$$(id -g)" >> .env
	@echo "COMPOSE_BAKE=false" >> .env

	@echo "Selecting base image..."
	@if [ -f /etc/nv_tegra_release ]; then \
		l4t=$$(sed -n 's/^# R\([0-9]*\) (release), REVISION: \([0-9.]*\).*/r\1.\2/p' /etc/nv_tegra_release); \
		echo "Jetson detected"; \
		echo "BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0" >> .env; \
		echo "DOCKER_RUNTIME=nvidia" >> .env; \
	else \
		echo "BASE_IMAGE=ros:humble-ros-base" >> .env; \
	fi

	@echo "========================================"
	@echo "Setup complete! Open VS Code, type CTRL + SHIFT + P and select 'Reopen in Container'."
	@echo "========================================"

launch: x11-refresh
	@[ -f .env ] || { echo "No .env found, run 'make setup' first."; exit 1; }
	@[ -f $(LOCAL_COMPOSE) ] || cp .devcontainer/docker-compose.local.template.yaml $(LOCAL_COMPOSE)
	@$(COMPOSE) up -d
	@docker exec $(CONTAINER) test -f /ws/install/setup.bash \
		|| docker exec -it $(CONTAINER) bash /ws/src/race_stack/.install_utils/post_create_command.sh
	@docker exec -it $(CONTAINER) bash


# For X11 forwarding for GUI applications, both locally (xhost) and remotely (xauth)
x11:
	@[ -n "$$DISPLAY" ] || { echo "No DISPLAY set, connect with ssh -X first."; exit 1; }
	@$(MAKE) -s x11-refresh
	@echo "Display access updated to $$DISPLAY. New container terminals use it automatically;"
	@echo "in already open ones run: export DISPLAY=$$DISPLAY"

x11-refresh:
	@mkdir -p $(XAUTH_DIR) && touch $(XAUTH_DIR)/xauth
	@if [ "$$(uname -s)" = Linux ] && [ -n "$$DISPLAY" ] && command -v xhost >/dev/null; then \
		xhost +SI:localuser:$$(id -un) >/dev/null 2>&1 || true; \
	fi
	@if [ -n "$$DISPLAY" ] && command -v xauth >/dev/null; then \
		xauth nlist $$DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $(XAUTH_DIR)/xauth nmerge - 2>/dev/null || true; \
	fi
	@# Sessions without a display (MacOS, ssh without X forwarding) use the noVNC display
	@if [ "$$(uname -s)" = Linux ] && [ -n "$$DISPLAY" ]; then echo "$$DISPLAY"; else echo ":501"; fi > $(XAUTH_DIR)/display
