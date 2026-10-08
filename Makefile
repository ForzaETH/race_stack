CACHE_DIR := ../race_stack_cache/humble
MAPS_DIR := ../race_stack_maps
DEPS_DIR := ../race_stack_deps

.PHONY: setup deps help

help:
	@echo "Usage: make setup"
	@echo "Prepares the cache and environnment variables for a VsCode DevContainer setup."
	@echo "       make deps"
	@echo "Clones the external repositories into $(DEPS_DIR) (mounted as /ws/src in the container)."

deps:
	@mkdir -p $(DEPS_DIR)
	@while read -r name url version; do \
		[ -d $(DEPS_DIR)/$$name ] || { \
			git clone -q $$url $(DEPS_DIR)/$$name && \
			git -C $(DEPS_DIR)/$$name checkout -q $$version && \
			git -C $(DEPS_DIR)/$$name submodule update -q --init --recursive; \
		} || exit 1; \
	done < .install_utils/dependencies.txt

setup: deps
	@mkdir -p $(CACHE_DIR)/build $(CACHE_DIR)/install $(CACHE_DIR)/log
	@mkdir -p $(MAPS_DIR)

	@echo "Exporting environment variables to .env..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter RACECAR_VERSION (SIM for laptops, NUCx for cars)[SIM]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-SIM}" >> .env
	@echo "HOST_UID=$$(id -u)" >> .env
	@echo "HOST_GID=$$(id -g)" >> .env

	@echo "Configuring Display for X11 GUI Forwarding..."
	@if [ "$$(uname -s)" = "Darwin" ]; then \
		echo "DISPLAY=:501" >> .env; \
		echo "COMPOSE_PROFILES=novnc" >> .env; \
	else \
		echo "DISPLAY=$$DISPLAY" >> .env; \
	fi

	@echo "========================================"
	@echo "Setup complete! Open VS Code, type CTRL + SHIFT + P and select 'Reopen in Container'."
	@echo "========================================"
