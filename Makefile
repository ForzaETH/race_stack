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
	@awk '/^    [^ ].*:$$/ {name = $$1; sub(/:$$/, "", name)} /url:/ {url = $$2} /version:/ {print name, url, $$2}' \
		.install_utils/dependencies.repos | while read -r name url version; do \
		if [ -d "$(DEPS_DIR)/$$name" ]; then echo "$$name: already cloned, skipped"; continue; fi; \
		echo "Cloning $$name ($$version)..."; \
		git clone --quiet "$$url" "$(DEPS_DIR)/$$name" \
			&& git -C "$(DEPS_DIR)/$$name" checkout --quiet "$$version" \
			&& git -C "$(DEPS_DIR)/$$name" submodule update --init --recursive --quiet \
			|| exit 1; \
	done
	@# f1tenth_gym is a plain Python library that colcon cannot build (package_dir layout); it is
	@# pip-installed in the container instead, so hide it from colcon (and from its own git status)
	@touch $(DEPS_DIR)/f1tenth_gym/COLCON_IGNORE
	@grep -qx COLCON_IGNORE $(DEPS_DIR)/f1tenth_gym/.git/info/exclude 2>/dev/null \
		|| echo COLCON_IGNORE >> $(DEPS_DIR)/f1tenth_gym/.git/info/exclude

setup: deps
	@mkdir -p $(CACHE_DIR)/build $(CACHE_DIR)/install $(CACHE_DIR)/log
	@mkdir -p $(MAPS_DIR)

	@echo "Exporting environment variables to .env..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter RACECAR_VERSION [NUC1]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-NUC1}" >> .env
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
