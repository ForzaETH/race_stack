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
	@command -v vcs >/dev/null
	@mkdir -p $(DEPS_DIR)
	@vcs import --input .install_utils/dependencies.repos --skip-existing --recursive $(DEPS_DIR)

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
