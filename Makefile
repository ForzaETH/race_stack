CACHE_DIR := ../race_stack_cache/humble

.PHONY: setup help

help:
	@echo "Usage: make setup"
	@echo "Prepares the cache and environnment variables for a VsCode DevContainer setup."

setup:
	@mkdir -p $(CACHE_DIR)/build $(CACHE_DIR)/install $(CACHE_DIR)/log

	@echo "Cloning external repositories..."
	@mkdir -p $(WORKSPACE_DIR)/src
	vcs import $(WORKSPACE_DIR) < .install_utils/dependencies.repos

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
