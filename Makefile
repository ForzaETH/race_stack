# Makefile for setting up ROS Noetic build cache and cloning repos
# NOTE: this is not a traditional Makefile, but a script-like structure
# Usage: make [target]

# Phony targets
.PHONY: \
	help \
	default_setup \
	build \
	launch\
	export_env

help: ## Show available targets and their descriptions
	@echo "\n\033[33mHelper Makefile for the ForzaETH race_stack setup.\033[0m"
	@echo "\nUsage:\n\tmake [target]"
	@echo "\nAvailable [target]s:"
	@grep -E '^[a-zA-Z_-]+:.*?## ' Makefile | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

# HIGH-LEVEL TARGETS
default_setup: export_env build ## Setup the race_stack on a new computer, with the simulator
# LEAF TARGETS
export_env: ## Write the .env file (asks for settings, detects the platform)
	@echo "Exporting environment variables to .env file..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter racecar_version [NUC5]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-NUC5}" >> .env
	@printf "Use VIO (RealSense + Basalt)? [false]: " && read use_vio && echo "USE_VIO=$${use_vio:-false}" >> .env
	@echo "HOST_UID=$(shell id -u)" >> .env
	@echo "HOST_GID=$(shell id -g)" >> .env
	@if [ "$$(uname -s)" = "Darwin" ]; then echo "CONTAINER_DISPLAY=:1800" >> .env; fi
	@case "$$(uname -m)" in \
		x86_64) 	echo "PLATFORM=x86" >> .env ;; \
		arm64)		echo "PLATFORM=arm" >> .env ;; \
		aarch64) 	echo "PLATFORM=jet" >> .env; \
					echo "BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r36.3.0" >> .env; \
					echo "DOCKER_RUNTIME=nvidia" >> .env ;; \
		*) echo "Unsupported architecture: $$(uname -m)"; exit 1 ;; \
	esac

build: ## Build the base workspace
	@platform=$$(grep -s '^PLATFORM=' .env | cut -d= -f2); echo "Building for $${platform:-x86} platform..."
	@docker compose -f docker-compose.yaml -f .devcontainer/docker-compose.build.yaml build race_stack novnc
	@echo "Base workspace built successfully."

launch: ## Launch (or attach to) the race_stack container
	@.devcontainer/launch_helper.sh

.DEFAULT: ## Handle unknown commands
	@echo "Unknown command: $@"
	@echo "Run 'make help' or 'make' to see available commands."
