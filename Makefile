# Makefile for setting up ROS Noetic build cache and cloning repos
# NOTE: this is not a traditional Makefile, but a script-like structure
# Usage: make [target]

# Configurable variables (edit as needed)
CACHE_DIR := ../race_stack_cache/humble

# Phony targets
.PHONY: \
	help \
	default_setup \
	setup_cache \
	build \
	launch\
	export_env

help: ## Show available targets and their descriptions
	@echo "\n\033[33mHelper Makefile for the ForzaETH race_stack setup.\033[0m"
	@echo "\nUsage:\n\tmake [target]"
	@echo "\nAvailable [target]s:"
	@grep -E '^[a-zA-Z_-]+:.*?## ' Makefile | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

# HIGH-LEVEL TARGETS
default_setup: export_env edit-json-arm check-registry setup_cache build ## Setup the race_stack on a new computer, with the simulator
# LEAF TARGETS
export_env:
	@echo "Exporting environment variables to .env file..."
	@printf "Enter ROS_DOMAIN_ID [48]: " && read domain_id && echo "ROS_DOMAIN_ID=$${domain_id:-48}" > .env
	@printf "Enter racecar_version [NUC5]: " && read racecar_version && echo "RACECAR_VERSION=$${racecar_version:-NUC5}" >> .env
	@echo "HOST_UID=$(shell id -u)" >> .env
	@echo "HOST_GID=$(shell id -g)" >> .env
setup_cache: ## Create cache folder structure
	@echo "Creating ros cache directories..."
	mkdir -p $(CACHE_DIR)/build \
		$(CACHE_DIR)/install \
		$(CACHE_DIR)/log
	@echo "Cache directories ready at $(CACHE_DIR)"


build: ## Build the base workspace
	@echo "Detecting system architecture..."
	@if [ "$$(uname -m)" = "x86_64" ]; then \
		echo "Building for x86 architecture..."; \
		docker compose build x86; \
	elif [ "$$(uname -m)" = "aarch64" ]; then \
		echo "Building for Jetson ARM architecture..."; \
		docker compose build jet; \
	elif [ "$$(uname -m)" = "arm64" ]; then \
		echo "Building for Apple ARM architecture..."; \
		docker compose build arm; \
	else \
		echo "Unsupported architecture: $$(uname -m)"; \
		exit 1; \
	fi
	@echo "Base workspace built successfully."

launch:
	@echo "Detecting system architecture..."
	@if [ "$$(uname -m)" = "x86_64" ]; then \
		export SERVICE_NAME="x86"; \
	elif [ "$$(uname -m)" = "aarch64" ]; then \
		export SERVICE_NAME="jet"; \
	elif [ "$$(uname -m)" = "arm64" ]; then \
		export SERVICE_NAME="arm"; \
		export DISPLAY=:501; \
		echo "Setting DISPLAY to :501 for Apple Silicon..."; \
	else \
		echo "Unsupported architecture: $$(uname -m)"; \
		exit 1; \
	fi; \
	.devcontainer/launch_helper.sh $$SERVICE_NAME

edit-json-arm:
	@echo "If on an ARM based device, please edit devcontainer.json as follows"
	@echo "Edit the service and runServices to end with arm and not x86"
	@echo "Change display from '\$${localEnv:DISPLAY}' to ':501'"
	@printf "Have you updated it if needed?  (y/N): " && read ans && [ "$$ans" = "y" -o "$$ans" = "Y" ] || (echo "Aborted."; exit 1)
	@echo "✅ Proceeding with build..."

edit-json-jetson:
	@echo "If on an jetson, please edit devcontainer.json as follows"
	@echo "Edit the image to end with ..jet and not ..x86"
	@printf "Have you updated it if needed?  (y/N): " && read ans && [ "$$ans" = "y" -o "$$ans" = "Y" ] || (echo "Aborted."; exit 1)
	@echo "✅ Proceeding with build..."

.DEFAULT: ## Handle unknown commands
	@echo "Unknown command: $@"
	@echo "Run 'make help' or 'make' to see available commands."
