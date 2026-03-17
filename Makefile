SHELL := /bin/bash
.SILENT:
.DEFAULT_GOAL := help

REPO_ROOT := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
ENV_SAMPLE := $(REPO_ROOT)/.env.sample
ENV_DST    := $(REPO_ROOT)/.env
FW_DIR     := $(REPO_ROOT)/firmware
FW_BIN     := $(FW_DIR)/.pio/build/esp32cam/firmware.bin
FW_SRC     := $(wildcard $(FW_DIR)/src/*.cpp $(FW_DIR)/src/*.h $(FW_DIR)/platformio.ini)

## Help
.PHONY: help
help: ## Show this help message
	echo "Available targets:"
	echo "=================="
	grep -E '(^[a-zA-Z_-]+:.*?## .*$$|^## )' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; \
		     /^## / {gsub("^## ", ""); print "\n\033[1;35m" $$0 "\033[0m"}; \
		     /^[a-zA-Z_-]+:/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

## Build & Flash
.PHONY: flash deploy

$(FW_BIN): $(FW_SRC)
	cd $(FW_DIR) && pio run

build: $(FW_BIN) ## Build firmware (skips if unchanged)

flash: $(FW_BIN) ## Flash firmware (builds first if needed, prompts for RST)
	uv run python $(REPO_ROOT)/ross/flash.py $(FW_BIN)

deploy: flash ## Build + flash in one step

## Teleop
.PHONY: teleop serial-teleop

teleop: $(FW_BIN) ## WiFi teleop — build, flash, video + keyboard control
	uv run python $(REPO_ROOT)/ross/flash.py $(FW_BIN)
	uv run $(REPO_ROOT)/ross/teleop.py $(ARGS)

serial-teleop: $(FW_BIN) ## Serial teleop — build, flash, keyboard motor control
	uv run python $(REPO_ROOT)/ross/flash.py $(FW_BIN)
	uv run $(REPO_ROOT)/ross/serial_teleop.py $(ARGS)

## Monitoring
.PHONY: serial fuel

serial: ## Monitor ESP32 serial output (exit: Ctrl-A k)
	screen /dev/ttyAMA0 115200

fuel: ## Monitor battery via fuel gauge
	uv run $(REPO_ROOT)/ross/fuel_gauge.py --watch $(ARGS)

## Development
.PHONY: clean sync lint lint-fix format

sync: ## Install Python dependencies
	uv sync

lint: ## Lint Python code
	uv run ruff check $(REPO_ROOT)/ross/

lint-fix: ## Lint and auto-fix Python code
	uv run ruff check --fix $(REPO_ROOT)/ross/

format: ## Format Python code
	uv run ruff format $(REPO_ROOT)/ross/

clean: ## Remove build artifacts
	rm -rf $(FW_DIR)/.pio
	find $(REPO_ROOT) -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true

## Setup
.PHONY: setup-env

setup-env: ## Generate .env interactively from .env.sample
	@if [ -f $(ENV_DST) ]; then \
		echo ".env already exists. Delete it first to regenerate."; \
	else \
		cp $(ENV_SAMPLE) $(ENV_DST); \
		echo "Filling out .env (press Enter to leave a field empty):"; \
		while IFS= read -r line || [ -n "$$line" ]; do \
			[[ "$$line" =~ ^#.*$$ || -z "$$line" ]] && continue; \
			key=$$(echo "$$line" | cut -d= -f1); \
			default=$$(echo "$$line" | cut -d= -f2-); \
			if echo "$$key" | grep -qiE 'PASSWORD|SECRET|KEY|TOKEN|PASS'; then \
				read -rsp "  $$key: " v < /dev/tty && echo; \
			else \
				read -rp  "  $$key [$$default]: " v < /dev/tty; \
			fi; \
			[ -z "$$v" ] && v="$$default"; \
			sed -i "s|^$$key=.*|$$key=$$v|" $(ENV_DST); \
		done < $(ENV_SAMPLE); \
		echo ".env created."; \
	fi
