SHELL := /bin/bash
.SILENT:
.DEFAULT_GOAL := help

ROOT   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
FW_DIR := $(ROOT)/firmware
FW_BIN := $(FW_DIR)/.pio/build/esp32cam/firmware.bin
FW_SRC := $(wildcard $(FW_DIR)/src/*.cpp $(FW_DIR)/src/*.h $(FW_DIR)/platformio.ini)

.PHONY: help setup-env hotspot build serial lint format clean

help: ## Show available targets
	echo "ROSS — dev infra targets:"
	echo
	grep -E '^[a-zA-Z_-]+:.*##' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'
	echo
	echo "Python CLI: run \`uv run ross --help\` (or \`ross --help\` if the venv is on PATH)."

setup-env: ## Configure WiFi credentials in .env
	@if [ -f $(ROOT)/.env ]; then \
		echo ".env already exists. Delete it first to regenerate."; \
	else \
		cp $(ROOT)/.env.sample $(ROOT)/.env; \
		read -rp  "  WIFI_SSID: " ssid < /dev/tty; \
		read -rsp "  WIFI_PASS: " pass < /dev/tty && echo; \
		sed -i "s|^WIFI_SSID=.*|WIFI_SSID=$$ssid|" $(ROOT)/.env; \
		sed -i "s|^WIFI_PASS=.*|WIFI_PASS=$$pass|" $(ROOT)/.env; \
		echo ".env created."; \
	fi

hotspot: ## Start the "omnibook" WiFi hotspot (NetworkManager profile "Hotspot")
	nmcli connection up Hotspot

$(FW_BIN): $(FW_SRC)
	cd $(FW_DIR) && pio run

build: $(FW_BIN) ## Build firmware (PlatformIO)

serial: ## Monitor serial output (Ctrl-A k to exit)
	screen /dev/ttyAMA0 115200

lint: ## Check style with ruff
	uvx ruff check ross

format: ## Format with ruff
	uvx ruff format ross

clean: ## Remove firmware build artifacts
	rm -rf $(FW_DIR)/.pio
