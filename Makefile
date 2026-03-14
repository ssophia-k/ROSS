SHELL := /bin/bash
.SILENT:
.DEFAULT_GOAL := help

REPO_ROOT := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
ENV_SAMPLE := $(REPO_ROOT)/.env.sample
ENV_DST    := $(REPO_ROOT)/.env

## Help
.PHONY: help
help: ## Show this help message
	echo "Available targets:"
	echo "=================="
	grep -E '(^[a-zA-Z_-]+:.*?## .*$$|^## )' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; \
		     /^## / {gsub("^## ", ""); print "\n\033[1;35m" $$0 "\033[0m"}; \
		     /^[a-zA-Z_-]+:/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

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