.SILENT:
.IGNORE:
.PHONY:

help: ## Show this help message
	echo "Available targets:"
	echo "=================="
	grep -E '(^[a-zA-Z_-]+:.*?## .*$$|^# Section: )' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; \
		     /^# Section:/ {gsub("^# Section: ", ""); print "\n\033[1;35m" $$0 "\033[0m"}; \
		     /^[a-zA-Z_-]+:/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

sync-notebooks: ## Sync all notebooks with their script counterparts
	@echo "Syncing all notebooks..."
	@for nb in notebooks/*.ipynb; do \
		echo "Syncing $$nb"; \
		jupytext --sync $$nb; \
	done
