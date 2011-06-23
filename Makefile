SRC_DIR		= modules
OUTPUT		= controller

.PHONY: all clean purge docs

# Build the Robot Controller 
all:
	@$(MAKE) -e --directory=$(SRC_DIR)
	@if [ "$$?" != 0 ]; then \
		echo "Failed to build the modules"; \
		exit 1; \
	fi

clean: 
	rm -rf $(OUTPUT)
	@$(MAKE) -e --directory=$(SRC_DIR) clean
	@if [ "$$?" != 0 ]; then \
		echo "Failed to clean the modules"; \
		exit 1; \
	fi

purge: 
	rm -rf $(OUTPUT)
	@$(MAKE) -e --directory=$(SRC_DIR) purge
	@if [ "$$?" != 0 ]; then \
		echo "Failed to clean the modules"; \
		exit 1; \
	fi

# Generate library documentation
docs:
	@echo "Generating documentation files."
	@doxygen
	@echo "Documentation files were installed into $(LIB_DIR)/doc."

