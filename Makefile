include ../shared.mk

# Find all C source files recursively in this project folder.
# Excludes GUI and VSCode folders.
SOURCES := $(shell find . -name "*.c" \
	-not -path "./gui/*" \
	-not -path "./.vscode/*")

# Add every project subfolder as an include path,
# so headers like motion.h, navigation.h, color_sensor.h, etc. can be found.
CFLAGS += $(shell find . -type d \
	-not -path "./gui*" \
	-not -path "./.vscode*" \
	| sed 's/^/-I/')

CFLAGS += -Werror

include ../end.mk