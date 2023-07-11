# Compiler
CC := g++
# Compiler flags for the debug build
DEBUG_FLAGS := -std=c++14 -Wall -Wextra -pedantic -g
# Compiler flags for the release build (no debugging)
RELEASE_FLAGS := -std=c++11 -Wall -Wextra -pedantic
# Source files
SRCS := testCoverage.cpp coverage.cpp
# Object files
OBJS := $(SRCS:.cpp=.o)
# Executable
TARGET := testCoverage

# Default target (release build)
all: $(TARGET)

# Release build - Linking object files to create the executable
$(TARGET): $(OBJS)
	$(CC) $(RELEASE_FLAGS) $(OBJS) -o $(TARGET)

# Compile source files into object files for the release build
%.o: %.cpp
	$(CC) $(RELEASE_FLAGS) -c $< -o $@

# Debug build - Linking object files to create the executable with debugging information
debug: DEBUG_FLAGS += -DDEBUG
debug: $(OBJS)
	$(CC) $(DEBUG_FLAGS) $(OBJS) -o $(TARGET)_debug

# Compile source files into object files for the debug build
%.o: %.cpp
	$(CC) $(DEBUG_FLAGS) -c $< -o $@

# Cleanup
clean:
	rm -f $(OBJS) $(TARGET) $(TARGET)_debug
