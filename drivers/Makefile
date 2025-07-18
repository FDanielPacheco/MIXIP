CC = clang

CFLAGS = -std=gnu99 -I/usr/local/include/ -I../include
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude -fPIC
CFLAGS += -D$(platform)
LD_FLAGS = -shared
LD_FLAGS += -lc -lpthread -lrt -lm -lserialposix -lxml2 -lgpiod

.PHONY: new compile clean

new:
ifeq ($(name),)
	@echo "Error: Please specify a driver name using 'make new name=your_driver_name'"
	@exit 1
endif
	@echo "Creating a new driver library $(name)..."
	@mkdir -p ./src
	@mkdir -p ./include
	@echo   "#include <mixip.h>" \
			"\n" \
			"\nint dsetup( serial_manager_t * serial, const char * name ){\n" \
			" // Runs first and once...\n" \
			" \n  return 0;" \
			"\n}\n" \
			"\nint dloop( flow_t * flow ){\n" \
			" // Runs in loop, in a separeted process, consider limiting the CPU poll with a sleep...\n" \
			" // To stop the other process (read/write) use halt_network( flow ), and continue_network( flow )\n" \
			" \n  return 0;" \
			"\n}\n" \
			"\nint dread( buffer_t * buf ){\n" \
			" \n  return 0;" \
			"\n}\n" \
			"\nint dwrite( buffer_t * buf ){\n" \
			" \n  return 0;" \
			"\n}\n" \ 
			"\nint dexit( void ){\n" \
			" \n  return 0;" \
			"\n}"> ./src/run-$(name).c
	
build:
	@echo "Compiling driver: $(name)"
	@mkdir -p build

build/%.o: src/%.c | build
	@echo "Compiling $< to object file $@"
	$(CC) $(CFLAGS) -c $< -o $@

# $(patsubst PATTERN,REPLACEMENT,TEXT)
build/lib$(name).so: $(patsubst src/%.c, build/%.o, $(wildcard src/*.c) )
	@echo "Building shared library: $@"
	$(CC) $(LD_FLAGS) -o $@ $(patsubst src/%.c, build/%.o, $(wildcard src/*.c) ) 
	@echo "Shared library created: $@"

compile: build/lib$(name).so
	@echo "Done creating the shared library $<!"

clean:
	@echo "Removing the build directory from the library..."
	@rm -rf ./build