CC = gcc

CFLAGS = -std=gnu99 -I/usr/local/include/ -I../include
CFLAGS += -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude
LD_FLAGS = -shared # -L/usr/local/lib/x86_64-linux-gnu/ 
LD_FLAGS += -lc -lpthread -lrt -lm -lserialposix -lxml2

.PHONY: new compile clean

new:
ifeq ($(name),)
	@echo "Error: Please specify a driver name using 'make new name=your_driver_name'"
	@exit 1
endif
	@echo "Creating a new driver library $(name)..."
	@mkdir -p ./$(name)
	@echo   "#include <mixip.h>" \
			"\n" \
			"\nint dsetup( serial_manager_t * serial ){\n" \
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
			"\n}" > ./$(name)/$(name).c
	
clean:
ifeq ($(name),)
	@echo "Error: Please specify a driver name using 'make clean name=your_driver_name'"
	@exit 1
endif
	@echo "Removing the build directory from the $(name) library..."
	@rm -rf ./$(name)/build
 

$(name)/build:
	@echo "Compiling driver: $(name)"
	@mkdir -p $(name)/build

$(name)/build/%.o: $(name)/%.c | $(name)/build
	@echo "Compiling $< to object file $@"
	$(CC) $(CFLAGS) -c $< -o $@

# $(patsubst PATTERN,REPLACEMENT,TEXT)
$(name)/build/lib$(name).so: $(patsubst $(name)/%.c, $(name)/build/%.o, $(wildcard $(name)/*.c) )
	@echo "Building shared library: $@"
	$(CC) $(LD_FLAGS) -o $@ $(patsubst $(name)/%.c, $(name)/build/%.o, $(wildcard $(name)/*.c) ) 
	@echo "Shared library created: $@"

compile: $(name)/build/lib$(name).so
	@echo "Done creating the shared library $<!"
