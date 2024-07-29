#######################################
# binaries
#######################################
PREFIX = 
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CPP = $(GCC_PATH)/$(PREFIX)g++
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif

TARGET = trapezoidprofile.so


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0


#######################################
# paths
#######################################
# Build path
BUILD_DIR = .

C_DEFS = -D_USE_PYBIND_

C_INCLUDES = `python3 -m pybind11 --includes`

CFLAGS = $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fPIC --shared -std=gnu++14

LDFLAGS = -shared -fPIC

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


OBJECTS = $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(notdir $(wildcard commandline/*.c))))
vpath %.c commandline/

OBJECTS += $(addprefix $(BUILD_DIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ./*.cpp))))
vpath %.cpp ./

OBJECTS += $(addprefix $(BUILD_DIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard rexjson/*.cpp))))
vpath %.cpp rexjson/

# default action: build all
all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJECTS) Makefile | $(BUILD_DIR)
	$(CPP) $(OBJECTS) -O3 -Wall -shared -std=c++11 -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CPP)  $< -c -o $@ -O3 -Wall $(CFLAGS) $(LDFLAGS) #-shared -std=c++11 -fPIC `python3 -m pybind11 --includes`

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@


$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(TARGET) $(BUILD_DIR)/*.o $(BUILD_DIR)/*.lst $(BUILD_DIR)/*.d

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)





