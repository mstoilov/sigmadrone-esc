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

TARGET = sigmadrive-tool


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
BUILD_DIR = bin

C_DEFS =

C_INCLUDES = -I../lib/rexjson \
	-I../lib/commandline \

	
CFLAGS = $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

LDFLAGS = -lm -lrt -pthread -ldl -lboost_system -lboost_filesystem -lboost_program_options -lboost_thread

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


OBJECTS = $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(notdir $(wildcard ../lib/commandline/*.c))))
vpath %.c ../lib/commandline/

OBJECTS += $(addprefix $(BUILD_DIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ../lib/rexjson/*.cpp))))
vpath %.cpp ../lib/rexjson/

OBJECTS += $(addprefix $(BUILD_DIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ../lib/cmdargs/*.cpp))))
vpath %.cpp ../lib/cmdargs/


OBJECTS += $(addprefix $(BUILD_DIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ./*.cpp))))
vpath %.cpp ./


# default action: build all
all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJECTS) Makefile | $(BUILD_DIR)
	$(CPP) -std=gnu++14 $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CPP) -std=gnu++14 -c $(CFLAGS) -fexceptions -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@


$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up










#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)





