
OUTDIR = bin
SRCDIR = .
INCLUDE = -I$(SRCDIR)
TARGET = $(OUTDIR)/ryno
LIBS =
LIBDIRS =

CPP = g++ -std=c++17
CC = gcc
LD = gcc
AR = ar

ifeq ($(BLDCFG), release)
CFLAGS = -O2 $(INCLUDE) -DUNIX
else
CFLAGS = -g -O0 -Wall $(INCLUDE) -DUNIX -DDEBUG
endif


ifeq ($(BLDPROF), yes)
CFLAGS += -pg
LDFLAGS += -pg
endif

all: $(TARGET)

#######################################
# dependencies
#######################################
-include $(wildcard $(OUTDIR)/*.d)

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -I../

OBJECTS += $(addprefix $(OUTDIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ./*.cpp))))
#vpath %.cpp ./

OBJECTS += $(addprefix $(OUTDIR)/, $(patsubst %.cpp,%.o,$(notdir $(wildcard ../cmdargs/*.cpp))))
#vpath %.cpp ../cmdargs


$(OUTDIR)/%.o: ./%.c | $(OUTDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUTDIR)/%.o: ../cmdargs/%.c | $(OUTDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUTDIR)/%.o: %.cpp | $(OUTDIR)
	$(CPP) $(CFLAGS) -c ./$< -o $@

$(OUTDIR)/%.o: ../cmdargs/%.cpp | $(OUTDIR)
	$(CPP) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS) 
	$(CPP) -o $@ $^ $(LIBS) $(LIBDIRS)$(LDFLAGS)

$(OUTDIR):
	@mkdir $@

distclean: clean
	@rm -f .depend
	@rm -rf $(OUTDIR)

clean:
	@rm -f $(TARGET)
	@rm -f $(OUTDIR)/*.o
	@rm -rf $(OUTDIR)
	@rm -f $(SRCDIR)/*~
	@rm -f *~
	@rm -rf docpages/out
	
