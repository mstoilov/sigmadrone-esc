PROJECT		= sigmadrive_ext
CPP		= $(CROSS_COMPILE)g++ -std=c++14
CC		= $(CROSS_COMPILE)gcc
INCLUDES	= -I../../../firmware/sigmadrive/src/ `pkg-config --cflags python-3.6`
LIBS		= -lm -lrt -lpthread -ldl -lboost_python3 `pkg-config --libs python-3.6`
CFLAGS		= -g -O0 -Wall -fPIC
TARGET		= $(PROJECT)
OBJECTS		= \
		main.o \



all : $(TARGET)

.SUFFIXES:	.c .cxx .cpp

.c.o: 
	$(CC) -c $< $(CFLAGS) $(INCLUDES)

.cxx.o: 
	$(CPP) -c $< $(CFLAGS) $(INCLUDES)

.cpp.o: 
	$(CPP) -c $< $(CFLAGS) $(INCLUDES)

$(TARGET) : $(OBJECTS)
	$(CPP) -shared $^ -Wl,--cref,-Map,$(TARGET).map -o $(TARGET).so $(LIBS)

clean:
	rm -f *.o *.map *.so *~ $(TARGET)

