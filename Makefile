CXX:= g++

DEFINES := -DUNIX

CXXFLAGS := -fPIC

ifdef BIT32
CXXFLAGS += -m32
endif

PCH_HEADER := ./DLL430_v3/src/TI/DLL430/pch.h
PCH_COMPILED := ./DLL430_v3/src/TI/DLL430/pch.h.gch

ifdef DEBUG
CXXFLAGS += -g -O0
endif

ifdef VERIFY
CXX:= clang++
CXXFLAGS += -O0
PCH_COMPILED := ./DLL430_v3/src/TI/DLL430/pch.h.pch
else
CXXFLAGS += -Os
DEFINES += -DNDEBUG
endif

MAKE_PCH += -x c++-header
USE_PCH += -include $(PCH_HEADER)

export BOOST_DIR
export BIT32
export STATIC
export DEBUG

INCLUDES := \
	-I./DLL430_v3/src/TI/DLL430 \
	-I./DLL430_v3/include \
	-I./DLL430_v3/src/TI/DLL430/EnergyTrace_TSPA \
	-I./Bios/include \
	-I./ThirdParty/include \
	-I./ThirdParty/BSL430_DLL

LIBDIRS := -L./ThirdParty/lib

BSLLIB := ./ThirdParty/lib/libbsl430.a

ifdef BOOST_DIR
INCLUDES += -I$(BOOST_DIR)
LIBDIRS += -L$(BOOST_DIR)/stage/lib
endif

LIBS :=
STATIC_LIBS :=

ifdef STATIC
STATIC_LIBS += -lboost_thread -lboost_filesystem -lboost_date_time -lboost_system -lbsl430 -lusb-1.0 -lrt
else
LIBS += -lboost_thread -lboost_filesystem -lboost_date_time -lboost_system -lbsl430 -lusb-1.0 -lrt
endif

LIBS += -lpthread



SRC := \
        ./DLL430_v3/src/TI/DLL430/EEM/CycleCounter.cpp \
        $(wildcard ./DLL430_v3/src/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/BreakpointManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/CycleCounter/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/EemRegisters/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/EmulationManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Exceptions/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Sequencer/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/SoftwareBreakpoints/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/StateStorage430/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trace/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trigger/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerCondition/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/VariableWatch/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EnergyTrace_TSPA/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/logging/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/TemplateDeviceDb/*.cpp)

OBJS := $(patsubst %.cpp, %.o, $(SRC))

OUTPUT := libmsp430.so

all: $(BSLLIB) $(OBJS)
	$(CXX) $(CXXFLAGS) -shared -Wl,-soname,$(OUTPUT) -o $(OUTPUT) $(OBJS) ThirdParty/lib/hid-libusb.o $(LIBDIRS) -Wl,-Bstatic $(STATIC_LIBS) -Wl,-Bdynamic $(LIBS)

$(PCH_COMPILED): $(PCH_HEADER)
	$(CXX) $(MAKE_PCH) -o $@ $< $(CXXFLAGS) $(INCLUDES) $(DEFINES)

%.o: %.cpp $(PCH_COMPILED)
	$(CXX) -c -o $@ $< $(USE_PCH) $(CXXFLAGS) $(INCLUDES) $(DEFINES)

$(BSLLIB):
	$(MAKE) -C ./ThirdParty/BSL430_DLL

install:
	cp $(OUTPUT) /usr/local/lib/

clean:
	$(MAKE) -C ./ThirdParty/BSL430_DLL clean
	@for i in $(OBJS); do rm -f $$i; done
	@rm -f $(PCH_HEADER).?ch build.log
