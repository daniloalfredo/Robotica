# ------------------------------------------------
# Generic Makefile
#
# Author: yanick.rochon@gmail.com
# Date  : 2011-08-10
#
# Changelog :
#   2010-11-05 - first version
#   2011-08-10 - added structure : sources, objects, binaries
#                thanks to http://stackoverflow.com/users/128940/beta
# ------------------------------------------------

ifeq (${mode},debug)  
   MODEFLAGS = -g
endif

ifeq (${mode},release)
   MODEFLAGS = -O2
endif


# project name (generate executable with this name)
TARGET   = robot

CC       = g++
# compiling flags here
CFLAGS   = $(MODEFLAGS) -Wall -I. -I/usr/local/lib

LINKER   = g++ -o
# linking flags here
OPENCV_LIBS = $(MODEFLAGS) -lrt -lopencv_core -lopencv_imgproc -lopencv_video -lopencv_highgui -lopencv_photo -lopencv_flann -lopencv_stitching -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_core -lopencv_legacy
LFLAGS   = -Wall -I. -lm  -lwiringPi -lpthread -lstdc++ $(OPENCV_LIBS)

MYSRCSDIR   = .
ROBOTAPIDIR = ./robotAPI
OBJDIR      = ./obj

MYSOURCES  := $(wildcard $(MYSRCSDIR)/*.cpp) 
MYINCLUDES := $(wildcard $(MYSRCSDIR)/*.h)
MYOBJS     := $(patsubst $(MYSRCSDIR)/%, $(OBJDIR)/%, $(MYSOURCES:.cpp=.o))

RAPISOURCES := $(wildcard $(ROBOTAPIDIR)/*.cpp)
RAPIINCLUDES := $(wildcard $(ROBOTAPIDIR)/*.h)
RAPIOBJS := $(patsubst $(ROBOTAPIDIR)/%, $(OBJDIR)/%, $(RAPISOURCES:.cpp=.o))

rm       = rm -f



$(TARGET): $(MYOBJS) $(RAPIOBJS)
	@echo "Linking" $@
	@echo $(LINKER) $@ $(LFLAGS) $(MYOBJS) $(RAPIOBJS)
	@$(LINKER) $@ $(LFLAGS) $(MYOBJS) $(RAPIOBJS)
	@echo "Linking complete!"

$(RAPIOBJS): $(OBJDIR)/%.o : $(ROBOTAPIDIR)/%.cpp $(ROBOTAPIDIR)/%.h
	@echo "Building" $<
	@echo $(CC) $(CFLAGS) -c $< -o $@
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

$(MYOBJS): $(OBJDIR)/%.o : $(MYSOURCES) $(MYINCLUDES)
	@echo "Building" $<
	@echo $(CC) $(CFLAGS) -c $< -o $@
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

.PHONEY: clean
clean:
	@$(rm) $(MYOBJS) $(RAPIOBJS)
	@echo "Cleanup complete!"

.PHONEY: remove
remove: clean    
	@$(rm) $(TARGET)
	@echo "Executable removed!"

.PHONEY: display
display:
	@echo mode: $(mode)
	@echo CFLAGS: $(CFLAGS)
	@echo LFLAGS: $(LFLAGS)
	@echo $(ROBOTAPIDIR)/%.cpp
	@echo "robotAPI="
	@echo $(ROBOTAPIDIR)
	@echo "RAPISOURCES="
	@echo $(RAPISOURCES) 
	@echo "RAPIINCLUDES="
	@echo $(RAPIINCLUDES)
	@echo "RAPIOBJECTS="
	@echo $(RAPIOBJS)
	@echo "MYSOURCES="
	@echo $(MYSOURCES) 
	@echo "MYINCLUDES="
	@echo $(MYINCLUDES)
	@echo "OBJECTS="
	@echo $(MYOBJS)
	@echo "ROBOT API RULE:"
	@echo $(RAPIOBJS): $(OBJDIR)/%.o : $(ROBOTAPIDIR)/%.cpp $(ROBOTAPIDIR)/%.h
	@echo "MYOBJECTS RULE:"
	@echo $(MYOBJ): $(OBJDIR)/%.o : $(MYSRCSDIR)/%.cpp $(MYSRCSDIR)/%.h
	@echo "TARGET RULE:"
	@echo $(TARGET): $(MYOBJ) $(RAPIOBJS)
 

