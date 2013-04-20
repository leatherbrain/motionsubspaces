#*****************************************************************************
# FILE:				Makefile
# DESCRIPTION:			Makefile for slammer library and tests.
# 
# REVISION HISTORY:		AUTHOR:		Sudeep S.
# 						DATE:		03 Nov 2010
# 						COMMENTS:	File created.
#*****************************************************************************/

OBJDIR = ./../lib/
TSTDIR = ./../tst/

CXX = colorgcc
CXX_GPP34 = g++34

FLAGS_REL_PASS_1 = -O3 -pipe -Wall -W -fprofile-arcs -D_FILE_OFFSET_BITS=64 -msse -finline-functions -ffast-math -fPIC
FLAGS_REL_PASS_2 = -O3 -pipe -Wall -W -freorder-functions -fbranch-probabilities -D_FILE_OFFSET_BITS=64 -msse -finline-functions -ffast-math -fPIC
FLAGS_REL_TST = -O3 -pipe -Wall -W -D_FILE_OFFSET_BITS=64 -msse -finline-functions -ffast-math

FLAGS_DEBUG = -g -DDEBUG -D_FILE_OFFSET_BITS=64 -Wall -W -fPIC
FLAGS_DEBUG_TST = -g -DDEBUG -D_FILE_OFFSET_BITS=64

#**********************************************************
# SET DEBUG / RELEASE MODES HERE
#**********************************************************
#
FLAGS_1 = $(FLAGS_REL_PASS_1)
FLAGS_2 = $(FLAGS_REL_PASS_2)
FLAGS_TST = $(FLAGS_REL_TST)
#
# UNCOMMENT THE FOLLOWING LINES FOR DEBUG MODE
#
#FLAGS_1 = $(FLAGS_DEBUG)
#FLAGS_2 = $(FLAGS_DEBUG)
#FLAGS_TST = $(FLAGS_DEBUG_TST)
#
#**********************************************************

INCLUDES = -I../inc \
	-I../../common \
	-I../../vslam/src \
	-I/usr/local \
	-I/usr/local/include \
	-I/usr/local/include/include \
	-I/usr/local/cvd/include \
	-I/usr/local/opencv/include \
	-I/usr/local/TooN/include \
	-I/usr/local/gvars3/include \
	-I/usr/local/tag/include \
		
LIBPATHS = -L../lib \
	-L/usr/local/lib \
	-L/usr/local/cvd/lib \
	-L/usr/lib \
			
LIBS = -lcvd -lopencv_core -lopencv_highgui -lopencv_video 

OBJS = $(OBJDIR)Tracker.o \
	$(OBJDIR)Subspace.o

TARGET = $(OBJDIR)libsubspaces

all: lib tst

$(OBJDIR)Tracker.o: Tracker.cpp ../inc/Tracker.h
	$(CXX) $(FLAGS_1) -c Tracker.cpp $(INCLUDES) -o $(OBJDIR)Tracker.o
	$(CXX) $(FLAGS_2) -c Tracker.cpp $(INCLUDES) -o $(OBJDIR)Tracker.o

$(OBJDIR)Subspace.o: Subspace.cpp ../inc/Subspace.h
	$(CXX) $(FLAGS_1) -c Subspace.cpp $(INCLUDES) -o $(OBJDIR)Subspace.o
	$(CXX) $(FLAGS_2) -c Subspace.cpp $(INCLUDES) -o $(OBJDIR)Subspace.o

lib: $(OBJS)
	rm -f $(TARGET).so*
	ar rcv $(TARGET).a $(OBJS)
	ranlib $(TARGET).a
	cp $(TARGET).a $(TARGET).so.1.0.0
	ln -s $(TARGET).so.1.0.0 $(TARGET).so.1.0
	ln -s $(TARGET).so.1.0.0 $(TARGET).so.1
	ln -s $(TARGET).so.1.0.0 $(TARGET).so
	
tst:

clean:
	rm -f $(OBJDIR)*.o $(OBJDIR)*.gcno $(TSTDIR)*.o $(TARGET)

