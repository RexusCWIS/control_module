.PHONY: all, clean, mrproper, distclean

# Implicit rules desactivation

TOPDIR   = ../..
GTESTDIR = gtest-svn
LIBGTEST = $(GTESTDIR)/make/libgtest.a

APP = unit.out

CXX = g++
INCFLAGS = -I$(TOPDIR) -Igtest-svn/include -I/opt/local/include
LIBS = $(LIBGTEST) -lpthread 

TESTSRC = crc_test.cpp

OBJ = $(SRC:.cpp=.o)    	\
      $(TESTSRC:.cpp=.o)

all: $(APP)

run: all
	./$(APP)

$(GTESTDIR):
	svn checkout http://googletest.googlecode.com/svn/trunk/ $(GTESTDIR)

$(LIBGTEST): $(GTESTDIR)
	cd $(GTESTDIR)/make; make
	ar -rv $(LIBGTEST) $(GTESTDIR)/make/gtest-all.o

$(APP): $(LIBGTEST) $(OBJ)
	g++ -g -o $@ $(OBJ) $(LIBS)

%.o: %.cpp
	$(CXX) -c -g $(INCFLAGS) -o $@ $<

clean:
	rm -rf $(OBJ)

distclean:
	rm -rf $(APP) $(GTESTDIR)

