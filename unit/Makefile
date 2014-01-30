.PHONY: all, clean, mrproper, distclean

# Implicit rules desactivation

TOPDIR   = ..
GTESTDIR = gtest-svn
LIBGTEST = $(GTESTDIR)/make/libgtest.a

BUILDDIR = build
APP = unit.out

CC  = gcc
CXX = g++
INCFLAGS = -I$(TOPDIR) -Igtest-svn/include -I/opt/local/include
LIBS = $(LIBGTEST) -lpthread 

PERL = perl

CSRC = crc_table.c 
TESTSRC = crc_test.cpp crc16.cpp

VPATH += $(sort $(dir $(CSRC) $(TESTSRC)))

OBJ = $(addprefix $(BUILDDIR)/, $(notdir $(CSRC:.c=.o) $(TESTSRC:.cpp=.o)))

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

$(BUILDDIR)/%.o: %.cpp | $(BUILDDIR)
	$(CXX) -c -g $(INCFLAGS) -o $@ $<

$(BUILDDIR)/%.o: %.c | $(BUILDDIR)
	$(CC) -c -g $(INCFLAGS) -o $@ $<

$(BUILDDIR): 
	mkdir $@

crc_table.c: 
	$(PERL) $(TOPDIR)/utils/crc_table_generator.pl 0x1021 $@ 

clean:
	rm -rf $(OBJ)

distclean:
	rm -rf $(APP) $(GTESTDIR)

