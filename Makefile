#
#  $Id: Makefile,v 1.5 2021/02/24 10:18:02 tanaka Exp tanaka $
#  $Revision: 1.5 $
#  $Date: 2021/02/24 10:18:02 $
#  $Author: tanaka $
#
.PHONY: all strip clean depend
GUROBI_ROOT = /opt/gurobi950/linux64
GUROBI_LIBS = -lgurobi_g++5.2 -lgurobi91

OBJS       = bay.o baystate.o greedy.o instance.o ipmodel.o main.o sequence.o \
	solve.o solution.o
SRCS      := $(OBJS:.o=.cpp)

TARGET     = rbrp_ip

CXX        = g++
MAKEDEP    = g++ -MM

CPPFLAGS   = -I$(GUROBI_ROOT)/include
CXXFLAGS   = -Wall -Wextra -Wconversion -Wold-style-cast -pedantic -O2
# CXXFLAGS  += -march=znver2
# CXXFLAGS  += -march=corei7

LDFLAGS    = 
LIBS       = -L$(GUROBI_ROOT)/lib -lboost_program_options $(GUROBI_LIBS)

override CPPFLAGS += -I. # -DDEBUG -DUSE_CLOCK

all:: $(TARGET)

.cpp.o:
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEFS) -c $<

$(TARGET): $(OBJS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEFS) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)

strip:: $(TARGET)
	@strip $(TARGET)

clean:
	rm -f $(TARGET) $(OBJS) *~ *.bak #*

depend:
	@sed -i -e "/^# START/,/# END/d" Makefile
	@echo "# START" >> Makefile
	@$(MAKEDEP) $(CPPFLAGS) $(DEFS) $(SRCS) >> Makefile
	@echo "# END" >> Makefile

# START
bay.o: bay.cpp bay.hpp instance.hpp
baystate.o: baystate.cpp baystate.hpp bay.hpp instance.hpp
greedy.o: greedy.cpp greedy.hpp bay.hpp instance.hpp solution.hpp \
 sequence.hpp /opt/gurobi911/linux64/include/gurobi_c++.h \
 /opt/gurobi911/linux64/include/gurobi_c.h baystate.hpp
instance.o: instance.cpp instance.hpp
ipmodel.o: ipmodel.cpp /opt/gurobi911/linux64/include/gurobi_c++.h \
 /opt/gurobi911/linux64/include/gurobi_c.h greedy.hpp bay.hpp \
 instance.hpp solution.hpp sequence.hpp baystate.hpp ipmodel.hpp \
 parameter.hpp
main.o: main.cpp instance.hpp parameter.hpp solve.hpp
sequence.o: sequence.cpp baystate.hpp bay.hpp instance.hpp sequence.hpp \
 /opt/gurobi911/linux64/include/gurobi_c++.h \
 /opt/gurobi911/linux64/include/gurobi_c.h
solve.o: solve.cpp instance.hpp ipmodel.hpp \
 /opt/gurobi911/linux64/include/gurobi_c++.h \
 /opt/gurobi911/linux64/include/gurobi_c.h bay.hpp baystate.hpp \
 parameter.hpp sequence.hpp solution.hpp solve.hpp
solution.o: solution.cpp solution.hpp bay.hpp instance.hpp sequence.hpp \
 /opt/gurobi911/linux64/include/gurobi_c++.h \
 /opt/gurobi911/linux64/include/gurobi_c.h baystate.hpp
# END
