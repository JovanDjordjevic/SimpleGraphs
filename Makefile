# FOR LINUX, CLANG++ COMPILER
MAIN_PROGRAM = main.out 
TEST_PROGRAM = tests.out
CXX 		 = clang++
CXXFLAGS     = -std=c++17 -g -Wall -Wextra -pedantic -O0

all : $(MAIN_PROGRAM) $(TEST_PROGRAM)

$(MAIN_PROGRAM) : main.o 
	$(CXX) -o $(MAIN_PROGRAM) main.o 

main.o : main.cpp SimpleGraphs.hpp
	$(CXX) -c $(CXXFLAGS) main.cpp

$(TEST_PROGRAM) : tests.o customClass.o SimpleGraphs.hpp
	$(CXX) -o $(TEST_PROGRAM) tests.o customClass.o

tests.o : tests.cpp SimpleGraphs.hpp
	$(CXX) -c $(CXXFLAGS) tests.cpp

customClass.o : CustomClass/customClass.cpp CustomClass/customClass.hpp
	$(CXX) -c $(CXXFLAGS) customClass/customClass.cpp

.PHONY: clean
clean:
	rm *.o *.out 



#FOR WINDOWS, MSVC COMPILER
# PROGRAM  = main.exe 
# CX 		 = cl
# CXXFLAGS = /std:c++17 /EHsc /WX
# LD		 = link
# LDFLAGS  = /DEBUG

# $(PROGRAM) : main.obj customClass.obj tests.obj SimpleGraphs.hpp customClass.hpp
# 	$(LD) $(LDFLAGS) main.obj customClass.obj tests.obj

# main.obj : main.cpp SimpleGraphs.hpp
# 	$(CXX) $(CXXFLAGS) /c main.cpp

# tests.obj : tests.cpp SimpleGraphs.hpp
# 	$(CXX) $(CXXFLAGS) /c tests.cpp

# customClass.obj : customClass.cpp 
# 	$(CXX) $(CXXFLAGS) /c customClass.cpp

# .PHONY: clean
# clean:
# 	del *.obj *.exe *.ilk *.pdb *.txt