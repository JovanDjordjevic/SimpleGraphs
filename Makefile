TEST_PROGRAM = tests.out
CXX 		 = clang++
CXXFLAGS     = -std=c++17 -g -Wall -Wextra -pedantic -O0

all : $(TEST_PROGRAM)

$(TEST_PROGRAM) : tests.o customClass.o SimpleGraphs.hpp
	$(CXX) -o $(TEST_PROGRAM) tests.o customClass.o

tests.o : tests.cpp SimpleGraphs.hpp
	$(CXX) -c $(CXXFLAGS) tests.cpp

customClass.o : CustomClass/customClass.cpp CustomClass/customClass.hpp
	$(CXX) -c $(CXXFLAGS) customClass/customClass.cpp

.PHONY: clean
clean:
	rm *.o *.out test.txt