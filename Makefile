TEST_PROGRAM = tests.out
CXX 		 = clang++
CXXFLAGS     = -std=c++17 -g -Wall -Wextra -pedantic -O0
#CXXFLAGS     = -std=c++17 -g -O3

all : $(TEST_PROGRAM)

$(TEST_PROGRAM) : tests.o customClass.o SimpleGraphs.hpp
	$(CXX) -o $(TEST_PROGRAM) tests.o customClass.o

tests.o : tests.cpp SimpleGraphs.hpp
	$(CXX) -c $(CXXFLAGS) tests.cpp

customClass.o : CustomClass/customClass.cpp CustomClass/customClass.hpp
	$(CXX) -c $(CXXFLAGS) CustomClass/customClass.cpp

.PHONY: clean
clean:
	rm -f *.o *.out test.txt
run:
	make clean
	make
	./$(TEST_PROGRAM)
grind:
	make clean
	make
	valgrind --tool=callgrind --callgrind-out-file=callgrind.out ./$(TEST_PROGRAM)
	kcachegrind callgrind.out