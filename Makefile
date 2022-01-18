# FOR LINUX, CLANG++ COMPILER
PROGRAM  = main.out 
CX 		 = clang++
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic

$(PROGRAM) : main.o customClass.o SimpleGraphs.hpp customClass.hpp
	$(CXX) main.o customClass.o

main.o : main.cpp SimpleGraphs.hpp
	$(CXX) -c $(CXXFLAGS) main.cpp

customClass.obj : customClass.cpp 
	$(CXX) -c $(CXXFLAGS) customClass.cpp

.PHONY: clean
clean:
	rm *.o *.out *.txt



# FOR WINDOWS, MSVC COMPILER
# PROGRAM  = main.exe 
# CX 		 = cl
# CXXFLAGS = /std:c++17 /EHsc /WX
# LD		 = link
# LDFLAGS  = /DEBUG

# $(PROGRAM) : main.obj customClass.obj SimpleGraphs.hpp customClass.hpp
# 	$(LD) $(LDFLAGS) main.obj customClass.obj

# main.obj : main.cpp SimpleGraphs.hpp
# 	$(CXX) $(CXXFLAGS) /c main.cpp

# customClass.obj : customClass.cpp 
# 	$(CXX) $(CXXFLAGS) /c customClass.cpp

# .PHONY: clean
# clean:
# 	del *.obj *.exe *.ilk *.pdb *.txt