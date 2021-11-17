PROGRAM  = main.exe 
CX 		 = cl
CXXFLAGS = /std:c++17 /EHsc /WX
LD		 = link
LDFLAGS  = /DEBUG

$(PROGRAM) : main.obj SimpleGraphs.hpp
	$(LD) $(LDFLAGS) main.obj

main.obj : main.cpp SimpleGraphs.hpp
	$(CXX) $(CXXFLAGS) /c main.cpp

.PHONY: clean
clean:
	del *.obj *.exe *.ilk *.pdb *.txt