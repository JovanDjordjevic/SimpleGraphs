PROGRAM  = main.exe 
CX 		 = cl
CXXFLAGS = /std:c++17 /EHsc /WX
LD		 = link
LDFLAGS  = /DEBUG

$(PROGRAM) : main.obj customClass.obj SimpleGraphs.hpp customClass.hpp
	$(LD) $(LDFLAGS) main.obj customClass.obj

main.obj : main.cpp SimpleGraphs.hpp
	$(CXX) $(CXXFLAGS) /c main.cpp

customClass.obj : customClass.cpp 
	$(CXX) $(CXXFLAGS) /c customClass.cpp

.PHONY: clean
clean:
	del *.obj *.exe *.ilk *.pdb *.txt