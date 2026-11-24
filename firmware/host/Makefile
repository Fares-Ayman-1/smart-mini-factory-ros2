CXXFLAGS = -std=c++17 -O1 -Wall -Wextra -fsanitize=address,undefined
all: test_core uno_sim
test_core: test_core.cpp ../uno_factory/core.cpp ../uno_factory/core.h ../uno_factory/config.h
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp ../uno_factory/core.cpp
uno_sim: uno_sim.cpp ../uno_factory/core.cpp ../uno_factory/core.h ../uno_factory/config.h
	$(CXX) $(CXXFLAGS) -o $@ uno_sim.cpp ../uno_factory/core.cpp
test: test_core
	./test_core
clean:
	rm -f test_core uno_sim
.PHONY: all test clean
