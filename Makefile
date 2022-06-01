#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all:  msgs navigation simulator
	$(info Build_type is [${build_type}])
	# $(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib
	cd submodules/graph_navigation && rm -rf build bin lib
	cd submodules/amrl_msgs && rm -rf build bin lib
	cd submodules/ut_multirobot_sim && rm -rf build bin lib
	cd submodules/pips && rm -rf build bin lib

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile manifest.xml
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

pedsim:
	cd submodules/pedsim_ros && catkin_make

msgs:
	cd submodules/amrl_msgs && $(MAKE)

navigation:
	cd submodules/graph_navigation && $(MAKE)

simulator:
	cd submodules/ut_multirobot_sim && $(MAKE)

pips:
	cd submodules/pips && $(MAKE)
