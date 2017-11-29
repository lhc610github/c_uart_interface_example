# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`

all: mavlink_control

mavlink_control: git_submodule mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp lcm_interface.cpp autopilot_interface.cpp avoid_potential_interface.cpp -o mavlink_control -lpthread $(LDFLAGS)

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o mavlink_control
