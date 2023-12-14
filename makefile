
# mavlink_control: main.cpp
# 	g++ -g -Wall -I include/mavlink/v2.0 main.cpp -o mavlink_control -lstdc++

# clean:
# 	 rm -rf *o mavlink_control
	 
	
CC := g++
CPPFLAGS := -std=c++11 -I include/mavlink/v2.0
OPENCV_LIBS := `pkg-config opencv4 --cflags --libs`

all: lander

aruco_reader: landing.cpp
	$(CC) $(CPPFLAGS) $< -o $@ $(OPENCV_LIBS)

clean:
	rm -f lander
