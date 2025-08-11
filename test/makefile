#CXX=g++ -g -std=c++11 -DUSEIMU -DUSEYAMLCPP
#CC=gcc -g -DUSEIMU -DUSEYAMLCPP
CXX=g++ -g -std=c++11 -DUSEIMU
CC=gcc -g -DUSEIMU
OBJ_DIR=build
#YAMLLIB=-lyaml-cpp
YAMLLIB=-lyaml
YAMLINC=-I/usr/include
INC=-I../include -I../include/libyuv -I/usr/include/libusb-1.0 $(YAMLINC) -I/usr/include -I/usr/include/eigen3/Eigen


#wildcard : 扩展通配符
YUVSRC=$(wildcard ../src/libyuv/*.cc)
# patsubst ：替换通配符
YUVOBJ=$(patsubst %.cc,%.o, $(YUVSRC))

all:testfalcon

testfalcon:$(YUVOBJ) test.o test_process.o nvpfmcpp.o nvpfmcc.o ring_queue.o nvptl_v4luvc.o nvptl_net.o  nvptl_libusb.o nvptl.o nvpfm_upgrade_module.o yuv_rgb.o timestamp.o log.o log_file.o trace.o list.o timer.o utils.o
	$(CXX) -o $@ $^ -pthread -lgtest $(YAMLLIB) -lusb-1.0 -ljpeg -ljsoncpp
	
$(YUVOBJ):%.o:%.cc
	$(CC) $(INC) -c $< -o $@ -pthread

timestamp.o:../src/timestamp.c
	$(CC) $(INC) -c $^ -o $@ -pthread
yuv_rgb.o:../src/yuv_rgb.c
	$(CC) $(INC) -c $^ -o $@ -pthread
nvpfmcc.o:../src/nvpfmcc.c
	$(CC) $(INC) -c $^ -o $@ -pthread
nvpfm.o:../src/nvpfm.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

utils.o:../src/utils.c
	$(CC) $(INC) -c $^ -o $@ -pthread
timer.o:../src/timer.c
	$(CC) $(INC) -c $^ -o $@ -pthread
log.o:../src/log.c
	$(CC) $(INC) -c $^ -o $@ -pthread
log_file.o:../src/log_file.c
	$(CC) $(INC) -c $^ -o $@ -pthread
trace.o:../src/trace.c
	$(CC) $(INC) -c $^ -o $@ -pthread
list.o:../src/list.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl_v4luvc.o:../src/transferlayer/nvptl_v4luvc.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

nvptl_net.o:../src/transferlayer/nvptl_net.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl_libusb.o:../src/transferlayer/nvptl_libusb.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl.o:../src/transferlayer/nvptl.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvpfmcpp.o:../src/nvpfm.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

nvpfm_upgrade_module.o:../src/nvpfm_upgrade_module.c
	$(CC) $(INC) -c $^ -o $@ -pthread

ring_queue.o:../src/ring_queue.c
	$(CC) $(INC) -c $^ -o $@ -pthread


test.o:test.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

test_process.o:test_process.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread
clean:
	rm -fr *.o testfalcon
