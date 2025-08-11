#CXX=g++ -DDEBUGLOG -g -std=c++11 $(CFLAGS) #-p -g
#CC=gcc -DDEBUGLOG -g $(CFLAGS) #-p -g

#CXX=g++ -Wall -Wextra -Wwrite-strings -Werror -g -std=c++11 $(CFLAGS) -DUSEYAMLCPP #-p -g
#CC=gcc -Wall -Wextra -Wwrite-strings -Werror -g $(CFLAGS) -DUSEYAMLCPP #-p -g

#CXX=g++ -g -DUSEIMU -std=c++11 $(CFLAGS) #-p -g
#CC=gcc -g -DUSEIMU $(CFLAGS) #-p -g
CXX=g++  -O3 -std=c++11 $(CFLAGS) #-p -g
CC=gcc   -O3 $(CFLAGS) #-p -g
TARGET=libfalconcamera.so

#CXX=g++ -Wall -Wextra -Wwrite-strings -Werror -O3 -std=c++11 $(CFLAGS) #-p -g
#CC=gcc -Wall -Wextra -Wwrite-strings -Werror -O3 $(CFLAGS) #-p -g
#CXX=g++ -g -std=c++11 $(CFLAGS) #-p -g
#CC=gcc -g $(CFLAGS) #-p -g
#YAMLINC=-I/usr/include/yaml-cpp
#YAMLLIB=-lyaml-cpp

YAMLINC=-I/usr/include
#YAMLLIB=-lyaml
YAMLLIB=-lyaml-cpp

YUVSRC=src/libyuv/compare.cc \
	src/libyuv/compare_common.cc \
	src/libyuv/compare_gcc.cc \
	src/libyuv/compare_msa.cc \
	src/libyuv/compare_neon.cc \
	src/libyuv/compare_neon64.cc \
	src/libyuv/compare_win.cc \
	src/libyuv/convert.cc \
	src/libyuv/convert_argb.cc \
	src/libyuv/convert_from.cc \
	src/libyuv/convert_from_argb.cc \
	src/libyuv/convert_jpeg.cc \
	src/libyuv/convert_to_argb.cc \
	src/libyuv/convert_to_i420.cc \
	src/libyuv/cpu_id.cc \
	src/libyuv/mjpeg_decoder.cc \
	src/libyuv/mjpeg_validate.cc \
	src/libyuv/planar_functions.cc \
	src/libyuv/rotate.cc \
	src/libyuv/rotate_any.cc \
	src/libyuv/rotate_argb.cc \
	src/libyuv/rotate_common.cc \
	src/libyuv/rotate_gcc.cc \
	src/libyuv/rotate_msa.cc \
	src/libyuv/rotate_neon.cc \
	src/libyuv/rotate_neon64.cc \
	src/libyuv/rotate_win.cc \
	src/libyuv/row_any.cc \
	src/libyuv/row_common.cc \
	src/libyuv/row_gcc.cc \
	src/libyuv/row_msa.cc \
	src/libyuv/row_neon.cc \
	src/libyuv/row_neon64.cc \
	src/libyuv/row_win.cc \
	src/libyuv/scale.cc \
	src/libyuv/scale_any.cc \
	src/libyuv/scale_argb.cc \
	src/libyuv/scale_common.cc \
	src/libyuv/scale_gcc.cc \
	src/libyuv/scale_msa.cc \
	src/libyuv/scale_neon.cc \
	src/libyuv/scale_neon64.cc \
	src/libyuv/scale_rgb.cc \
	src/libyuv/scale_uv.cc \
	src/libyuv/scale_win.cc \
	src/libyuv/video_common.cc

YUVOBJ=$(patsubst %.cc,%.o,$(YUVSRC))

TRANSFERLAYERSRC_CPP= src/transferlayer/nvptl_v4luvc.cpp

TRANSFERLAYERSRC_C=src/transferlayer/nvptl.c \
	src/transferlayer/nvptl_net.c \
	src/transferlayer/nvptl_libusb.c

SRC_C=src/timer.c \
	src/nvpfm.cpp \
	src/log.c \
	src/list.c \
	src/utils.c \
	src/nvpfm_upgrade_module.c \
	src/yuv_rgb.c \
	src/timestamp.c \
	src/nvpfmcc.c \
	src/trace.c \
	src/ring_queue.c \
	src/log_file.c

SRC_CPP=src/jsoncpp.cpp

CPPOBJ=$(filter %.o, \
	$(patsubst %.cpp,%.o,$(TRANSFERLAYERSRC_CPP)) \
	$(patsubst %.cpp, %.o, $(SRC_CPP)))
COBJ=$(filter %.o, \
	$(patsubst %.c,%.o, $(TRANSFERLAYERSRC_C)) \
	$(patsubst %.c,%.o, $(SRC_C)) \
	)

INC=-I./include -I./include/libyuv -I/usr/include/libusb-1.0 $(YAMLINC) -I/usr/include -I/usr/include/eigen3/Eigen

CFLAGS += -Wall -g -I $(INC) -fPIC
LDFLAGS += -fPIC -shared

all:test_cpp upgrade_falcon test_rotate batch_upgrade $(TARGET)

$(TARGET): $(CPPOBJ) $(COBJ) $(YUVOBJ)
	$(CC) $(LDFLAGS) $^ -o $@

test_cpp:$(YUVOBJ) utils.o nvptl_v4luvc.o nvptl_libusb.o nvptl.o nvptl_net.o cpp.o nvpfmcpp.o nvpfmcc.o ring_queue.o nvpfm_upgrade_module.o yuv_rgb.o timestamp.o log.o log_file.o trace.o list.o timer.o utils.o
	$(CXX) $^ -o $@ -pthread -lusb-1.0 $(YAMLLIB) -ljpeg -lyaml -ljsoncpp

test_rotate:$(YUVOBJ) testrotate.o
	$(CXX) $^ -o $@ -pthread -lusb-1.0 $(YAMLLIB) -ljpeg -lyaml -ljsoncpp

upgrade_falcon:$(YUVOBJ) utils.o nvptl_v4luvc.o nvptl_libusb.o nvptl.o nvptl_net.o upgrade.o nvpfmcpp.o nvpfmcc.o ring_queue.o nvpfm_upgrade_module.o yuv_rgb.o timestamp.o log.o log_file.o trace.o list.o timer.o utils.o
	$(CXX) $^ -o $@ -pthread -lusb-1.0 $(YAMLLIB) -ljpeg -lyaml -ljsoncpp

batch_upgrade:$(YUVOBJ) utils.o nvptl_v4luvc.o nvptl_libusb.o nvptl.o nvptl_net.o batchupgrade.o nvpfmcpp.o nvpfmcc.o ring_queue.o nvpfm_upgrade_module.o yuv_rgb.o timestamp.o log.o log_file.o trace.o list.o timer.o utils.o
	$(CXX) $^ -o $@ -pthread -lusb-1.0 $(YAMLLIB) -ljpeg -lyaml -ljsoncpp

# sdk-falcon.o: $(SDKOBJ)
# 	$(CXX) $(INC) -c $^ -o $@ -pthread

nvptl_v4luvc.o:src/transferlayer/nvptl_v4luvc.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

nvptl_net.o:src/transferlayer/nvptl_net.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl_libusb.o:src/transferlayer/nvptl_libusb.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl.o:src/transferlayer/nvptl.c
	$(CC) $(INC) -c $^ -o $@ -pthread

nvptl_test.o:src/transferlayer/nvptl.c
	$(CC) -D_TEST $(INC) -c $^ -o $@ -pthread

cpp.o:example/cpp.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

upgrade.o:example/upgrade.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

batchupgrade.o:example/batchupgrade.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread
	
timestamp.o:src/timestamp.c
	$(CC) $(INC) -c $^ -o $@ -pthread
yuv_rgb.o:src/yuv_rgb.c
	$(CC) $(INC) -c $^ -o $@ -pthread
nvpfmcc.o:src/nvpfmcc.c
	$(CC) $(INC) -c $^ -o $@ -pthread

$(YUVOBJ):%.o:%.cc
	$(CC) $(INC) -c $< -o $@ -pthread

$(CPPOBJ):%.o:%.cpp
	$(CC) $(CFLAGS) $(INC) -c $< -o $@ -pthread

$(COBJ):%.o:%.c
	$(CC) $(CFLAGS) $(INC) -c $< -o $@ -pthread

nvpfm.o:src/nvpfm.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

testrotate.o:example/testrotate.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread
	
utils.o:src/utils.c
	$(CC) $(INC) -c $^ -o $@ -pthread
timer.o:src/timer.c
	$(CC) $(INC) -c $^ -o $@ -pthread
log.o:src/log.c
	$(CC) $(INC) -c $^ -o $@ -pthread
log_file.o:src/log_file.c
	$(CC) $(INC) -c $^ -o $@ -pthread
trace.o:src/trace.c
	$(CC) $(INC) -c $^ -o $@ -pthread
list.o:src/list.c
	$(CC) $(INC) -c $^ -o $@ -pthread
nvpfmcpp.o:src/nvpfm.cpp
	$(CXX) $(INC) -c $^ -o $@ -pthread

nvpfm_upgrade_module.o:src/nvpfm_upgrade_module.c
	$(CC) $(INC) -c $^ -o $@ -pthread

ring_queue.o:src/ring_queue.c
	$(CC) $(INC) -c $^ -o $@ -pthread

clean:
	rm -f *.o $(TARGET) src/*.o src/transferlayer/*.o src/libyuv/*.o test_cpp upgrade_falcon test_rotate batch_upgrade sdk-falcon
