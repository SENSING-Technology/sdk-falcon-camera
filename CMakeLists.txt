cmake_minimum_required(VERSION 3.1.0)
project(falcon_sdk)
# 开启debug编译模式, 这样编译会带上符号
set(CMAKE_BUILD_TYPE "Release")

set(YUVSRC
  src/libyuv/compare.cc
        src/libyuv/compare_common.cc
        src/libyuv/compare_gcc.cc
        src/libyuv/compare_msa.cc
        src/libyuv/compare_neon.cc
        src/libyuv/compare_neon64.cc
        src/libyuv/compare_win.cc
        src/libyuv/convert.cc
        src/libyuv/convert_argb.cc
        src/libyuv/convert_from.cc
        src/libyuv/convert_from_argb.cc
        src/libyuv/convert_jpeg.cc
        src/libyuv/convert_to_argb.cc
        src/libyuv/convert_to_i420.cc
        src/libyuv/cpu_id.cc
        src/libyuv/mjpeg_decoder.cc
        src/libyuv/mjpeg_validate.cc
        src/libyuv/planar_functions.cc
        src/libyuv/rotate.cc
        src/libyuv/rotate_any.cc
        src/libyuv/rotate_argb.cc
        src/libyuv/rotate_common.cc
        src/libyuv/rotate_gcc.cc
        src/libyuv/rotate_msa.cc
        src/libyuv/rotate_neon.cc
        src/libyuv/rotate_neon64.cc
        src/libyuv/rotate_win.cc
        src/libyuv/row_any.cc
        src/libyuv/row_common.cc
        src/libyuv/row_gcc.cc
        src/libyuv/row_msa.cc
        src/libyuv/row_neon.cc
        src/libyuv/row_neon64.cc
        src/libyuv/row_win.cc
        src/libyuv/scale.cc
        src/libyuv/scale_any.cc
        src/libyuv/scale_argb.cc
        src/libyuv/scale_common.cc
        src/libyuv/scale_gcc.cc
        src/libyuv/scale_msa.cc
        src/libyuv/scale_neon.cc
        src/libyuv/scale_neon64.cc
        src/libyuv/scale_rgb.cc
        src/libyuv/scale_uv.cc
        src/libyuv/scale_win.cc
        src/libyuv/video_common.cc)

set(TRANSFERLAYERSRC
        src/transferlayer/nvptl_v4luvc.cpp
        src/transferlayer/nvptl.c
        src/transferlayer/nvptl_net.c
        src/transferlayer/nvptl_libusb.c)

set(OTHERSRC src/timer.c
    src/nvpfm.cpp
    src/log.c
    src/list.c
    src/utils.c
    src/nvpfm_upgrade_module.c
    src/yuv_rgb.c
    src/timestamp.c
    src/nvpfmcc.c
    src/trace.c
    src/ring_queue.c
    src/jsoncpp.cpp
    src/log_file.c)
set(CNNSRC
        src/cnn/yolox_post_process.cpp
        src/cnn/hand_post.cpp
        )
set(DEPTHFILTERSRC
        src/depthfilter/smearfilter.cpp
        )

include_directories(
        ./include
        ./include/cnn/
        ./include/depthfilter
        ./include/libyuv
        ./test
        /usr/include/libusb-1.0
        ${OpenCV_INCLUDE_DIRS}
        /usr/include
        /usr/include/eigen3/Eigen
  )
  file(GLOB BIND_SRC "pybind/*.cc" "pybind/*.cpp" "pybind/*.c")
  find_package(OpenCV REQUIRED)
  # import pybind
  find_package(pybind11 REQUIRED)
add_library(falcon ${OTHERSRC} ${YUVSRC} ${CNNSRC} ${TRANSFERLAYERSRC} ${DEPTHFILTERSRC})


add_executable(test_cpp example/cpp.cpp)
add_executable(changeip example/changeip.cpp)
add_executable(test_align example/depthalignrgb.cpp)
add_executable(test_pointcloud example/pointcloud.cpp)
add_executable(test_ai example/ai.cpp)
#add_executable(testfalcon ${TESTSRC})
target_link_libraries(test_cpp falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS})
target_link_libraries(changeip falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS})
target_link_libraries(test_align falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS})
target_link_libraries(test_pointcloud falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS})
target_link_libraries(test_ai falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS})
#target_link_libraries(testfalcon falcon pthread usb-1.0 yaml jpeg turbojpeg ${OpenCV_LIBS} gtest)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/libyuv /usr/include/libusb-1.0 /usr/include/eigen3/Eigen)
# it could also build sdk! easier than makefile.
pybind11_add_module(${PROJECT_NAME} ${OTHERSRC} ${YUVSRC} ${CNNSRC} ${TRANSFERLAYERSRC} ${DEPTHFILTERSRC} ${BIND_SRC})

target_link_libraries(${PROJECT_NAME} PRIVATE  pthread usb-1.0  jpeg yaml jsoncpp turbojpeg ${OpenCV_LIBS})
