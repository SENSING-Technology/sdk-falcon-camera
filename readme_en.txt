Directory and file description:
doc:
1.falcon.chm, chm format api document, including interface function description and parameter description
Example:
1.callback.c, c interface example code, used to execute function test through commands
2.continus.cpp, c++ interface sample code, used to test the stability of long-time operation and the function of plug and pull automatic connection
3.cpp.cpp, c++ interface sample code,  used to execute function test through commands
4.sync.c, c interface example code, including c language synchronous data acquisition interface
5.tcp_client.c,tcp_server.cpp, tcp based application example code
6.upgrade.c, upgrade function example code

include:
1.nvpfm.h, c interface header file
2.nvpfm.hpp, c++ interface header file
3.ring_queue.h, unlocked queue header file
4.yuv_rgb.h, yuv rgb format mutual conversion function header file

src:
1.nvpfm.cpp, c++ interface implementation file
2.nvpfm_upgrade_module.c, camera firmware upgrade, file upload implementation file
3.nvpfmcc.c, c interfaces implementation file 
4.ring_queue.c, unlocked queue implementation file
5.yuv_rgb.c, yuv rgb format mutual conversion implementation file

test:
1.ctest.cpp, gtest test code of c interface
2.makefile, gtest test code compiling makefile
3.test.cpp, gtest test code of c++interface
4.testfile_upload.txt, a sample file used to test uploaded files

changelog.txt: code change log file
LICENSE: license file
makefile: compile the makefile of the sample code in example
nextvpu-usb.rules: The device file permission profile of Falcon camera requires sudo cp nextvpu-usb.rules /etc/udev/rules.d

instructions:
dependencies:
libusb libyaml or libyaml-cpp

compilation method:
make

test method:
0.sudo cp nextvpu-usb.rules /etc/udev/rules.d
1.Insert falcon camera
2.Run test_cpp:
./test_cpp
3.Wait a moment, and enter the test command on the keyboard:
help --- List available test commands
Enter the test command and press Enter