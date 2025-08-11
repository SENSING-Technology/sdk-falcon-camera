使用说明：
安装依赖:
sudo apt-get update
sudo apt-get install libusb-1.0-0-dev libusb-dev libyaml-cpp-dev libyaml-dev libyaml-cpp0.6 gcc g++ make libopencv-dev libjsoncpp-dev libeigen3-dev libjpeg-turbo8-dev python3-sympy


编译方法:
make

测试方法(假设当前目录是sdk-falcon-camera):
0.sudo cp nextvpu-usb.rules /etc/udev/rules.d
1.插入falcon相机
2.运行test_cpp:
./test_cpp
可以进行usb插拔测试，观察控制台打印

自动化测试方法(假设当前目录是sdk-falcon-camera):
1.   cd test
2.   make
3.   ./testfalcon --gtest_repeat=-1


upgrade multiple cameras:
make upgrade_falcon
./upgrade_falcon romfilepath "romfileversion"

example:
./upgrade_falcon ~/falcon_fw.bin "2023/06/08 16:24:56"


add additional vid/pid:

modify src/nvpfmcc.c:
find "const VPS" array definition
example:
```
const VIDPID VPS[]={
	{0x4E56,0x5055},
	{0x2BDF,0x0001}
};
```
vid/pid1:vid=0x4E56,pid=0x5055
vid/pid2:vid=0x2BDF,pid=0x0001

if we want to add additional vid/pid:vid=0x1234,pid=0x3456:
just modify const VIDPID as below:

```
const VIDPID VPS[]={
	{0x4E56,0x5055},
	{0x2BDF,0x0001},
	{0x1234,0x3456}
};
```
then,we make clean and make


add additional vid/pid:vid=0x1234 pid=0x4567 to rules file:
edit ./nextvpu-usb.rules
```
SUBSYSTEM=="usb", ATTR{idVendor}=="4e56", ATTR{idProduct}=="5055", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="falcon0-%b"
SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", ATTR{idProduct}=="0001", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="falcon1-%b"
SUBSYSTEM=="usb", ATTR{idVendor}=="1234", ATTR{idProduct}=="4567", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="falcon1-%b"
```
sudo cp nextvpu-usb.rules /etc/udev/rules.d

notes:
if there's already a file named nextvpu-usb.rules in /etc/udev/rules.d exists,so you should reboot computer to make rules effective.
