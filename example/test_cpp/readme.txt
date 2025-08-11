依赖libusb/libyaml
git clone https://gitee.com/ant1423/libusb.git
cd libusb
git checkout v1.0.25
msvc下有解决方案，编译64bit的静态库，运行时也静态

git clone https://github.com/yaml/libyaml
用windows下的cmake产生vs的解决方案，然后编译64bit的静态库，运行时也静态
