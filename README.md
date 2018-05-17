ADIS_Driver is a ROS driver for adis16488, ROS indigo is tested(other version have not been tested yet!) to be fine. In order to build this project, the ROS indigo should be installed first.
#Install dependencies
```
#yaml-cpp should build from source
mkdir path/to/yaml-cpp
git clone https://github.com/jbeder/yaml-cpp
cd yaml-cpp
mkdir build& cd build
cmake ..
make -j4
sudo make install 
# QtExtSerialPort
git clone https://github.com/qextserialport/qextserialport
qmake
make 
sudo make install
# c++11 is need for yaml-cpp
```
After the dependencies is installed, follow the instruction below to build the code.
```
cd path/to/catkin_ws
catkin_make
```
#Open Configure
```
sudo su
cd path/to/catkin_ws
source devel/setup.bash
rosrun adis_driver adis_driver
```

Q&A:
If sampled data is strange, you should check you pc is big end or small end. This driver used small end, you can change it in the config file.