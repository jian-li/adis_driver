ADIS_Driver is a ROS driver for adis16488, ROS indigo is tested(other version have not been tested yet!) to be fine. In order to build this project, the ROS indigo should be installed first.
#Install dependencies
```
#yaml-cpp should build from source
mkdir path/to/yaml-cpp
git clone https://github.com/jbeder/yaml-cpp
cd yaml-cpp
mkdir build& cd build
cmake ..
# QtExtSerialPort
sudo apt-get install libqextserialport-dev
# c++11 is need for yaml-cpp

```
After the dependencies is installed, follow the instruction below to build the code.
```
cd path/to/catkin_ws
catkin_make
```
#Open Configure
Add the Use to group "dialout"
```
sudo usermod -a -G dialout $USER
```
then suspend and reboot.

Q&A:
If sampled data is strange, you should check you pc is big end or small end. This driver used small end, you can change it in the config file.