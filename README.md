ADIS_Driver for 
#Install dependencies
```
#yaml-cpp
sudo apt-get install libyaml-cpp-dev
# QtExtSerialPort
sudo apt-get install libqextserialport-dev
# c++11 is need for yaml-cpp

```
#Open Configure
Add the Use to group "dialout"
```
sudo usermod -a -G dialout $USER
```
then suspend and reboot.