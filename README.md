ADIS_Driver for adis16488
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

Q&A:
If sampled data is strange, you should check you pc is big end or small end. This driver used small end, you can change it in the config file.