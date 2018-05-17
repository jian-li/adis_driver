#include "adis_driver/adis_driver.h"
#include "yaml-cpp/yaml.h"

AdisDriver::AdisDriver(std::string pkg_path)
{
	YAML::Node config = YAML::LoadFile(pkg_path+ + "/config/config.yaml");

	// if(config)
	std::string port_path = "/dev/" + config["Device"].as<std::string>();
	port_name_ = QLatin1String(port_path.c_str());
	port_ptr_ = new QextSerialPort(QString(port_name_), QextSerialPort::EventDriven);

	if(port_ptr_->open(QIODevice::ReadOnly) == true)
		std::cout << "==Open " << port_name_.toStdString() << " success!" << std::endl;
	else
		std::cout << "==Open " << port_name_.toStdString() << " failed!" << std::endl;

	baud_rate_ = config["BaudRate"].as<int>();
	flow_ctrl_ = config["FlowCtrl"].as<bool>();
	parity_chk_ = config["Parity"].as<bool>();
	data_bit_len_ = config["DataBit"].as<int>();
	stop_bit_len_ = config["StopBit"].as<int>();
	frame_len_ = config["FrameLen"].as<int>();
	small_end_ = config["SmallEnd"].as<bool>();
	frame_rate_ = config["FrameRate"].as<int>();

	acc_resolution_ = config["AccResolution"].as<float>();
	gyro_resolution_ = config["GyroResolution"].as<float>();
	gravrity_ = config["Gravrity"].as<float>();

	log_data_ = config["LogData"].as<bool>();

	std::cout << "--Data Frame Length is: " << frame_len_ << std::endl;

	if(baud_rate_ == 115200)
	{
		std::cout << "--Port Baud Rate is: " << baud_rate_ << std::endl;
		port_ptr_->setBaudRate(BAUD115200);
	}

	if(flow_ctrl_ == true)
	{
		std::cout << "--Do Flow Control " << std::endl;
		port_ptr_->setFlowControl(FLOW_HARDWARE);
	}
	else
	{
		std::cout << "--Do not Flow Control " << std::endl;
		port_ptr_->setFlowControl(FLOW_OFF);
	}

	if(parity_chk_ == false)
	{
		std::cout << "--Do no do Parity Check! " << std::endl;
		port_ptr_->setParity(PAR_NONE);
	}
	else
	{
		std::cout << "--Do Parity Check!" << std::endl;
	}

	switch(data_bit_len_)
	{
		case 7: 
			port_ptr_->setDataBits(DATA_7);
			break;
		case 8: 
			port_ptr_->setDataBits(DATA_8); 
			break;
		default: 
			port_ptr_->setDataBits(DATA_8); break;
	}
	std::cout << "--Data Bit Length is: " << data_bit_len_ << std::endl;

	switch(stop_bit_len_)
	{
		case 0: break;
		case 1: port_ptr_->setStopBits(STOP_1); break;
		case 2: port_ptr_->setStopBits(STOP_2); break;
		default: 
			port_ptr_->setStopBits(STOP_1); break;
	}
	std::cout << "--Stop Bit Length is: " << stop_bit_len_ << std::endl;

	std::cout << "--IMU Frame Rate is: " << frame_rate_ << std::endl;

	if(log_data_)
	{
		imu_writer_.open(pkg_path + "/data/imu.txt");
	}

}

AdisDriver::~AdisDriver()
{
	if(log_data_)
	{
		imu_writer_.close();
	}
}

bool AdisDriver::ReceiveData(IMU_Data &imu_data)
{
	QByteArray frame;

	if(port_ptr_->bytesAvailable() >= 2)
	{
		frame.resize(2);
		port_ptr_->read(frame.data(), 2);

		if(frame[0] == 0xAA && frame[1] == 0xBB)
		{
			if(port_ptr_->bytesAvailable() >= frame_len_ - 2)
			{
				frame.resize(frame_len_ - 2);
				port_ptr_->read(frame.data(), frame_len_ - 2);

				if(frame[frame_len_ - 4] == 0xCC && frame[frame_len_ - 3] == 0xDD)
				{
					std::cout << "Receive One Frame!" << std::endl;

					int imu_number = 0;
					char* p = (char*)&imu_number;
					if(small_end_)
					{
						p[0] = frame[0];
						p[1] = frame[1];
						p[2] = frame[2];
						p[3] = frame[3];
					}
					else
					{
						p[0] = frame[3];
						p[1] = frame[2];
						p[2] = frame[1];
						p[3] = frame[0];
					}

					imu_data.time = imu_number * 1.0 / frame_rate_;
					std::cout << "Current IMU TimeStamp is: " << imu_data.time << std::endl;

					for(int i = 0; i < 6; ++i)
					{
						short temp = 0;
						char* p = (char*)&temp;
						if(small_end_)
						{
							p[0] = frame[4+ 2*i];
							p[1] = frame[4+ 2*i+1];
						}
						else
						{
							p[0] = frame[4 + 2*i+1];
							p[1] = frame[4 + 2*i];
						}
						imu_data.data[i] = temp;
					}
					
					//change the unit of acc data
					std::cout << "Acc Data: ";
					for(int i = 0; i < 3; i++)
					{
						imu_data.data[i] = imu_data.data[i]*acc_resolution_*gravrity_;
						std::cout << imu_data.data[i] << " ";
					}
					std::cout << std::endl;

					// change the unit of gyro data
					std::cout << "Gyro data: ";
					for(int i = 0; i < 3; i++)
					{
						imu_data.data[3+i] = imu_data.data[3+i]*gyro_resolution_*3.1415926/180;
						std::cout << imu_data.data[3+i] << " ";
					}
					std::cout << std::endl;

					if(log_data_)
					{
						imu_writer_ << imu_data.time << " " << 
						imu_data.acc_x << " " << imu_data.acc_y << " " << imu_data.acc_z << " " <<
						imu_data.gyro_x << " " << imu_data.gyro_y << " " << imu_data.gyro_z << std::endl;
					}

					return true;
				}
			}
		}

	}
	
	return false;
}