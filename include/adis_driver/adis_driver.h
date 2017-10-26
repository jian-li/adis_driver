#include <iostream>
#include <QtExtSerialPort/qextserialport.h>
#include <QCoreApplication>
#include <fstream>
#include <string>

struct IMU_Data
{
	float time;
	union
	{
		struct 
		{	
			float acc_x;
			float acc_y;
			float acc_z;
			float gyro_x;
			float gyro_y; 
			float gyro_z;
		};
		float data[6];
	};
};

class AdisDriver
{
public:
	AdisDriver(const std::string config_file_path);
	~AdisDriver();

	bool ReceiveData(IMU_Data &imu_data);

private:
	QString port_name_;
	QextSerialPort *port_ptr_;

	unsigned int baud_rate_;
	bool flow_ctrl_;
	bool parity_chk_;
	unsigned int data_bit_len_;
	unsigned int stop_bit_len_;
	unsigned int frame_len_;

	float acc_resolution_;
	float gyro_resolution_;
	float gravrity_;

	bool small_end_;
	int frame_rate_;

	// Log data to txt
	bool log_data_;
	std::ofstream imu_writer_;
};