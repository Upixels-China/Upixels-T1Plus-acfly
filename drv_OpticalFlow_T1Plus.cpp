/*
 *  T1Plus support for acfly
 *  Update on: Feb 12, 2025
 * 	E-mail: pzhy521@163.com
 *      Author: StrangeFly@github
 */

//驱动头文件，用于飞控初始化时注册此驱动
#include "drv_OpticalFlow_T1Plus.hpp"

//飞控接口头文件以及FreeRTOS相关头文件
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"

//对地测距传感器编号
#define SensorInd 2

bool status = 0;
bool port_status = 0;

//定义传感器参数结构体
struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key_opf;
	uint32_t sensor_key_tof;
};

//定义串口通信数据包结构体
typedef struct
{
	int16_t flow_x_integral;		// X 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	int16_t flow_y_integral;		// Y 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	uint16_t integration_timespan;	// 上一次发送光流数据到本次发送光流数据的累计时间（us）
	uint16_t ground_distance; 		// TOF测距距离(mm)
	uint8_t valid;					// 光流置信度:0(0x00)为光流数据不可用
									// 245(0xF5)为光流数据可用
	uint8_t tofval; 				// TOF测距置信度
}__PACKED _Flow;

//数据包帧头，二字节
static const unsigned char packet_ID[2] = { 0xfe , 0x0a };

//光流驱动主程序函数
static void OpticalFlow_Server(void* pvParameters)
{
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;

	//进入循环
	while(1)
	{
		uint8_t rdata;
		int tof_i = 0;
		port_status = 0;

		//读取串口数据
		if( driver_info.port.read( &rdata, 1, 0.02, 0.02 ) )
		{
			if( rc_counter < 2 )
			{	//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 12 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				sum ^= (signed char)rdata;
				++rc_counter;
			}
			else if( rc_counter == 12 )
			{	//校验
				if( sum != rdata )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == 0x55 )
				{
					vector3<double> position;

					//如果TOF可用
					if( Flow.tofval != 0){
						
						//赋值，将mm转换为cm，飞控接口单位为cm
						position.z = Flow.ground_distance*0.1;
						port_status = 1;

						//获取飞机倾角
						Quaternion quat;
						get_Airframe_quat( &quat );
						double lean_cosin = quat.get_lean_angle_cosin();

						//更新距离，补偿飞机倾角导致的距离误差
						position.z *= lean_cosin;

						//更新定高传感器状态及数据
						PositionSensorUpdatePosition( SensorInd,driver_info.sensor_key_tof, position, true );
					}else{

						//如数据不可用注销传感器
						PositionSensorSetInavailable( SensorInd,driver_info.sensor_key_tof );
					}

					//PosSensorHealthInf1 ZRange_inf;
					//get_OptimalRange_Z( &ZRange_inf )
					if( Flow.valid != 0 &&  Flow.tofval != 0 )
					{	//测距传感器可用
						//获取TOF高度(补偿后)
						double height = position.z;

						//获取角速度
						vector3<double> AngularRate;
						get_AngularRate_Ctrl( &AngularRate );

						//补偿光流
						double rotation_compensation_x = -constrain( AngularRate.y * 10000 , 4500000000.0 );
						double rotation_compensation_y = constrain(  AngularRate.x * 10000 , 4500000000.0 );
						double integral_time = (Flow.integration_timespan * 1e-6f);
						double temp_flow_x, temp_flow_y;
						double flow_x, flow_y;
						temp_flow_x = Flow.flow_x_integral;
						temp_flow_y = -Flow.flow_y_integral;

						//传感器安装偏移
						switch(driver_info.param)
						{
							case 0:
							default:
							{
								flow_x = temp_flow_x;
								flow_y = temp_flow_y;
								break;
							}
							case 1:
							{
								flow_x = temp_flow_y;
								flow_y = -temp_flow_x;
								break;
							}
							case 2:
							{
								flow_x = -temp_flow_x;
								flow_y = -temp_flow_y;
								break;
							}
							case 3:
							{
								flow_x = -temp_flow_y;
								flow_y = temp_flow_x;
								break;
							}
						}
						
						integral_time = 1.0 / integral_time;

						//根据TOF和光流数据计算当前速度
						vector3<double> vel;
						vel.x = ( flow_x*integral_time - rotation_compensation_x ) * 1e-4f * ( 1 + height );
						vel.y = ( flow_y*integral_time - rotation_compensation_y ) * 1e-4f * ( 1 + height ) ;

						//更新光流传感器状态及数据
						PositionSensorUpdateVel( default_optical_flow_index,driver_info.sensor_key_opf, vel , true );
					}else{
						//如数据不可用则注销传感器
						PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key_opf );
					}

					//重置参数
					Flow.flow_x_integral = 0;
					Flow.flow_y_integral = 0;
					Flow.ground_distance = 0;
					Flow.integration_timespan = 0;
					Flow.tofval = 0;
					Flow.valid = 0;
				}
				rc_counter = 0;
			}
		}
	}
}

//传感器配置函数
static bool OpticalFlow_DriverInit( Port port, uint32_t param )
{
	//串口波特率115200
	port.SetBaudRate( 115200, 1, 1 );

	//注册光流传感器
	uint32_t sensor_key_opf = PositionSensorRegister( default_optical_flow_index , \
																								"Upixels_OpticalFlow" ,\	//传感器名称
																								Position_Sensor_Type_RelativePositioning , \	//传感器类型
																								Position_Sensor_DataType_v_xy_nAC , \	//传感器数据类型
																								Position_Sensor_frame_BodyHeading , \	//传感器坐标系
																								0.1, 100 );				//延时，xy置信度	
	//注册失败																																				
	if( sensor_key_opf==0 )
		return false;

	//注册TOF传感器
	uint32_t sensor_key_tof = PositionSensorRegister( SensorInd , \
																								"Upixels_TOF" ,\	//传感器名称
																								Position_Sensor_Type_RangePositioning , \	//传感器类型
																								Position_Sensor_DataType_s_z , \	//传感器数据类型
																								Position_Sensor_frame_ENU , \	//传感器坐标系
																								0.01 , //延时
																								0 ,	//xy信任度
																								0 //z信任度			
																								);			
	//注册失败																																				
	if( sensor_key_tof==0 )
		return false;

	//注册参数
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key_opf = sensor_key_opf;
	driver_info->sensor_key_tof = sensor_key_tof;

	//创建FreeRTOS任务，开始调度任务，启动T1Plus驱动
	xTaskCreate( OpticalFlow_Server, "Upixels_T1Plus", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

//传感器驱动注册调用接口
void init_drv_OpticalFlow_T1Plus()
{
	//配置传感器，设置传感器编号为32(请视情况调整此编号值)
	PortFunc_Register( 34, OpticalFlow_DriverInit );
}