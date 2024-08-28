#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include "sensor_msgs/msg/imu.hpp"

/* ROM ADD */
extern "C" {
  #include "wit_node/serial.h"
  #include "wit_node/REG.h"
  #include "wit_node/wit_c_sdk.h"
}
#include <stdint.h>

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);

unsigned char * port = (unsigned char *)"/dev/ttyUSB0";

float fAcc[3], fGyro[3], fAngle[3];
int i , ret;
unsigned char cBuff[1];

/* ROM END */

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    if((fd = serial_open(port , 9600)<0))
	    {
	      printf("open /dev/ttyUSB0 fail\n");
	    }

      WitInit(WIT_PROTOCOL_NORMAL, 0x50);
      //printf("WIT_PROTOCOL_NORMAL OK\n");
	    WitRegisterCallBack(SensorDataUpdata);
      //printf("SensorDataUpdata ok\n");

    // Create a node
    auto node = rclcpp::Node::make_shared("minimal_publisher");

    // Create a publisher on a topic 'topic' with message type 'std_msgs::msg::String'
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);

    rclcpp::Rate loop_rate(10);

    while(1)
    {
      while(serial_read_data(fd, cBuff, 1))
		  {
		      WitSerialDataIn(cBuff[0]);
		  }
		  printf("\n");
          Delayms(500);
        //loop_rate.sleep();
        if(s_cDataUpdate)
		   {
			   for(i = 0; i < 3; i++)
			    {
				    fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				    fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			    }

  			  if(s_cDataUpdate & ACC_UPDATE)
			    {
				   printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				   s_cDataUpdate &= ~ACC_UPDATE;
		   	    }
			  if(s_cDataUpdate & GYRO_UPDATE)
			    {
				   printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				   s_cDataUpdate &= ~GYRO_UPDATE;
			    }
			  if(s_cDataUpdate & ANGLE_UPDATE)
			    {
				   printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
				   s_cDataUpdate &= ~ANGLE_UPDATE;
			    }
			  if(s_cDataUpdate & MAG_UPDATE)
			    {
				   printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				   s_cDataUpdate &= ~MAG_UPDATE;
			    }
		    }

      auto msg = std_msgs::msg::String();
      msg.data = "rom dynamics";
      publisher->publish(msg);
      RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.data.c_str());
      
    }

    // Spin the node so it can keep running
    rclcpp::spin(node);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
