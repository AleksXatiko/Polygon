#include "communication.h"
#include "computationMovement.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>

bool obstructionClose = false;
mavros_msgs::State current_state;
int moveLocalCoordinates;
int range1, range2;
double distinction, min_distance; 

//Callback-функци€ - приЄмник текущего состо€ни€ PixHawk контроллера
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/*
Callback-функци€ - прослушка сообщений, поступающих с центрального сервера по MAVLink,
в ней мы перехватываем сообщени€ с msgid=84 - это команда "ƒвигатьс€ в заданные локальные координаты"
сообщение заданной цели и декодирование
*/
void to_px_cb(const mavros_msgs::Mavlink::ConstPtr& rmsg)
{
    mavlink_set_position_target_local_ned_t pos_ned;
    mavlink_message_t mmsg;

    ROS_INFO("gcs_recv: %d-%d", rmsg->sysid, rmsg->msgid);
    if(rmsg->msgid == moveLocalCoordinates)
    {
        mavros_msgs::mavlink::convert(*rmsg, mmsg); // from mavlink to ros
        mavlink_msg_set_position_target_local_ned_decode(&mmsg, &pos_ned); // decode mavlink to struct

	//«апишем целевые координаты в старом формате
        target_x = pos_ned.x;
        target_y = pos_ned.y;
        target_z = pos_ned.z;

	target_set = 1;

        //ROS_INFO("target x,y=(%0.2f,%0.2f)", target_x, target_y);
    }
}

//определение своего положени€ в пространстве с лидара
void lidar_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    static double xx=2,yy=2,aa=2;

    //«апишем в продвинутом формате
    tf::pointMsgToEigen(msg->pose.position, current_point);
    tf::quaternionMsgToEigen(msg->pose.orientation, current_attitude);

    //double yaw=tf::getYaw(msg->pose.orientation);
    //ROS_INFO("Yaw=%0.2f",yaw);
    //current_yaw = msg->pose.orientation.z*M_PI;

    //«апишем в старом формате
    current_yaw = tf::getYaw(msg->pose.orientation);
    current_x =  msg->pose.position.x;
    current_y =  msg->pose.position.y;
    current_z =  msg->pose.position.z;

    if(fabs(aa-current_yaw) > distinction  || hypot(xx-current_x,yy-current_y) > distinction)
    {
        //ROS_INFO("current x,y=(%0.2f,%0.2f) yaw=%0.2f",current_x,current_y,current_yaw);
        aa=-current_yaw;
        xx=current_x;
        yy=current_y;
    };
	//«апишем, что теперь знаем свои координаты
    position_set=1;

}

//отправка текущего место-положени€ робота 
int send_local_pos(mavros_msgs::Mavlink &rmsg)
{
    auto stat = mavlink_get_channel_status(0);
    stat->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    mavlink_message_t mmsg;
    ROS_INFO ("%0.2f, %0.2f", current_x, current_y);
    mavlink_msg_local_position_ned_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, current_x, current_y, current_z, 0, 0, 0);
    mavros_msgs::mavlink::convert(mmsg, rmsg);

    return 1;
}

// ‘ункци€ формировани€ MAVLink сообщени€ о наших текущих углах (положение в пространстве в смысле поворотов)
int send_attitude(mavros_msgs::Mavlink &rmsg)
{
    mavlink_message_t mmsg;

    double current_roll=0;
    double current_pitch=0;

    mavlink_msg_attitude_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, current_roll, current_pitch, current_yaw, 0, 0, 0);
    mavros_msgs::mavlink::convert(mmsg, rmsg);

    return 1;
}

void  chatterCallback(const poly_ros::obstacles::ConstPtr& mas) //new new new
{
    obstructionClose = false;
	float min = 6.0;
    int k = 0;
    for (int i = 0; i < mas->num; i++)
    {
		if (obstacleCheck(mas, i, range1, range2, INFRONT))
		{
			if(mas->mass[i].min_distance < min)
			{
				min = mas->mass[i].min_distance;
				k = i;
			}
		}
    }
	if (mas->mass[k].min_distance < min_distance)
		obstructionClose = true;
    //ROS_INFO( "(number of obstacle: %d)",  mas->num);
}

/*void chatter(const poly_ros::Num::ConstPtr& a)
{
    ROS_INFO("(%0.2f)", a->num);
}*/
