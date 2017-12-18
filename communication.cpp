#include "communication.h"
#include "computationMovement.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include "poly_ros/obstacles.h"
#include "poly_ros/robotModel_parametrs.h"
#include "poly_ros/target.h"

const double err = 0.02;

bool obstructionClose = false;
mavros_msgs::State current_state;
int moveLocalCoordinates, currentAction, pos_x, pos_y, move_mode;
double distinction, lidar_x, lidar_rx, lidar_y, lidar_by, min_distance, radius, x_offset, y_offset; 

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
    if(rmsg->msgid == moveLocalCoordinates && move_mode != 0)
    {
        mavros_msgs::mavlink::convert(*rmsg, mmsg); // from mavlink to ros
        mavlink_msg_set_position_target_local_ned_decode(&mmsg, &pos_ned); // decode mavlink to struct

	//«апишем целевые координаты в старом формате
        target_x = pos_ned.x;
        target_y = pos_ned.y;
        target_z = pos_ned.z;
		target_set = 1;

        ROS_INFO("target x,y=(%0.2f,%0.2f)", target_x, target_y);
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

void chatterCallback(const poly_ros::obstacles::ConstPtr& mas) //new new new
{
	float begin_x, begin_y, end_x, end_y;
	int i = 0;
    obstructionClose = false;
    while (!obstructionClose && i < mas->num)
    {
		begin_y = mas->mass[i].begin_y;
		begin_x = mas->mass[i].begin_x;
		end_y = mas->mass[i].end_y;
		end_x = mas->mass[i].end_x;
		if (currentAction == ACTION_TURN_LEFT || currentAction == ACTION_TURN_RIGHT)
		{
			begin_y += y_offset;
			begin_x += x_offset;
			end_y += y_offset;
			end_x += x_offset;
		}
		double k = (begin_y - end_y) / (begin_x - end_x);
		double b = begin_y - k * begin_x;
		
		if (obstacleCheck(mas, i, lidar_x, lidar_rx, lidar_y, lidar_by, currentAction, k, b, radius))
			obstructionClose = true;
		i++;
    }
}

void GetData(const poly_ros::robotModel_parametrs::ConstPtr& parametrs)
{
	lidar_x = parametrs->parametr[0] + err;
	lidar_rx = parametrs->parametr[1] + err;
	lidar_y = parametrs->parametr[2] + err;
	lidar_by = parametrs->parametr[3] + err;
	min_distance = parametrs->parametr[4];
	radius = parametrs->parametr[5];
	x_offset = parametrs->parametr[6];
	y_offset = parametrs->parametr[7];
	//ROS_INFO("EEEEEEEEEEE %0.3f %0.3f %0.3f %0.3f", (float)angle1, (float)angle2, (float)angle3, (float)angle4);
	//ROS_INFO("EEEEEEEEEEE %0.3f %0.3f", (float)min_distance, (float)radius);
}

void test(const poly_ros::target::ConstPtr& msg)
{
	if (move_mode == 0 && pos_x == 0 && pos_y == 0)
	{
		target_x = msg->x;
		target_y = msg->y;
		target_set = 1;
		//ROS_INFO("___________________G | %0.3f | %0.3f", msg->x, msg->y);
	}
}

//Проверка наличия препятствия в радиуе обзора
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, double lidar_x, double lidar_rx, double lidar_y, double lidar_by, int action, double k, double b, double rad)
{
	switch(action)
	{
		case ACTION_MOVE_FORWARD:
			return ((mas->mass[index].begin_x < lidar_x &&
					mas->mass[index].begin_x > -lidar_rx) || 
					(mas->mass[index].end_x < lidar_x &&
					mas->mass[index].end_x > -lidar_rx) ||
					(mas->mass[index].end_x > lidar_x &&
					mas->mass[index].begin_x < -lidar_rx)) && 
					mas->mass[index].min_distance < min_distance + lidar_y && b > 0;
		case ACTION_MOVE_BACK:
			return ((mas->mass[index].begin_x < lidar_x &&
					mas->mass[index].begin_x > -lidar_rx) || 
					(mas->mass[index].end_x < lidar_x &&
					mas->mass[index].end_x > -lidar_rx) ||
					(mas->mass[index].end_x > lidar_x &&
					mas->mass[index].begin_x < -lidar_rx)) && 
					mas->mass[index].min_distance < min_distance + lidar_by && b < 0;
		case ACTION_TURN_LEFT:
		case ACTION_TURN_RIGHT:
			return pow(radius, 2) * (pow(k, 2) + 1) - pow(b, 2) > 0 && mas->mass[index].min_distance < radius + lidar_x;
		default:
			return true;
		
	}
}

/*void chatter(const poly_ros::Num::ConstPtr& a)
{
    ROS_INFO("(%0.2f)", a->num);
}*/
