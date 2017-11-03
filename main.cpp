#include "communication.h"
#include "power.h"
#include "computationMovement.h"
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <ctime>
#include "poly_ros/obstacles.h"
//#include "poly_ros/Num.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int algorithm, pos_x, pos_y;
int regular;
int moveLocalCoordinates, range1, range2, zero;
double distinction;
double min_distance;

int main(int argc, char **argv)
{
	//Инициализация ROS-ноды, должна быть в самом начале
    ros::init(argc, argv, "main");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

	nhPrivate.param("pos_x", pos_x, 0);
	nhPrivate.param("pos_y", pos_y, 0);
	nhPrivate.param("algorithm", algorithm, 1);
    nhPrivate.param("regular", regular, 1);
    nhPrivate.param("movelocalCoorginates", moveLocalCoordinates, 84);
    nhPrivate.param("distinction", distinction, 0.2);
    nhPrivate.param("range1", range1, 50);
    nhPrivate.param("range2", range2, 250);
    nhPrivate.param("min_distance", min_distance,  0.45);
    nhPrivate.param("zero", zero, 0);
    ROS_INFO("movee=%d",regular );

	//не известа цель и положение робота 
    target_set=0;
    position_set=0;

    ROS_INFO("Poly ON 1");
    ROS_DEBUG("Poly ON debug");

	//Подпишемся на топик статуса PixHawk контроллера
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
			
    ros::Subscriber sub = nh.subscribe("obstacles" , 1000, chatterCallback);  //new new new
			
    //ros::Subscriber subs = nh.subscribe("Num", 1000, &chatter);
	//Зарегистрируемся в качестве паблишера для топика, который передаёт  центральный сервер
    ros::Publisher to_gcs = nh.advertise<mavros_msgs::Mavlink>
            ("mavlink/from", 10);
			
	//Зарегистрируемся в качестве паблишера для топика, который передаёт информацию о RC-командах (*1) на PixHawk
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 1);
			

    //Установим частоту, с которой будут обрабатываться данные в нашей ноде в 20Гц (должно быть не менее 2Гц)
    ros::Rate rate(20.0);

    ROS_INFO("Wait for FCU connection...");

    
	//Подождём, пока не подключимся к PixHawk
    while(ros::ok() && !current_state.connected)
	{
       ros::spinOnce();
       rate.sleep();
	};

    ROS_INFO("FCU connected");
	//Подпишемся на топик c MAVLink-командами, поступающими с центрального сервера
    ros::Subscriber to_px = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/to", 10, to_px_cb);
			
	//Подпишемся на топик c нашим текущим положением, которое публикует лидар
    ros::Subscriber lidar_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("slam_out_pose", 10, lidar_cb);
			
	//Подключим сервис для управление ARMED состоянием контроллера PixHawk (защитный механизм, если !ARMED, то двигатели отключены)
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
			
	//Подключим сервис для управление MODE состоянием контроллера PixHawk (выбор режима управления PixHawk - автопилот, ручной и т.д.)
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	//Подготовим команду для PixHawk для перевода в MODE=MANUAL для последующего управления прямыми RC-командами (*1)
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";
	
	//Подготовим команду для PixHawk для перевода в ARMED-режим для последующего управления прямыми RC-командами (*1)
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	
	//Тут будем хранить последнее время отправки запроса на смену режима PixHawk контроллера, чтобы не слать этот запрос слишком часто
    ros::Time last_request = ros::Time::now();

	//Подготовим переменные для работы
    mavros_msgs::OverrideRCIn rc; //Тут будем формировать прямые RC-команды для отправки на PixHawk
    geometry_msgs::PoseStamped local_pose;//Тут будем
    mavros_msgs::Mavlink rmsg; //Тут будем формировать сообщение для отправки на центральный сервер (наше текущее положение)
	
	//Переключим MAVLink в режим отправки сообщений по 1.0 версии протокола MAVLink.
    auto stat = mavlink_get_channel_status(0);
    stat->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;


    if( current_state.mode == "MANUAL" )
    {
	ROS_INFO("Start state: MANUAL mode enabled!");
    }
    else
    {
	ROS_INFO("Start state: No MANUAL mode enabled!");
    };
    if( current_state.armed )
    {
	ROS_INFO("Start state: Vehicle ARMED!");
    }
    else
    {
	ROS_INFO("Start state: Vehicle NOT ARMED!");
    };

    while(ros::ok())
	{
	    //Если не режим контроллера не соответствует MANUAL или он не ARMED, то попробуем перевести его в нужное состояние.
	    if(ros::Time::now() - last_request > ros::Duration(2.0)){
		if( current_state.mode != "MANUAL" )
		{
			ROS_INFO("Try enable MANUAL mode");
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){ ROS_INFO("Manual enabled"); };
		};
			if( !current_state.armed && current_state.mode == "MANUAL" ){
				ROS_INFO("Try arm vehicle");
			if( arming_client.call(arm_cmd) && arm_cmd.response.success){ 
				ROS_INFO("Vehicle armed"); };
			};
				
				last_request = ros::Time::now();
			};

		//Сообщим на GCS своё положение в пространстве
			if(send_local_pos(rmsg)) to_gcs.publish(rmsg);
			if(send_attitude(rmsg)) to_gcs.publish(rmsg);

		double local_angle_to_target; //текущий угол относително заданной цели
		int currentAction = ACTION_STAY; //текущее положение робота:стоит
		double control = 0.0;//управляющее воздействие
		//algorithm = 3; //выбранный алгоритм движения
		//ROS_INFO("ALGORITHM %d", algorithm);

		//Если знаем, где мы и куда нам надо - предпримем какие-то действия, если нужно
		if( target_set && position_set )
			
			if (pos_x != 0 || pos_y != 0)
			{
				target_x = pos_x;
				target_y = pos_y;
			}
			//узнаем, исходя из координат  робота, координат заданной цели и угла робота относительно цели, какое действие нужно предпринять, чтобы оказаться в заданной точке
			currentAction = getAction(current_x, current_y, current_yaw, target_x, target_y, &local_angle_to_target);
			
			//взависимости от полученного действия рассчитаем упраляющее воздействие на шасси 			
			control = controlAction(algorithm, currentAction, local_angle_to_target);
			
			//получив конкретное действие для достижения заданой цели, подадим роботу команды на шасси, чтобы он начал(или закончил) движение
			updateRCfromAction(algorithm, control, currentAction,rc);
			rc_pub.publish(rc);
			
			//Обработаем всё что нужно ROS
			ros::spinOnce();
			
			//Выждем паузу для нужной нам частоты работы (см. определение переменной rate выше) и продолжим работу
			rate.sleep();
	}
    return 0;
}
