#include <ros/ros.h>
#include <ros/console.h>
#include "poly_ros/obstacles.h"

#include <cmath>
#include <fstream>
#include <string>
#include <ctime>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>

#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>

using namespace mavros;
using namespace mavconn;
using namespace mavlink;

//#include <mavlink/v1.0/common/mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

//Список действий для шасси
enum Action { STAY, MOVE_FORWARD, TURN_LEFT, TURN_RIGHT};
#define ACTION_STAY 0
#define ACTION_MOVE_FORWARD 1
#define ACTION_TURN_LEFT 2
#define ACTION_TURN_RIGHT 3

//Допустимые отклонения при управлении шасси
#define MOVE_ACCURACY 0.1  //Если расстояние до цели меньше этого параметра, то считаем, что робот приехал в целевую точку
#define TURN_ACCURACY 0.2  //Если отклонение угла от желаемого меньше этого параметра, то считаем, что повернули робота в нужное положение, единица - радиан

//Храним информацию ещё и в новом продвинутом формате (надо будет перейти на него везде)
Eigen::Vector3d current_point, target_point;
Eigen::Quaterniond current_attitude, target_attitude;

//Рабочую информацию будем хранить в старом формате
double current_yaw=0.0;
double current_x=0.0, current_y=0.0, current_z=0.0;
double target_x=0.0, target_y=0.0, target_z=0.0;

//Флаги, в которых будет храниться отметка о наличии информации о своём положении и о целевой точке (0 - нет информации, 1 - есть)
double target_set;
double position_set;

bool obstructionClose = false;

//старая ошибка для ПД-регулятора
double errold = 0.0;

//перечислимый тип выбора П-, ПД-регулятора или релейного
enum controlMethod { relay=1, P_regulator, PD_regulator };
#define relay 1
#define P_regulator 3
#define PD_regulator 2

//константы для шасси
const int STOP = 1500;
const int MOVE = 1650;
const int LEFT = 1200;
const int RIGHT = 1800;

//коэффициенты регуляторов 
const double k=17.3;//коэффициент П-регулятора
const double kd=30.5;//коэффициент Д-регулятора

mavros_msgs::State current_state;
//Callback-функция - приёмник текущего состояния PixHawk контроллера
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/*
Callback-функция - прослушка сообщений, поступающих с центрального сервера по MAVLink,
в ней мы перехватываем сообщения с msgid=84 - это команда "Двигаться в заданные локальные координаты"
сообщение заданной цели и декодирование
*/
void to_px_cb(const mavros_msgs::Mavlink::ConstPtr& rmsg)
{
    mavlink_set_position_target_local_ned_t pos_ned;
    mavlink_message_t mmsg;

    ROS_INFO("gcs_recv: %d-%d", rmsg->sysid, rmsg->msgid);
    if(rmsg->msgid == 84)
    {
        mavros_msgs::mavlink::convert(*rmsg, mmsg); // from mavlink to ros
        mavlink_msg_set_position_target_local_ned_decode(&mmsg, &pos_ned); // decode mavlink to struct

	//Запишем целевые координаты в старом формате
        target_x = pos_ned.x;
        target_y = pos_ned.y;
        target_z = pos_ned.z;

	target_set = 1;

        ROS_INFO("target x,y=(%0.2f,%0.2f)", target_x, target_y);
    }
}

//определение своего положения в пространстве с лидара
void lidar_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    static double xx=2,yy=2,aa=2;

    //Запишем в продвинутом формате
    tf::pointMsgToEigen(msg->pose.position, current_point);
    tf::quaternionMsgToEigen(msg->pose.orientation, current_attitude);

    //double yaw=tf::getYaw(msg->pose.orientation);
    //ROS_INFO("Yaw=%0.2f",yaw);
    //current_yaw = msg->pose.orientation.z*M_PI;

    //Запишем в старом формате
    current_yaw = tf::getYaw(msg->pose.orientation);
    current_x =  msg->pose.position.x;
    current_y =  msg->pose.position.y;
    current_z =  msg->pose.position.z;

    if(fabs(aa-current_yaw)>0.2 || hypot(xx-current_x,yy-current_y)>0.2)
    {
        ROS_INFO("current x,y=(%0.2f,%0.2f) yaw=%0.2f",current_x,current_y,current_yaw);
        aa=-current_yaw;
        xx=current_x;
        yy=current_y;
    };
	//Запишем, что теперь знаем свои координаты
    position_set=1;

}

//отправка текущего место-положения робота 
int send_local_pos(mavros_msgs::Mavlink &rmsg)
{
    mavlink_message_t mmsg;
    ROS_INFO ("%0.2f, %0.2f", current_x, current_y);
    mavlink_msg_local_position_ned_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, current_x, current_y, current_z, 0, 0, 0);
    mavros_msgs::mavlink::convert(mmsg, rmsg);

    return 1;
}

// Функция формирования MAVLink сообщения о наших текущих углах (положение в пространстве в смысле поворотов)
int send_attitude(mavros_msgs::Mavlink &rmsg)
{
    mavlink_message_t mmsg;

    double current_roll=0;
    double current_pitch=0;

    mavlink_msg_attitude_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, current_roll, current_pitch, current_yaw, 0, 0, 0);
    mavros_msgs::mavlink::convert(mmsg, rmsg);

    return 1;
}

//приведение угла к диапазону -Pi<angle<=+Pi, для того чтобы не поворачивать через полный угол (для разворота по кратчайшей траектории)
double normalizeAngle(double angle) 
{
  while(angle>M_PI)
	{
		angle-=2*M_PI; 
	};
  while(angle<=-M_PI)
	{ 
		angle+=2*M_PI; 
	};
  return angle;
}

/*
	алгоритм, который выдает(возвращает) конкретное действие для робота, на основе его текущего положения и заданной цели;
	т.е. выдает набор команд роботу, чтобы он смог добраться до цели
*/
int getAction(double current_x, double current_y, double current_angle, double target_x, double target_y, double *local_angle_to_target) 
{

    double dx=target_x-current_x; //разница между заданной координатой цели и текущим положением (по X)
    double dy=target_y-current_y; //разница между заданной координатой цели и текущим положением (по Y)
    double distance_to_target = hypot(dx,dy); //считаем расстояние между текущим положением робота и заданной координатой (гипотенуза треугольника)
    double global_angle_to_target=atan2(dy, dx); //глобальный угол к цели
     *local_angle_to_target=normalizeAngle(global_angle_to_target - current_angle); //текущий угол относително заданной цели

	int action;
	
	//если расстояние до цели меньше 0.1(допустимого отклонения), то будем считать, что мы добрались до цели и нужно остановиться 
    if( distance_to_target<MOVE_ACCURACY )
	{ 
		action = ACTION_STAY; //приехали 
	}; 
	
	/*
		если мы не у цели, то проверим, как расположен робот(куда напревлен) относительно заданной точки
		если угол между положением робота и целью меньше 0.2(допустимого отклонения), то будем считать, что робот направлен(смотрит) на цель 
	*/
    if(fabs(*local_angle_to_target)<TURN_ACCURACY)
	{ 
		action = ACTION_MOVE_FORWARD; //едем прямо
	};
	
	//если текущий угол робота относительно цели больше 0, то нужно повернуть налево
    if(*local_angle_to_target>0)
	{ 
		action = ACTION_TURN_LEFT; //поворачиваем налево 
	}
	else
     {
        action = ACTION_TURN_RIGHT; //поворачиваем направо
     }
	 
	return action; 
}

/*
	алгоритм, реализующий управяющее воздействие
	основанный на Пропорциональном или Пропорционально-дифференциальном регуляторе, в зависимости от выбора пользователя
*/
double controlAction(int algorithm, int currentAction, double *local_angle_to_target)
{
	double u; 
	//если робот не стоит и не едет прямо, то будем считать управлюящее воздействие для движения
	if (currentAction != ACTION_STAY && currentAction != ACTION_MOVE_FORWARD)
	{
		double err = *local_angle_to_target; //ошибка между заданной целью робота и его текущем положением
			switch (algorithm)
			{
				case PD_regulator: //выбор  ПД-регулятора по флагу
					u = k * err + kd *(err - errold);//вычисление управляющей Пропорционально-дифференциального регулятора
					errold = err;//запоминаем предыдущую ошибку
					break;
				case P_regulator://выбор  П-регулятора по флагу
					u = k*err;
					break;
				default:
					u = 0.0;
			}

		//Если нам нужен конкретный поворот, а знак управляющей не совпадает
		if((currentAction == ACTION_TURN_LEFT && u > 0) || (currentAction == ACTION_TURN_RIGHT && u < 0))
		{
			u = -u;
		}
	}
	else
	{
		u = 0.0;
	};
	return u;
}

/*
	алгоритм перевод команд для робота в конкретное управление гусеницами
	т.е. от того, стоит робот, едет прямо или поворачивает будет подаваться управление на шасси, благодаря чему происходит движение робота к цели 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc)
/*
rc.channels[0]-отвечает за поворот,
rc.channels[2]-отвечает за движение гусениц вперёд
значения от 1000 до 2000
1500-стоим на месте
больше 1500- гусеницы движутся по часовой стрелке
меньше 1500- гусеницы движутся против часовой стрелки
*/
{
	//если выбран релейный регулятор
	if (algorithm == relay )
	{
	    if (obstructionClose)
	    {
		currentAction = ACTION_STAY;
		ROS_INFO("STAY");
	    }
 
		switch (currentAction)
		{
		case ACTION_MOVE_FORWARD: //едем прямо
			rc.channels[0] = STOP;
			rc.channels[2] = MOVE;
			break;
		case ACTION_TURN_LEFT: //поворачиваем налево
			rc.channels[0] = LEFT;
			rc.channels[2] = STOP;
			break;
		case ACTION_TURN_RIGHT: //поворачиваем направо
			rc.channels[0] = RIGHT;
			rc.channels[2] = STOP;
			break;
		default: //стоим
			rc.channels[0] = STOP;
			rc.channels[2] = STOP;

		}
	}
	else //действуем по алгоритмам П- и ПД-регуляторов
	{
	    if (obstructionClose)
	    {
		rc.channels[0] = STOP;
		rc.channels[2] = STOP;
		ROS_INFO("STAY");
	    }
	    else
	    {
		//если робот не едет прямо, то добавим управляюще воздейсвтие к шасси, чтобы он развернулся к цели
		if (currentAction != ACTION_MOVE_FORWARD)
		{
			/*
			округлим управляющую составляющую и прибавим её к rc.channels[0], который отвечает за поворот
			управляющая control будет иметь знак "+" или "-"(зависит от того, как повернулся робот и какой угол образовался между ним и целью)
			и в зависимости от этого будет происходить поворот налево или направо
			если у control знак "+", то направо
			если у control знак "-", то налево

			*/
			rc.channels[0] = STOP + round(control);
			rc.channels[2] = STOP;
		}
		else//робот поедет прямо без управляющей составляющей
		{
			rc.channels[0] = STOP;
			rc.channels[2] = MOVE;
		};
	    };
	}
}

void  chatterCallback(const poly_ros::obstacles::ConstPtr& mas) //new new new
{
    obstructionClose = false;
    float k=9999;
    for (int i=0; i< mas->num; i++)
    {
	//ROS_INFO("(num: %d, degree1: %0.2f, degree2: %0.2f, min_distance: %0.2f)", mas->mass[i].nomer, mas->mass[i].angle1, mas->mass[i].angle2, mas->mass[i].min_distance);
	if (mas->mass[i].min_distance < k)
	    k = i;
    }
    if ((mas->mass[k].angle1 < 30 || mas->mass[k].angle2 >330) && ( mas->mass[k]. min_distance < 0.33))
	obstructionClose = true;

    ROS_INFO( "(number of obstacle: %d)",  mas->num);
}

int main(int argc, char **argv)
{
	//Инициализация ROS-ноды, должна быть в самом начале
    ros::init(argc, argv, "poly_ros_node");
    ros::NodeHandle nh;

	//не известа цель и положение робота 
    target_set=0;
    position_set=0;

    ROS_INFO("Poly ON 1");
    ROS_DEBUG("Poly ON debug");

	//Подпишемся на топик статуса PixHawk контроллера
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
			
    ros::Subscriber sub = nh.subscribe("obstacles" , 1000, chatterCallback);  //new new new
			
	//Зарегистрируемся в качестве паблишера для топика, который передаёт  центральный сервер
    ros::Publisher to_gcs = nh.advertise<mavros_msgs::Mavlink>
            ("mavlink/from", 10);
			
	//Зарегистрируемся в качестве паблишера для топика, который передаёт информацию о RC-командах (*1) на PixHawk
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 1);
			
//    ros::Subscriber sub = nh.subscribe("bolt" , 1000, chatterCallback); //new new new

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
				if( current_state.mode != "MANUAL" ){
				ROS_INFO("Try enable MANUAL mode");
				if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){ ROS_INFO("Manual enabled"); };
			};
			if( !current_state.armed && current_state.mode == "MANUAL" ){
				ROS_INFO("Try arm vehicle");
				if( arming_client.call(arm_cmd) && arm_cmd.response.success){ ROS_INFO("Vehicle armed"); };
					};
				
				last_request = ros::Time::now();
			};

		//Сообщим на GCS своё положение в пространстве
			if(send_local_pos(rmsg)) to_gcs.publish(rmsg);
			if(send_attitude(rmsg)) to_gcs.publish(rmsg);

		double local_angle_to_target; //текущий угол относително заданной цели
		int currentAction = ACTION_STAY; //текущее положение робота:стоит
		double control = 0.0;//управляющее воздействие
		int  algorithm = relay; //выбранный алгоритм движения

		//Если знаем, где мы и куда нам надо - предпримем какие-то действия, если нужно
		if( target_set && position_set )
			
			//узнаем, исходя из координат  робота, координат заданной цели и угла робота относительно цели, какое действие нужно предпринять, чтобы оказаться в заданной точке
			currentAction=getAction(current_x, current_y, current_yaw, target_x, target_y, &local_angle_to_target);
			
			//взависимости от полученного действия рассчитаем упраляющее воздействие на шасси 			
			control=controlAction(algorithm, currentAction, &local_angle_to_target);
			
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
