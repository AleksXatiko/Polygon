#pragma once

#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>
#include "poly_ros/obstacles.h"
#include "poly_ros/Num.h"
#include "ros/ros.h"
#include "poly_ros/robotModel_parametrs.h"
#include "poly_ros/target.h"

using namespace mavros;
using namespace mavconn;
using namespace mavlink;

//#include <mavlink/v1.0/common/mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

extern mavros_msgs::State current_state;
extern int moveLocalCoordinates;

//Callback-функция - приёмник текущего состояния PixHawk контроллера
void state_cb(const mavros_msgs::State::ConstPtr& msg);

/*
Callback-функция - прослушка сообщений, поступающих с центрального сервера по MAVLink,
в ней мы перехватываем сообщения с msgid=84 - это команда "Двигаться в заданные локальные координаты"
сообщение заданной цели и декодирование
*/
void to_px_cb(const mavros_msgs::Mavlink::ConstPtr& rmsg);

//определение своего положения в пространстве с лидара
void lidar_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

//отправка текущего место-положения робота 
int send_local_pos(mavros_msgs::Mavlink &rmsg);

// Функция формирования MAVLink сообщения о наших текущих углах (положение в пространстве в смысле поворотов)
int send_attitude(mavros_msgs::Mavlink &rmsg);

void chatterCallback(const poly_ros::obstacles::ConstPtr& mas); //new

void GetData(const poly_ros::robotModel_parametrs::ConstPtr& parametrs);

void test(const poly_ros::target::ConstPtr& msg);

//void chatter(const poly_ros::Num::ConstPtr& a);

//Проверка наличия препятствия в радиуе обзора
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, double range1, double range2, int action, double dist, double rad, double ang);