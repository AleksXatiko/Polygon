#pragma once

#include <mavros_msgs/OverrideRCIn.h>
#include "ros/ros.h"

//старая ошибка для ПД-регулятора
double errold = 0.0;
double i_old = 0.0;

//перечислимый тип выбора П-, ПД-, ПИД-регулятора или релейного
enum controlMethod { relay=1, P_regulator, PD_regulator };
#define relay 1
#define P_regulator 2
#define PD_regulator 3
#define PID_regulator 4
extern bool obstructionClose;

//константы для шасси
const int STOP = 1500;
const int MOVE = 1600;
const int LEFT = 1200;
const int RIGHT = 1800;

//коэффициенты регуляторов 
const double kp = 221;//коэффициент П-регулятора
const double kd = 180;//коэффициент Д-регулятора
const double ki = 0.03;//коэффициент И-регулятора
const int kl = -30;//коэффициент для левой гусеницы
const int kr = 70;//коэффициент для правой гусеницы

/*
	алгоритм, реализующий управяющее воздействие
	основанный на Пропорциональном или Пропорционально-дифференциальном регуляторе, в зависимости от выбора пользователя
*/
double controlAction(int algorithm, int currentAction, double local_angle_to_target);

/*
	алгоритм перевод команд для робота в конкретное управление гусеницами
	т.е. от того, стоит робот, едет прямо или поворачивает будет подаваться управление на шасси, благодаря чему происходит движение робота к цели 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc);