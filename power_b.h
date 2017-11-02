#pragma once

#include <mavros_msgs/OverrideRCIn.h>
#include "ros/ros.h"

//старая ошибка для ПД-регулятора
double errold = 0.0;

//перечислимый тип выбора П-, ПД-регулятора или релейного
enum controlMethod { relay=1, P_regulator, PD_regulator };
#define relay 1
#define P_regulator 3
#define PD_regulator 2
extern bool obstructionClose;

//константы для шасси
const int STOP = 1500;
const int MOVE = 1590;
const int LEFT = 1200;
const int RIGHT = 1800;

//коэффициенты регуляторов 
const double k = 17.3;//коэффициент П-регулятора
const double kd = 30.5;//коэффициент Д-регулятора

/*
	алгоритм, реализующий управяющее воздействие
	основанный на Пропорциональном или Пропорционально-дифференциальном регуляторе, в зависимости от выбора пользователя
*/
double controlAction(int algorithm, int currentAction);

/*
	алгоритм перевод команд для робота в конкретное управление гусеницами
	т.е. от того, стоит робот, едет прямо или поворачивает будет подаваться управление на шасси, благодаря чему происходит движение робота к цели 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc);