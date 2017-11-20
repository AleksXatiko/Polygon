#pragma once

#include "poly_ros/obstacles.h"
#include <cmath>

#define INFRONT 1
#define BEHIND 2
#define ASIDE 3

//Проверка наличия препятствия в радиуе обзора
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2, int side);

double GetW_X(double x1, double x2, double dx); //находим расстояние от центра робота до центров гусениц по оси X

double GetW_Y(double dy);	//находим расстояние от центра робота до центров гусениц по оси Y

double GetS_X(double lid_x, double x1, double w_x, double p4);	//смещение сенсора по оси X

double GetS_Y(double lid_y, double y1, double w_y, double p1);	//смещение сенсора по оси Y

double Get_Lidar_Right_X(double p4, double x1, double x2, double dx, double p2, double lid_x);	//расстояние от центра лидара до правого края

double Get_Lidar_Back_Y(double p1, double y1, double dy, double p3, double lid_y);	//расстояние от центра лидара до заднего края

double Get_Safety_Zone(double w_x, double x, double p4, double c_x, double y, double dy, double p1, double p3, double c_y);	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой или левой стороны. это требуется для разворота с фиксированной гусеницей 

double Get_Safety_Tank_Zone(double w_x, double x2, double p2, double c_x, double y2, double w_y, double p1, double p3, double p4, double c_y);	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой и левой стороны. Это требуется для танкового разворота 

double Safety_Angle(double lid_y, double S, double lid_x, double c_x, double c_y); // общая формула для нахождения углов зоны безопасного движения вперед и назад