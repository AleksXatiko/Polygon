#pragma once

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//—писок действий дл€ дл€ гусениц робота, всего 4 положени€: стоим, едем пр€мо, поворачиваем налево, поворачиваем направо
enum Action { STAY, MOVE_FORWARD, TURN_LEFT, TURN_RIGHT };
#define ACTION_STAY 0
#define ACTION_MOVE_FORWARD 1
#define ACTION_TURN_LEFT 2
#define ACTION_TURN_RIGHT 3

//ƒопустимые отклонени€ при управлении шасси, рассчитанные на погрешности при движении(люфт, погрешность измерени€ лидара, помехи и т.п.)
#define MOVE_ACCURACY 0.1  //≈сли рассто€ние до заданной цели меньше этого параметра, то считаем, что робот приехал в заданную точку
#define TURN_ACCURACY 0.2  //≈сли отклонение текущего угла от заданного меньше этого параметра, то считаем, что робот направлен в сторону заданной цели; единица - радиан

//’раним информацию ещЄ и в новом продвинутом формате
Eigen::Vector3d current_point, target_point;
Eigen::Quaterniond current_attitude, target_attitude;

//–абочую информацию будем хранить в старом формате
double current_yaw=0.0;
double current_x=0.0, current_y=0.0, current_z=0.0;
double target_x=0.0, target_y=0.0, target_z=0.0;

//‘лаги, в которых будет хранитьс€ отметка о наличии информации о своЄм положении и о целевой точке (0 - нет информации, 1 - есть)
double target_set;
double position_set;

//приведение угла к диапазону -Pi<angle<=+Pi, дл€ того чтобы не поворачивать через полный угол (дл€ разворота по кратчайшей траектории)
double normalizeAngle(double angle); 

/*
	алгоритм, который выдает(возвращает) конкретное действие дл€ робота, на основе его текущего положени€ и заданной цели;
	т.е. выдает набор команд роботу, чтобы он смог добратьс€ до цели
*/
int getAction(double current_x, double current_y, double current_angle, double target_x, double target_y);
