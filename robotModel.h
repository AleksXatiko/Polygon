#pragma once

#include "poly_ros/obstacles.h"

//Проверка наличия препятствия в радиуе обзора впереди робота
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2);

