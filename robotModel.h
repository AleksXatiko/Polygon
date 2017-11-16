#pragma once

#include "poly_ros/obstacles.h"
#include <cmath>

#define INFRONT 1
#define BEHIND 2
#define ASIDE 3

float p1, p2, p3, p4;
float left_caterpillar_width, right_caterpillar_width, left_caterpillar_length, right_caterpillar_lenght;
float distance_between_caterpillar, caterpillar_offset;
float lidar_x, lidar_y, lidar_offset_x, lidar_offset_y, error_x, error_y;S
float w_x, w_y, s_x, s_y, r, l, q;

//Проверка наличия препятствия в радиуе обзора
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2, int side);

