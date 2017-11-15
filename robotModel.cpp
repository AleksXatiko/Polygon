#include "robotModel.h"

//Проверка наличия препятствия в радиуе обзора впереди робота
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2)
{
	return  mas->mass[index].angle1 < range1 || 
			mas->mass[index].angle1 > range2 || 
			mas->mass[index].angle2 < range1 || 
			mas->mass[index].angle2 > range2 || 
			mas->mass[index].angle1 > mas->mass[index].angle2;
}

