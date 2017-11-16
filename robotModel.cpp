#include "robotModel.h"

//Проверка наличия препятствия в радиуе обзора
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2, int side)
{
	switch(side)
	{
		case INFRONT:
			return  mas->mass[index].angle1 < range1 || 
					mas->mass[index].angle1 > range2 || 
					mas->mass[index].angle2 < range1 || 
					mas->mass[index].angle2 > range2 || 
					mas->mass[index].angle1 > mas->mass[index].angle2;
		case BEHIND:
			return  mas->mass[index].angle1 > range1 && 
					mas->mass[index].angle1 < range2 || 
					mas->mass[index].angle2 > range1 && 
					mas->mass[index].angle2 < range2 || 
					mas->mass[index].angle1 < mas->mass[index].angle2;
		case ASIDE:
			
		
	}
}

float GetW_X(float x1, float x2, float dx)
{
	return (x1/2 + x2/2 + dx)/2;
}

float GetW_Y(float dy)
{
	return abs(dy)/2;
}

float GetS_X(float lid_x, float x1, float w_x, float p4)
{
	return lid_x - (x1/2 + w_x + p4);
}

float GetS_Y(float lid_y, float y1, float w_y, float p1)
{
	return (y1/2 + w_y + p1) - lid_y;
}

float Get_Lidar_Right_X(float p4, float x1, float x2, float dx, float p2, float lid_x)
{
	return p4 + x1 + x2 + dx + p2 - lid_x;
}

float Get_Lidar_Back_Y(float p1, float y1, float dy, float p3, float lid_y)
{
	return p1 + y1 + dy + p3 - lid_y;
}

float Get_Safety_Zone(float w_x, float x, float p4, float c_x, float y, float dy, float p1, float p3, float c_y)
{
	float d1 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p1 + c_y, 2));
	float d2 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p3 + c_y, 2));
	if (d1 > d2)
		return d1;
	else 
		return d2;
}

float Get_Safety_Tank_Zone(float w_x, float x2, float p2, float c_x, float y2, float w_y, float p1, float p3, float p1, float p4, float c_y)
{
	float d1 = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p1 + c_y, 2));
	float d2 = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p3 + c_y, 2));
	float d3 = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p1 + c_y, 2));
	float d4 = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p3 + c_y, 2));
	if (d1 > d2 && d1 > d3 && d1 > d4)
		return d1;
	else if (d2 > d3 && d2 > d4)
		return d2;
	else if (d3 > d4)
		return d3;
	else
		return d4;
}
