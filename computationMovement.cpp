#include "computationMovement.h"
#include <tuple>
#include "ros/ros.h"
int zero;

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
	//target_x = 1;
	//target_y = 1;
    double dx=target_x-current_x; //разница между заданной координатой цели и текущим положением (по X)
    double dy=target_y-current_y; //разница между заданной координатой цели и текущим положением (по Y)
    double distance_to_target = hypot(dx,dy); //считаем расстояние между текущим положением робота и заданной координатой (гипотенуза треугольника)
    double global_angle_to_target=atan2(dy, dx); //глобальный угол к цели
    *local_angle_to_target=normalizeAngle(global_angle_to_target - current_angle); //текущий угол относително заданной цели
	//ROS_INFO("angle=%0.2f", (float)(*local_angle_to_target));
	ROS_INFO("_____TARGET______ %0.3f %0.3f", target_x, target_y);
	ROS_INFO("_____CURRENT_____ %0.3f %0.3f", current_x, current_y);
	ROS_INFO("_____ANGLE_______ %0.3f", *local_angle_to_target);

	int action;
	
	//если расстояние до цели меньше 0.1(допустимого отклонения), то будем считать, что мы добрались до цели и нужно остановиться 
    if( distance_to_target < MOVE_ACCURACY )
	{ 
		action = ACTION_STAY; //приехали 
	}
	
	/*
		если мы не у цели, то проверим, как расположен робот(куда напревлен) относительно заданной точки
		если угол между положением робота и целью меньше 0.2(допустимого отклонения), то будем считать, что робот направлен(смотрит) на цель 
	*/
    else if(fabs(*local_angle_to_target)<TURN_ACCURACY)
	{ 
		action = ACTION_MOVE_FORWARD; //едем прямо
	}
	
	//если текущий угол робота относительно цели больше 0, то нужно повернуть налево
    else if(*local_angle_to_target > zero)
	{ 
		action = ACTION_TURN_LEFT; //поворачиваем налево 
	}
	else
     {
        action = ACTION_TURN_RIGHT; //поворачиваем направо
     }
	 
	return action; 
}