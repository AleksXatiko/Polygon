#include "power.h"
#include "computationMovement.h"
#include <mavros_msgs/OverrideRCIn.h>

/*
	алгоритм, реализующий управяющее воздействие
	основанный на Пропорциональном или Пропорционально-дифференциальном регуляторе, в зависимости от выбора пользователя
*/
bool obstructionClose = false;
int zero;

double controlAction(int algorithm, int currentAction)
{
	//ROS_INFO("(%d)", zero);
	double u; 
	//если робот не стоит и не едет прямо, то будем считать управлюящее воздействие для движения
	if (currentAction != ACTION_STAY && currentAction != ACTION_MOVE_FORWARD)
	{
		double err = 0; /*local_angle_to_target*/ //ошибка между заданной целью робота и его текущем положением
			switch (algorithm)
			{
				case PD_regulator: //выбор  ПД-регулятора по флагу
					u = k * err + kd *(err - errold);//вычисление управляющей Пропорционально-дифференциального регулятора
					errold = err;//запоминаем предыдущую ошибку
					break;
				case P_regulator://выбор  П-регулятора по флагу
					u = k*err;
					break;
				default:
					u = 0.0;
			}

		//Если нам нужен конкретный поворот, а знак управляющей не совпадает
		if((currentAction == ACTION_TURN_LEFT && u > zero) || (currentAction == ACTION_TURN_RIGHT && u < zero))
		{
			u = -u;
		}
	}
	else
	{
		u = 0.0;
	};
	return u;
}

/*
	алгоритм перевод команд для робота в конкретное управление гусеницами
	т.е. от того, стоит робот, едет прямо или поворачивает будет подаваться управление на шасси, благодаря чему происходит движение робота к цели 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc)
/*
rc.channels[0]-отвечает за поворот,
rc.channels[2]-отвечает за движение гусениц вперёд
значения от 1000 до 2000
1500-стоим на месте
больше 1500- гусеницы движутся по часовой стрелке
меньше 1500- гусеницы движутся против часовой стрелки
*/
{
    if(obstructionClose)
    {
	rc.channels[0] = STOP;
	rc.channels[2] = STOP;
	ROS_INFO("STAY 1");
    }
    else
    {
	//если выбран релейный регулятор
	if (algorithm == relay )
	{
		//currentAction = 1;
		ROS_INFO("destvie=%d", currentAction);
		//currentAction=ACTION_MOVE_FORWARD;
		switch (currentAction)
		{
		case ACTION_MOVE_FORWARD: //едем прямо
			rc.channels[0] = STOP;
			rc.channels[2] = MOVE;
			ROS_INFO("MOVE");
			break;
		case ACTION_TURN_LEFT: //поворачиваем налево
			rc.channels[0] = LEFT;
			rc.channels[2] = STOP;
			break;
		case ACTION_TURN_RIGHT: //поворачиваем направо
			rc.channels[0] = RIGHT;
			rc.channels[2] = STOP;
			break;
		default: //стоим
			rc.channels[0] = STOP;
			rc.channels[2] = STOP;

		}
	}
	else //действуем по алгоритмам П- и ПД-регуляторов
	{
		//если робот не едет прямо, то добавим управляюще воздейсвтие к шасси, чтобы он развернулся к цели
		if (currentAction != ACTION_MOVE_FORWARD)
		{
			ROS_INFO("regular 2 or 3");
			/*
			округлим управляющую составляющую и прибавим её к rc.channels[0], который отвечает за поворот
			управляющая control будет иметь знак "+" или "-"(зависит от того, как повернулся робот и какой угол образовался между ним и целью)
			и в зависимости от этого будет происходить поворот налево или направо
			если у control знак "+", то направо
			если у control знак "-", то налево

			*/
			rc.channels[0] = STOP + round(control);
			rc.channels[2] = STOP;
		}
		else//робот поедет прямо без управляющей составляющей
		{
			rc.channels[0] = STOP;
			rc.channels[2] = MOVE;
		};
	    };
	}
}


