#include "power.h"
#include "computationMovement.h"
#include <mavros_msgs/OverrideRCIn.h>

/*
	алгоритм, реализующий управяющее воздействие
	основанный на Пропорциональном или Пропорционально-дифференциальном регуляторе, в зависимости от выбора пользователя
*/
bool obstructionClose = false;
int zero;

double controlAction(int algorithm, int currentAction, double local_angle_to_target)
{
	//ROS_INFO("(%d)", zero);
	double u; 
	double i;
	//если робот не стоит и не едет прямо, то будем считать управлюящее воздействие для движения
	if (currentAction != ACTION_STAY && currentAction != ACTION_MOVE_FORWARD)
	{
		double err = local_angle_to_target; /*local_angle_to_target*/ //ошибка между заданной целью робота и его текущем положением
			switch (algorithm)
			{
				case P_regulator: //Выбор П-регулятора по флагу
					u = kp * err; //Вычисление управляющей Пропорционального регулятора
					break;
				case PD_regulator: //Выбор ПД-регулятора по флагу
					u = kp * err + kd *(err - errold); //Вычисление управляющей Пропорционально-Дифференциального регулятора
					errold = err; //Запоминаем предыдущую ошибку
					break;
				case PID_regulator: //Выбор ПИД-регулятора по флагу
					i = i_old + ki * err; //Вычисление интегральной составляющей
					i_old = i; //Запоминаем предыдущую интегральную составляющую
					u = kp * err + kd * (err - errold) + i; //Вычисление управляющей Пропорцианольно-Интегрально-Дифференциального регулятора
					errold = err; //Запоминаем предыдущую ошибку
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
					rc.channels[2] = MOVE;
					rc.channels[0] = STOP;
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
			if (currentAction != ACTION_STAY)
			{
				if (control > 200)
					control = 200;
				else if (control < -200)
					control = -200;
				
				if (currentAction == ACTION_TURN_LEFT)
				{
					rc.channels[2] = STOP + 200 + control;
					rc.channels[0] = STOP - 30 + control * 2;
				}
				else
				{
					rc.channels[2] = STOP + 200;
					rc.channels[0] = STOP + 70 + control * 2;
				}
				ROS_INFO("_______P________ %d %d %0.3f", rc.channels[2], rc.channels[0], (float)control);
				/*
				округлим управляющую составляющую и прибавим её к rc.channels[0], который отвечает за поворот
				управляющая control будет иметь знак "+" или "-"(зависит от того, как повернулся робот и какой угол образовался между ним и целью)
				и в зависимости от этого будет происходить поворот налево или направо
				если у control знак "+", то направо
				если у control знак "-", то налево

			
				rc.channels[0] = STOP + round(control);
				rc.channels[2] = STOP;
				*/
			}
			else 
			{
					rc.channels[0] = STOP;
					rc.channels[2] = STOP;
					ROS_INFO("_______P________ STOP");
			}
		}
	}
}


