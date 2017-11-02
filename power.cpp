#include "power.h"
#include "computationMovement.h"
#include <mavros_msgs/OverrideRCIn.h>

/*
	��������, ����������� ���������� �����������
	���������� �� ���������������� ��� ���������������-���������������� ����������, � ����������� �� ������ ������������
*/
bool obstructionClose = false;
int zero;

double controlAction(int algorithm, int currentAction, double local_angle_to_target)
{
	//ROS_INFO("(%d)", zero);
	double u; 
	double i;
	//���� ����� �� ����� � �� ���� �����, �� ����� ������� ����������� ����������� ��� ��������
	if (currentAction != ACTION_STAY && currentAction != ACTION_MOVE_FORWARD)
	{
		double err = local_angle_to_target; /*local_angle_to_target*/ //������ ����� �������� ����� ������ � ��� ������� ����������
			switch (algorithm)
			{
				case P_regulator: //����� �-���������� �� �����
					u = kp * err; //���������� ����������� ����������������� ����������
					break;
				case PD_regulator: //����� ��-���������� �� �����
					u = kp * err + kd *(err - errold); //���������� ����������� ���������������-����������������� ����������
					errold = err; //���������� ���������� ������
					break;
				case PID_regulator: //����� ���-���������� �� �����
					i = i_old + ki * err; //���������� ������������ ������������
					i_old = i; //���������� ���������� ������������ ������������
					u = kp * err + kd * (err - errold) + i; //���������� ����������� ���������������-�����������-����������������� ����������
					errold = err; //���������� ���������� ������
					break;
				default:
					u = 0.0;
			}

		//���� ��� ����� ���������� �������, � ���� ����������� �� ���������
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
	�������� ������� ������ ��� ������ � ���������� ���������� ����������
	�.�. �� ����, ����� �����, ���� ����� ��� ������������ ����� ���������� ���������� �� �����, ��������� ���� ���������� �������� ������ � ���� 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc)
/*
rc.channels[0]-�������� �� �������,
rc.channels[2]-�������� �� �������� ������� �����
�������� �� 1000 �� 2000
1500-����� �� �����
������ 1500- �������� �������� �� ������� �������
������ 1500- �������� �������� ������ ������� �������
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
		//���� ������ �������� ���������
		if (algorithm == relay )
		{
			//currentAction = 1;
			ROS_INFO("destvie=%d", currentAction);
			//currentAction=ACTION_MOVE_FORWARD;
			switch (currentAction)
			{
				case ACTION_MOVE_FORWARD: //���� �����
					rc.channels[2] = MOVE;
					rc.channels[0] = STOP;
					ROS_INFO("MOVE");
					break;
				case ACTION_TURN_LEFT: //������������ ������
					rc.channels[0] = LEFT;
					rc.channels[2] = STOP;
					break;
				case ACTION_TURN_RIGHT: //������������ �������
					rc.channels[0] = RIGHT;
					rc.channels[2] = STOP;
					break;
				default: //�����
					rc.channels[0] = STOP;
					rc.channels[2] = STOP;
			}
		}
		else //��������� �� ���������� �- � ��-�����������
		{
			//���� ����� �� ���� �����, �� ������� ���������� ����������� � �����, ����� �� ����������� � ����
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
				�������� ����������� ������������ � �������� � � rc.channels[0], ������� �������� �� �������
				����������� control ����� ����� ���� "+" ��� "-"(������� �� ����, ��� ���������� ����� � ����� ���� ����������� ����� ��� � �����)
				� � ����������� �� ����� ����� ����������� ������� ������ ��� �������
				���� � control ���� "+", �� �������
				���� � control ���� "-", �� ������

			
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


