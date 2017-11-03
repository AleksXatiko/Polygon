#include "computationMovement.h"
#include <tuple>
#include "ros/ros.h"
int zero;

//���������� ���� � ��������� -Pi<angle<=+Pi, ��� ���� ����� �� ������������ ����� ������ ���� (��� ��������� �� ���������� ����������)
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
	��������, ������� ������(����������) ���������� �������� ��� ������, �� ������ ��� �������� ��������� � �������� ����;
	�.�. ������ ����� ������ ������, ����� �� ���� ��������� �� ����
*/
int getAction(double current_x, double current_y, double current_angle, double target_x, double target_y, double *local_angle_to_target) 
{
	//target_x = 1;
	//target_y = 1;
    double dx=target_x-current_x; //������� ����� �������� ����������� ���� � ������� ���������� (�� X)
    double dy=target_y-current_y; //������� ����� �������� ����������� ���� � ������� ���������� (�� Y)
    double distance_to_target = hypot(dx,dy); //������� ���������� ����� ������� ���������� ������ � �������� ����������� (���������� ������������)
    double global_angle_to_target=atan2(dy, dx); //���������� ���� � ����
    *local_angle_to_target=normalizeAngle(global_angle_to_target - current_angle); //������� ���� ����������� �������� ����
	//ROS_INFO("angle=%0.2f", (float)(*local_angle_to_target));
	ROS_INFO("_____TARGET______ %0.3f %0.3f", target_x, target_y);
	ROS_INFO("_____CURRENT_____ %0.3f %0.3f", current_x, current_y);
	ROS_INFO("_____ANGLE_______ %0.3f", *local_angle_to_target);

	int action;
	
	//���� ���������� �� ���� ������ 0.1(����������� ����������), �� ����� �������, ��� �� ��������� �� ���� � ����� ������������ 
    if( distance_to_target < MOVE_ACCURACY )
	{ 
		action = ACTION_STAY; //�������� 
	}
	
	/*
		���� �� �� � ����, �� ��������, ��� ���������� �����(���� ���������) ������������ �������� �����
		���� ���� ����� ���������� ������ � ����� ������ 0.2(����������� ����������), �� ����� �������, ��� ����� ���������(�������) �� ���� 
	*/
    else if(fabs(*local_angle_to_target)<TURN_ACCURACY)
	{ 
		action = ACTION_MOVE_FORWARD; //���� �����
	}
	
	//���� ������� ���� ������ ������������ ���� ������ 0, �� ����� ��������� ������
    else if(*local_angle_to_target > zero)
	{ 
		action = ACTION_TURN_LEFT; //������������ ������ 
	}
	else
     {
        action = ACTION_TURN_RIGHT; //������������ �������
     }
	 
	return action; 
}