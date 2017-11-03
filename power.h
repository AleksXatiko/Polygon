#pragma once

#include <mavros_msgs/OverrideRCIn.h>
#include "ros/ros.h"

//������ ������ ��� ��-����������
double errold = 0.0;
double i_old = 0.0;

//������������ ��� ������ �-, ��-, ���-���������� ��� ���������
enum controlMethod { relay=1, P_regulator, PD_regulator };
#define relay 1
#define P_regulator 2
#define PD_regulator 3
#define PID_regulator 4
extern bool obstructionClose;

//��������� ��� �����
const int STOP = 1500;
const int MOVE = 1600;
const int LEFT = 1200;
const int RIGHT = 1800;

//������������ ����������� 
const double kp = 221;//����������� �-����������
const double kd = 180;//����������� �-����������
const double ki = 0.03;//����������� �-����������
const int kl = -30;//����������� ��� ����� ��������
const int kr = 70;//����������� ��� ������ ��������

/*
	��������, ����������� ���������� �����������
	���������� �� ���������������� ��� ���������������-���������������� ����������, � ����������� �� ������ ������������
*/
double controlAction(int algorithm, int currentAction, double local_angle_to_target);

/*
	�������� ������� ������ ��� ������ � ���������� ���������� ����������
	�.�. �� ����, ����� �����, ���� ����� ��� ������������ ����� ���������� ���������� �� �����, ��������� ���� ���������� �������� ������ � ���� 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc);