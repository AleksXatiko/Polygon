#pragma once

#include <mavros_msgs/OverrideRCIn.h>
#include "ros/ros.h"

//������ ������ ��� ��-����������
double errold = 0.0;

//������������ ��� ������ �-, ��-���������� ��� ���������
enum controlMethod { relay=1, P_regulator, PD_regulator };
#define relay 1
#define P_regulator 3
#define PD_regulator 2
extern bool obstructionClose;

//��������� ��� �����
const int STOP = 1500;
const int MOVE = 1590;
const int LEFT = 1200;
const int RIGHT = 1800;

//������������ ����������� 
const double k = 17.3;//����������� �-����������
const double kd = 30.5;//����������� �-����������

/*
	��������, ����������� ���������� �����������
	���������� �� ���������������� ��� ���������������-���������������� ����������, � ����������� �� ������ ������������
*/
double controlAction(int algorithm, int currentAction);

/*
	�������� ������� ������ ��� ������ � ���������� ���������� ����������
	�.�. �� ����, ����� �����, ���� ����� ��� ������������ ����� ���������� ���������� �� �����, ��������� ���� ���������� �������� ������ � ���� 
*/
void updateRCfromAction(int algorithm, double control, int currentAction, mavros_msgs::OverrideRCIn &rc);