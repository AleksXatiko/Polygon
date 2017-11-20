#pragma once

#include "poly_ros/obstacles.h"
#include <cmath>

#define INFRONT 1
#define BEHIND 2
#define ASIDE 3

//�������� ������� ����������� � ������ ������
bool obstacleCheck(const poly_ros::obstacles::ConstPtr& mas, int index, int range1, int range2, int side);

double GetW_X(double x1, double x2, double dx); //������� ���������� �� ������ ������ �� ������� ������� �� ��� X

double GetW_Y(double dy);	//������� ���������� �� ������ ������ �� ������� ������� �� ��� Y

double GetS_X(double lid_x, double x1, double w_x, double p4);	//�������� ������� �� ��� X

double GetS_Y(double lid_y, double y1, double w_y, double p1);	//�������� ������� �� ��� Y

double Get_Lidar_Right_X(double p4, double x1, double x2, double dx, double p2, double lid_x);	//���������� �� ������ ������ �� ������� ����

double Get_Lidar_Back_Y(double p1, double y1, double dy, double p3, double lid_y);	//���������� �� ������ ������ �� ������� ����

double Get_Safety_Zone(double w_x, double x, double p4, double c_x, double y, double dy, double p1, double p3, double c_y);	//������� ���������� ��������� �� ������ ������ �� ���� �������������� �������������� ��� ������ ��� ����� �������. ��� ��������� ��� ��������� � ������������� ��������� 

double Get_Safety_Tank_Zone(double w_x, double x2, double p2, double c_x, double y2, double w_y, double p1, double p3, double p4, double c_y);	//������� ���������� ��������� �� ������ ������ �� ���� �������������� �������������� ��� ������ � ����� �������. ��� ��������� ��� ��������� ��������� 

double Safety_Angle(double lid_y, double S, double lid_x, double c_x, double c_y); // ����� ������� ��� ���������� ����� ���� ����������� �������� ������ � �����