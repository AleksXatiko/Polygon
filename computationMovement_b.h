#pragma once

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//������ �������� ��� ��� ������� ������, ����� 4 ���������: �����, ���� �����, ������������ ������, ������������ �������
enum Action { STAY, MOVE_FORWARD, TURN_LEFT, TURN_RIGHT };
#define ACTION_STAY 0
#define ACTION_MOVE_FORWARD 1
#define ACTION_TURN_LEFT 2
#define ACTION_TURN_RIGHT 3

//���������� ���������� ��� ���������� �����, ������������ �� ����������� ��� ��������(����, ����������� ��������� ������, ������ � �.�.)
#define MOVE_ACCURACY 0.1  //���� ���������� �� �������� ���� ������ ����� ���������, �� �������, ��� ����� ������� � �������� �����
#define TURN_ACCURACY 0.2  //���� ���������� �������� ���� �� ��������� ������ ����� ���������, �� �������, ��� ����� ��������� � ������� �������� ����; ������� - ������

//������ ���������� ��� � � ����� ����������� �������
Eigen::Vector3d current_point, target_point;
Eigen::Quaterniond current_attitude, target_attitude;

//������� ���������� ����� ������� � ������ �������
double current_yaw=0.0;
double current_x=0.0, current_y=0.0, current_z=0.0;
double target_x=0.0, target_y=0.0, target_z=0.0;

//�����, � ������� ����� ��������� ������� � ������� ���������� � ���� ��������� � � ������� ����� (0 - ��� ����������, 1 - ����)
double target_set;
double position_set;

//���������� ���� � ��������� -Pi<angle<=+Pi, ��� ���� ����� �� ������������ ����� ������ ���� (��� ��������� �� ���������� ����������)
double normalizeAngle(double angle); 

/*
	��������, ������� ������(����������) ���������� �������� ��� ������, �� ������ ��� �������� ��������� � �������� ����;
	�.�. ������ ����� ������ ������, ����� �� ���� ��������� �� ����
*/
int getAction(double current_x, double current_y, double current_angle, double target_x, double target_y);
