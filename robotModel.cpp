#include <ros/ros.h>
#include <cmath>
#include "poly_ros/robotModel_parametrs.h"

using namespace std;
using namespace poly_ros;

double p1, p2, p3, p4, turn_mode;
double left_caterpillar_width, right_caterpillar_width, left_caterpillar_length, right_caterpillar_lenght;
double distance_between_caterpillar, caterpillar_offset;
double lidar_x, lidar_y, error_x, error_y;
double w_x, w_y, s_x, s_y, r, l, q, lidar_offset_x, lidar_offset_y;
ros::Publisher robot_pub;

double GetW_X(double x1, double x2, double dx) //������� ���������� �� ������ ������ �� ������� ������� �� ��� X
{
	return (x1/2 + x2/2 + dx)/2;
}

double GetW_Y(double dy)	//������� ���������� �� ������ ������ �� ������� ������� �� ��� Y
{
	return abs(dy)/2;
}

double GetS_X(double lid_x, double x1, double w_x, double p4)	//�������� ������� �� ��� X
{
	return lid_x - (x1/2 + w_x + p4);
}

double GetS_Y(double lid_y, double y1, double w_y, double p1)	//�������� ������� �� ��� Y
{
	return (y1/2 + w_y + p1) - lid_y;
}

double Get_Lidar_Right_X(double p4, double x1, double x2, double dx, double p2, double lid_x)	//���������� �� ������ ������ �� ������� ����
{
	return p4 + x1 + x2 + dx + p2 - lid_x;
}

double Get_Lidar_Back_Y(double p1, double y1, double dy, double p3, double lid_y)	//���������� �� ������ ������ �� ������� ����
{
	return p1 + y1 + dy + p3 - lid_y;
}

double Get_Safety_Zone(double w_x, double x, double p4, double c_x, double y, double dy, double p1, double p3, double c_y)	//������� ���������� ��������� �� ������ ������ �� ���� �������������� �������������� ��� ������ ��� ����� �������. ��� ��������� ��� ��������� � ������������� ��������� 
{
	double d1 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p1 + c_y, 2));
	double d2 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p3 + c_y, 2));
	if (d1 > d2)
		return d1;
	else 
		return d2;
}

double Get_Safety_Tank_Zone(double w_x, double x1, double x2, double p2, double c_x, double y1, double y2, double w_y, double p1, double p3, double p4, double c_y)	//������� ���������� ��������� �� ������ ������ �� ���� �������������� �������������� ��� ������ � ����� �������. ��� ��������� ��� ��������� ��������� 
{
	double d1 = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p1 + c_y, 2));
	double d2 = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p3 + c_y, 2));
	double d3 = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p1 + c_y, 2));
	double d4 = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p3 + c_y, 2));
	if (d1 > d2 && d1 > d3 && d1 > d4)
		return d1;
	else if (d2 > d3 && d2 > d4)
		return d2;
	else if (d3 > d4)
		return d3;
	else
		return d4;
}

double Get_Safety_Angle(double lid_y, double S, double lid_x, double c_x, double c_y) // ����� ������� ��� ���������� ����� ���� ����������� �������� ������ � �����
{
	return atan((lid_x + c_x)/(lid_y + S + c_y)) * 180/M_PI;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotModel");
    ros::NodeHandle nhPr("~");

	nhPr.param("p1", p1, 4.0);
	nhPr.param("p2", p2, 4.5);
	nhPr.param("p3", p3, 4.5);
	nhPr.param("p4", p4, 3.0);
	nhPr.param("left_caterpillar_width", left_caterpillar_width, 2.8);
	nhPr.param("right_caterpillar_width", right_caterpillar_width, 2.8);
	nhPr.param("left_caterpillar_length", left_caterpillar_length, 28);
	nhPr.param("right_caterpillar_lenght", right_caterpillar_lenght, 28.0);
	nhPr.param("distance_between_caterpillar", distance_between_caterpillar, 8.5);
	nhPr.param("caterpillar_offset", caterpillar_offset, 0.0);
	nhPr.param("lidar_x", lidar_x, 10.0);
	nhPr.param("lidar_y", lidar_y, 10.0);
	nhPr.param("min_distance", min_distance, 0.45);
	nhPr.param("turn_mode", turn_mode, 1.0); 
	nhPr.param("error_x", error_x, 1.0); 
	nhPr.param("error_y", error_y, 1.0); 
	
	double lidar_rx = Get_Lidar_Right_X(p4, left_caterpillar_width, right_caterpillar_width, distance_between_caterpillar, p2, lidar_x);
	double lidar_by = Get_Lidar_Back_Y(p1, left_caterpillar_length, caterpillar_offset, p3, lidar_y);
	double range1 = Get_Safety_Angle(lidar_y, min_distance, lidar_x, error_x, error_y);
	double range2 = Get_Safety_Angle(lidar_y, min_distance, lidar_rx, error_x, error_y);
	double range3 = Get_Safety_Angle(lidar_by, min_distance, lidar_x, error_x, error_y);
	double range4 = Get_Safety_Angle(lidar_by, min_distance, lidar_rx, error_x, error_y);
	
	ros::NodeHandle nh;
	robot_pub = nh.advertise<robotModel_parametrs>("robotModel_parametrs", 1000);
	
	poly_ros::robotModel_parametrs parametrs;
	vector<double> p(6);
	p[0] = range1;
	p[1] = range2;
	p[2] = range3;
	p[3] = range4;
	p[4] = min_distance;
	p[5] = turn_mode;
	parametrs.parametr = p;
	
	ros::Rate rate(2.0);
	while (ros::ok())
	{
		robot_pub.publish(parametrs);
		ros::spinOnce(); 
		rate.sleep();
	}
	
	return 0;
}