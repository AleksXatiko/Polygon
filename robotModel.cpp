#include <ros/ros.h>
#include <cmath>
#include "poly_ros/robotModel_parametrs.h"

using namespace std;
using namespace poly_ros;

double p1, p2, p3, p4;
double left_caterpillar_width, right_caterpillar_width, left_caterpillar_length, right_caterpillar_lenght;
double distance_between_caterpillar, caterpillar_offset;
double lidar_x, lidar_y, error_x, error_y;
double w_x, w_y, s_x, s_y, r, l, q, lidar_offset_x, lidar_offset_y;

double GetW_X(double x1, double x2, double dx) //находим расстояние от центра робота до центров гусениц по оси X
{
	return (x1/2 + x2/2 + dx)/2;
}

double GetW_Y(double dy)	//находим расстояние от центра робота до центров гусениц по оси Y
{
	return abs(dy)/2;
}

double GetS_X(double lid_x, double x1, double w_x, double p4)	//смещение сенсора по оси X
{
	return lid_x - (x1/2 + w_x + p4);
}

double GetS_Y(double lid_y, double y1, double w_y, double p1)	//смещение сенсора по оси Y
{
	return (y1/2 + w_y + p1) - lid_y;
}

double Get_Lidar_Right_X(double p4, double x1, double x2, double dx, double p2, double lid_x)	//расстояние от центра лидара до правого края
{
	return p4 + x1 + x2 + dx + p2 - lid_x;
}

double Get_Lidar_Back_Y(double p1, double y1, double dy, double p3, double lid_y)	//расстояние от центра лидара до заднего края
{
	return p1 + y1 + dy + p3 - lid_y;
}

double Get_Safety_Zone(double w_x, double x, double p4, double c_x, double y, double dy, double p1, double p3, double c_y)	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой или левой стороны. это требуется для разворота с фиксированной гусеницей 
{
	double d1 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p1 + c_y, 2));
	double d2 = sqrt(pow(2*w_x + x/2 + p4 + c_x, 2) + pow(y/2 + abs(dy) + p3 + c_y, 2));
	if (d1 > d2)
		return d1;
	else 
		return d2;
}

double Get_Safety_Tank_Zone(double w_x, double x1, double x2, double p2, double c_x, double y1, double y2, double w_y, double p1, double p3, double p4, double c_y)	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой и левой стороны. Это требуется для танкового разворота 
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

double Safety_Angle(double lid_y, double S, double lid_x, double c_x, double c_y) // общая формула для нахождения углов зоны безопасного движения вперед и назад
{
	return atan((lid_x + c_x)/(lid_y + S + c_y)) * 180/M_PI;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotModel");
    ros::NodeHandle nhPr("~");
	ros::Publisher robot_pub;

	nhPr.param("p1", p1, 7.0);
	nhPr.param("p2", p2, 7.0);
	nhPr.param("p3", p3, 7.0);
	nhPr.param("p4", p4, 7.0);
	
	ros::NodeHandle nh;
	robot_pub = nh.advertise<robotModel_parametrs>("robotModel_parametrs", 1000);
	
	poly_ros::robotModel_parametrs parametrs;
	vector<double> p(2);
	p[0] = p1;
	p[1] = p2;
	parametrs.parametr = p;
	robot_pub.publish(parametrs);
	
	ros::spin(); 
	
	return 0;
}