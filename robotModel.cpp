#include <ros/ros.h>
#include <cmath>
#include "poly_ros/robotModel_parametrs.h"

#define FIXED_LEFT_CATERPILLAR 1;
#define FIXED_RIGHT_CATERPILLAR 2;
#define TANK_TURN 3;

using namespace std;
using namespace poly_ros;

double p1, p2, p3, p4, turn_mode;
double left_caterpillar_width, right_caterpillar_width, left_caterpillar_length, right_caterpillar_lenght;
double distance_between_caterpillar, caterpillar_offset;
double lidar_x, lidar_y, error_x, error_y;
ros::Publisher robot_pub;

double CatCenter_LidCenter_Angle(double w_x, double w_y, double lidar_offset_x, double lidar_offset_y, double dist)
{
	return acos(-(w_x + lidar_offset_x) / dist) * 180/M_PI;
}

double CatCenter_to_LidCenter(double w_x, double w_y, double lidar_offset_x, double lidar_offset_y)
{
	return sqrt(pow(w_x + lidar_offset_x, 2) + pow(w_y - lidar_offset_y, 2));
}

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

double Get_Safety_Zone(double w_x, double x, double p, double c_x, double y, double dy, double p1, double p3, double c_y)	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой или левой стороны. это требуется для разворота с фиксированной гусеницей 
{
	double d[2];
	d[0] = sqrt(pow(2*w_x + x/2 + p + c_x, 2) + pow(y/2 + abs(dy) + p1 + c_y, 2));
	d[1] = sqrt(pow(2*w_x + x/2 + p + c_x, 2) + pow(y/2 + abs(dy) + p3 + c_y, 2));
	return Get_Max(d, 2);
}

double Get_Safety_Tank_Zone(double w_x, double x1, double x2, double p2, double c_x, double y1, double y2, double w_y, double p1, double p3, double p4, double c_y)	//находим наибольшую диагональ от центра лидара до угла охватываюбщего прямоугольника для правой и левой стороны. Это требуется для танкового разворота 
{
	double d[4];
	d[0] = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p1 + c_y, 2));
	d[1] = sqrt(pow(w_x + x2/2 + p2 + c_x, 2) + pow(y2/2 + w_y + p3 + c_y, 2));
	d[2] = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p1 + c_y, 2));
	d[3] = sqrt(pow(w_x + x1/2 + p4 + c_x, 2) + pow(y1/2 + w_y + p3 + c_y, 2));
	return Get_Max(d, 4);
}

double Get_Max(double[] data, int length)
{
	double max = -1;
	for (int i = 0; i < length, i++)
	{
		if (data[i] > max)
			max = data[i];
	}
	return max;
}

double Get_Safety_Angle(double lid_y, double S, double lid_x, double c_x, double c_y) // общая формула для нахождения углов зоны безопасного движения вперед и назад
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
	
	double w_x = GetW_X(left_caterpillar_width, right_caterpillar_width, distance_between_caterpillar);
	double w_y = GetW_Y(caterpillar_offset);
	double s_x = GetS_X(lidar_x, left_caterpillar_width, w_x, p4);
	double s_y = GetS_Y(lidar_y, left_caterpillar_length, w_y, p1);
	
	double radius;
	double distance;
	double angle;
	switch ((int)turn_mode)
	{
		case FIXED_LEFT_CATERPILLAR:
			radius = Get_Safety_Zone(w_x, right_caterpillar_width, p2, error_x, left_caterpillar_length, caterpillar_offset, p1, p3, error_y);
			distance = CatCenter_to_LidCenter(w_x, w_y, lidar_offset_x, lidar_offset_y);
			angle = CatCenter_LidCenter_Angle(w_x, w_y, lidar_offset_x, lidar_offset_y, distance);
			break;
		case FIXED_RIGHT_CATERPILLAR:
			radius = Get_Safety_Zone(w_x, left_caterpillar_width, p4, error_x, right_caterpillar_lenght, caterpillar_offset, p1, p3, error_y);
			distance = CatCenter_to_LidCenter(-w_x, -w_y, -lidar_offset_x, lidar_offset_y);
			angle = CatCenter_LidCenter_Angle(-w_x, -w_y, -lidar_offset_x, lidar_offset_y, distance);
			break;
		case TANK_TURN:
			radius = Get_Safety_Tank_Zone(w_x, left_caterpillar_width, right_caterpillar_width, p2, error_x, left_caterpillar_length, right_caterpillar_lenght, w_y, p1, p3, p4, error_y);
			distance = CatCenter_to_LidCenter(0, 0, lidar_offset_x, lidar_offset_y);
			angle = CatCenter_LidCenter_Angle(0, 0, lidar_offset_x, lidar_offset_y, distance);
			break;
	}
	
	ros::NodeHandle nh;
	robot_pub = nh.advertise<robotModel_parametrs>("robotModel_parametrs", 1000);
	
	poly_ros::robotModel_parametrs parametrs;
	vector<double> p(9);
	p[0] = range1;
	p[1] = range2;
	p[2] = range3;
	p[3] = range4;
	p[4] = min_distance;
	p[5] = turn_mode;
	p[6] = radius;
	p[7] = distance;
	p[8] = angle;
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