#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "poly_ros/obstacle.h"
#include "poly_ros/obstacles.h"
#include "poly_ros/points.h"
#include "poly_ros/target.h"

#define NUM 360

using namespace std;
using namespace poly_ros;

double vector_error, lidar_error;

ros::Publisher chatter_pub;
ros::Publisher points_pub;

struct Vector
{
	float X, Y, Length;
	
	float GetLength()
	{
		return sqrt(pow(X, 2) + pow(Y, 2));
	}
	
	void Normalize()
	{
		X /= Length;
		Y /= Length;
	}
};

struct Point
{
	float X, Y;
};

struct Obstacle
{
	Point Begin;
	Point End;
	float min_distance;
};

Obstacle* GetObstacles(float data[NUM], int *number_of_obstacles)
{	
	Point points[NUM];
	int k = 0;
	for (int i = 0; i < NUM; i++)
	{
		if (data[i] >= (float)lidar_error)
		{
			points[k].X = data[i] * sin(i * M_PI / 180);
			points[k].Y = data[i] * cos(i * M_PI / 180);
			k++;
		}
	}
	poly_ros::points _points;
	vector<target> targets(k);
	for (int i = 0; i < k; i++)
	{
		targets[i].x = points[i].X;
		targets[i].y = points[i].Y;
	}
	_points.num = k;
	_points.points = targets;
	points_pub.publish(_points);
	
	int len = k;
	k = 0;
	Obstacle *obstacles = new Obstacle[len];
	
	Vector Vector1, Vector2;
	float x3, y3, cos_between_vectors, min_distance, temp;
	float x1 = points[0].X;
	float y1 = points[0].Y;
	float x2 = points[1].X;
	float y2 = points[1].Y;
	obstacles[0].Begin.X = x1;
	obstacles[0].Begin.Y = y1;
	
	int j = 2;
	while(j < len)
	{
		bool isGap = false;
		min_distance = 6.0f;
		while (!isGap && j < len)
		{
			x3 = points[j].X;
			y3 = points[j].Y;
			
			temp = sqrt(pow(x3, 2) + pow(y3, 2));
			if (temp < min_distance)
				min_distance = temp;
			
			Vector1.X = x2 - x1;
			Vector1.Y = y2 - y1;
			Vector2.X = x3 - x2;
			Vector2.Y = y3 - y2;

			Vector1.Length = Vector1.GetLength();
			Vector2.Length = Vector2.GetLength();
			cos_between_vectors = (Vector1.X * Vector2.X + Vector1.Y * Vector2.Y) / (Vector1.Length * Vector2.Length);
			if (fabs(1 - cos_between_vectors) >= (float)vector_error)
				isGap = true;

			x1 = x2;
			y1 = y2;
			x2 = x3;
			y2 = y3;
			j++;
		}
		
		obstacles[k].End.X = x1;
		obstacles[k].End.Y = y1;
		obstacles[k].min_distance = min_distance;
		obstacles[k + 1].Begin.X = x2;
		obstacles[k + 1].Begin.Y = y2;
		k++;
	}
	
	*number_of_obstacles = k;
	return obstacles;
}

// получаем данные с лидара 
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float Data[NUM];
    for (int i = 0; i < NUM; i++) //заполняем массив данными с лидара  
        Data[i] = (msg->ranges[i]);
	
	int num_obst = 0;
	Obstacle *info = GetObstacles(Data, &num_obst);
	
	poly_ros::obstacles obstacles;
	vector<obstacle> obst(num_obst);
	for (int i = 0; i < num_obst; i++)
	{
		obst[i].begin_x = info[i].Begin.X;
		obst[i].begin_y = info[i].Begin.Y;
		obst[i].end_x = info[i].End.X;
		obst[i].end_y = info[i].End.Y;
		obst[i].min_distance = info[i].min_distance;
		//printf("%0.3f;%0.3f\n", -obst[i].end_x, obst[i].end_y);
	}
	obstacles.num = num_obst;
	obstacles.mass = obst;
	chatter_pub.publish(obstacles);
	//printf("\n\n");
}

int main(int argc, char **argv)
{	
	ros::init (argc, argv, "findObstacles"); //инициализация узла
	ros::NodeHandle nh("~"); //объявление преерменной nh в пространстве имен NodeHandle
	ros::Subscriber scanSub; //обьявляем узел result подписчиком
	
	nh.param("vector_error", vector_error, 0.2);
	nh.param("lidar_error", lidar_error, 0.1);
	scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan); //подписка на данные с Lidar

	ros::NodeHandle n;
	chatter_pub = n.advertise<obstacles>("obstacles",1000); //публикация обработанных данных (карта препятствий)
	points_pub = n.advertise<points>("points",1000); 
	ros::spin(); 
	
	return 0;
}
