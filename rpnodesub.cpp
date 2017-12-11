#include <iostream>
#include <cmath>
#include "ros/ros.h" 
#include "poly_ros/target.h"
#include "poly_ros/obstacles.h"

using namespace poly_ros;
using namespace std;

float x, y;
ros::Publisher chatter_pub;

void go_to_target(const poly_ros::obstacles::ConstPtr& mas)
{
	cout << mas->mass[mas->num - 1].begin_x << '\n';
	cout << mas->mass[mas->num - 1].begin_y << '\n';
	cout << mas->mass[0].end_x << '\n';
	cout << mas->mass[0].end_y << '\n';
	cout << '\n';
	x = (mas->mass[mas->num - 1].begin_x + mas->mass[0].end_x)/2;
	y = (mas->mass[mas->num - 1].begin_y + mas->mass[0].end_y)/2;
}
	
int main(int argc, char **argv)
{	
	ros::init (argc, argv, "rpnodesub");	//инициализация узла
	ros::NodeHandle target_sub;
	
	//Подпишемся на топик c данными лидара
	ros::Subscriber scanSub = target_sub.subscribe("obstacles" , 1000, go_to_target);
	
	///////////////////////////////////////////////////////////////
	ros::NodeHandle target_pub;									///
	chatter_pub = target_pub.advertise<target>("target", 1000);	
	///---------------------------------------------------------///
	poly_ros::target mas;										///
	ros::Rate rate(5.5);
	
	while(ros::ok())
	{
		mas.x = x; 		// публикуем количествой препятствий				///
		mas.y = y; 		// публикуем количествой препятствий	
		chatter_pub.publish(mas); // публикуем данные в топик 		///
		ros::spinOnce();
		rate.sleep();
	}
	return 0;			
}