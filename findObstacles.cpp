#include <iostream>
#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "poly_ros/obstacle.h"
#include "poly_ros/obstacles.h"

using namespace std;
using namespace poly_ros;

double error;
double degree;
int zero, circle, endElement, gap, minLength;

ros::Publisher chatter_pub;


/*
���������� ������, ������� � A[0]. ���������� R0-����, ��� ������� ��������� ������, ����� A[R0] � A[R0+1].
� ��������� ������ ������� ������� ���������� ������� (NULL)
*/
int FirstBreak(float C[360], float *min1_distance, int *min_angle)
{
	int i = 0;
	*min1_distance = C[0];  //����������� ����������� �����������
	*min_angle = 0;
	int R0 = 0;   //Ro - ������ ������
				  //���� �� ������ ������ ��� �� �������� ��� ��������
	while (R0 == zero && i < circle)  //����� ������ ���� �� ������ ������ (Ro = 0) ��� ���� �� ������ ���� ����
	{
		if (abs(C[i] - C[i + 1]) <= error) //���� ������� ������� � ��������� �������� ���������
		{
			    if (C[i] < *min1_distance) //���� ����������� ����������� 
			    {
					*min1_distance = C[i];
					*min_angle = i;
			    }
		}
		else
		{
			R0 = i + 1;
		}
	    i = i + 1;
	}

	return R0; //���������� ������ ������

}


// �������� ������
int StudyData(float B[360])
{
	int num = 360; //������ �������
	poly_ros::obstacle ms;
	poly_ros::obstacles mas;
	int *nomer= new int[num];// ��������� ������ ��� �������
	int *angle1= new int[num];// ��������� ������ ��� �������
	float *min = new float[num]; // ��������� ������ ��� �������
	int *angle2= new int[num];// ��������� ������ ��� �������
	int *angle_min = new int[num];// ��������� ������ ��� �������
	int First_R0;  //Ro - ������ ������
	int R01;//Ro - ������ ������
	float min1_distance = 0.0;//����������� ����������� �����������
	int min1_angle = 0; //���� �� ������������ ����������
	First_R0 = FirstBreak(B, &min1_distance, &min1_angle);
	R01 = First_R0 - 1;
	float min11_distance = min1_distance;//����������� ����������� �����������
	int min11_angle = min1_angle; //���� �� ������������ ����������
	float min_distance1;//����������� ����������� �����������
	int angle3; //���� �� ������������ ����������
	int Ri = First_R0 + 1; //��� ������ ����������� 
	First_R0 = First_R0 + 1;// ��� ����� ����������
	int NUL = 0; // ������� ���������� �������
	int quantityr_obstacles = 0;// ���������� ����������� 
	
	if (First_R0 != zero)
	{	
		min_distance1 = B[First_R0 + 1];  //����������� ����������� �����������
		angle3 = First_R0 + 1; //���� �� ����������� �����������
		
		while (First_R0 < circle)       //���� �� ������ ����
		{
			if (abs(B[First_R0] - B[First_R0 + 1]) <= error)//������� ������� � ��������� �������� �����, �� ����� �������, ��� ��� ���� �����������
			{
				NUL = 0;
			}
			else
			{
				NUL = 1;
			}
			if (B[First_R0] < min_distance1 && B[First_R0] > 0.2) //���� ����������� ����������� 
			{
				min_distance1 = B[First_R0];
				angle3 = First_R0;
			}

			if (NUL == gap) // ���� ���� ������
			{
				if  (First_R0 != endElement) // ������� ��� ���� �� 360
				{
				    if(abs(Ri-First_R0) > minLength) //���� ������ ����������� ������ 2
					{
						angle1[quantityr_obstacles] = Ri; // ������� ���� ������ ����������� 
						min[quantityr_obstacles] = min_distance1;// ������� ����������� ��������� 
						angle_min[quantityr_obstacles] = angle3; //������� ���� �� ������������ ����������
						angle2[quantityr_obstacles] = First_R0;// ������� ���� ����� �����������
						nomer[quantityr_obstacles] = quantityr_obstacles + 1; // ������� ����� �����������  
						quantityr_obstacles = quantityr_obstacles + 1;// �������� ���������� ����������� 
					}
					NUL = 0;
					Ri = First_R0 + 1; 
					min_distance1 = B[First_R0 + 1];	
					angle3 = First_R0 + 1;
				}
			}
			First_R0 = First_R0 + 1 ;
		}
		
		if (B[First_R0] < min_distance1 && B[First_R0] > 0.2) //���� ����������� ����������� 
			{
				min_distance1 = B[First_R0];
				angle3 = First_R0;
			};


		if (((abs(B[359] - B[0])) > error) && (abs(B[359] - B[0]) != B[359])) //���� ������  ����� � [360] � A[0?]
		{
			angle1[quantityr_obstacles] = Ri; // ������� ���� ������ �����������
			min[quantityr_obstacles] = min_distance1;// ������� ����������� ��������� 
			angle_min[quantityr_obstacles] = angle3; //������� ���� �� ������������ ����������
			angle2[quantityr_obstacles] = First_R0;// ������� ���� ����� �����������
			nomer[quantityr_obstacles] = quantityr_obstacles + 1;// ������� ����� ����������� 
			quantityr_obstacles = quantityr_obstacles + 1;// �������� ���������� ����������� 
			Ri = 0;
			
		};

		if(Ri==zero)
			{
				min[quantityr_obstacles] = min_distance1;// ������� ����������� ��������� 
				angle_min[quantityr_obstacles] = angle3; //������� ���� �� ������������ ����������
				angle2[quantityr_obstacles] = First_R0;// ������� ���� ����� �����������
			}
		else
			{
				if (min11_distance > min_distance1) //���� ������������ ����������� 
				{
					min1_distance = min_distance1;
					min1_angle = angle3;
				}
				min[quantityr_obstacles] = min11_distance; // ������� ����������� ��������� 
				angle_min[quantityr_obstacles] = min11_angle; //������� ���� �� ������������ ����������
				if (abs(B[359] - B[0]) == B[359])
				    {
						angle2[quantityr_obstacles] = 359; // ������� ���� ����� �����������
				    }
				    else
				    {
						angle2[quantityr_obstacles] = R01; // ������� ���� ����� �����������
				    }
			}
			angle1[quantityr_obstacles] = Ri; // ������� ���� ������ �����������
			nomer[quantityr_obstacles] = quantityr_obstacles + 1; // ������� ����� �����������
			quantityr_obstacles = quantityr_obstacles + 1;// �������� ���������� ����������� 
	}
	else
	{
		angle1[quantityr_obstacles] = Ri; // ������� ���� ������ �����������
		min[quantityr_obstacles] = min1_distance;// ������� ����������� ���������
		angle_min[quantityr_obstacles] = min1_angle; //������� ���� �� ������������ ����������
		angle2[quantityr_obstacles] = First_R0;// ������� ���� ����� �����������
		nomer[quantityr_obstacles] = quantityr_obstacles + 1;// ������� ����� ����������� 
		quantityr_obstacles = quantityr_obstacles + 1;// �������� ���������� ����������� 
	}

	int number_obstacle = 0; // ����� ����������� 
	int Flag = 0; //������ ������� ������ �� ���������� ���������� ����������� 

	/* � ����� ���������� �������� ���� ����� ����������� ������ ��� degree � 
	��������� �� ��������� ���� ����������� ����������� �������������� ����� 
	��������� �� ���������� ���� ������� ����������� �� ��� ���� �����������  */
	while(number_obstacle < quantityr_obstacles)
	{
		if(abs(angle1[number_obstacle+1] - angle2[number_obstacle+1]) < degree) // ���� ����� ����������� ������ ��� degree �� ��� ����������� 
		{
			// ���� ��������� �� ��������� ���� ����������� ����������� �������������� ����� ��������� �� ���������� ���� ������� ����������� �� ��� ���� ����������� 
			if((abs(B[angle2[number_obstacle - Flag]] - B[angle1[number_obstacle + 1]]) < error))   
			{
				angle2[number_obstacle - Flag] = angle1[number_obstacle + 1]; //����������� ����� �����������
				if (min[number_obstacle - Flag] > min[number_obstacle + 1]) // ������� ����������� ���������
				{
					min[number_obstacle - Flag] = min[number_obstacle + 1];
					angle_min[number_obstacle - Flag] = angle_min[number_obstacle + 1]; 
				}
				//������� �������� �������
				angle1[number_obstacle + 1] = angle1[number_obstacle + 2];
				angle2[number_obstacle + 1] = angle2[number_obstacle + 2];
				min[number_obstacle + 1] = min[number_obstacle + 2];
				angle_min[number_obstacle + 1] = angle_min[number_obstacle + 2];
				angle1[number_obstacle + 2] = angle1[number_obstacle + 3];
				angle2[number_obstacle + 2] = angle2[number_obstacle + 3];
				min[number_obstacle + 2] = min[number_obstacle + 3];
				angle_min[number_obstacle + 2] = angle_min[number_obstacle + 3];
				if(number_obstacle < quantityr_obstacles - 2) // ����� �� ����� �� ������ ������� 
				{
					number_obstacle = number_obstacle + 1;
				}
				else
				{
					number_obstacle = quantityr_obstacles;
				}
				Flag = Flag + 1;
			}
			else
			{
				Flag = Flag - 1;
				number_obstacle = number_obstacle + 1;
			}
		}
		else
		{
			number_obstacle = number_obstacle + 1;
		}
	}
	quantityr_obstacles = quantityr_obstacles - Flag; //��������� ���������� ����������� 
	vector<obstacle> a(quantityr_obstacles);
	mas.num = quantityr_obstacles; // ��������� ����������� ����������� 
	for (int i = 0; i < quantityr_obstacles; i++) // ��������� ������ �������������
	{
	    a[i].nomer = nomer[i];
	    a[i].angle1 = angle1[i];
	    a[i].angle2 = angle2[i];
	    a[i].min_distance = min[i];
		a[i].angle_min = angle_min[i];
	//    ROS_INFO("(%d, %0.2f, %0.2f, %0.2f)", a[i].nomer, a[i].angle1, a[i].angle2, a[i].min_distance);
	}
	mas.mass = a; // ��������� ������ �������������
	chatter_pub.publish(mas); // ��������� ������ � ����� 
	return quantityr_obstacles;
}

// �������� ������ � ������ 
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int r=0;
    float A[360];
    for (int i = 0; i < 360; i++) //��������� ������ ������� � ������  
    {
        A[i] = (msg->ranges[i]);
		//ROS_INFO ("(<%d>,<%0.2f>)", i , A[i]);
    }
	
    r = StudyData(A);
	
    //ROS_INFO("(<%d>)", r);
    //ROS_INFO( "( %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f )", msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max );
}


int main(int argc, char **argv)
{	
	ros::init (argc, argv, "findObstacles"); //������������� ����
	ros::NodeHandle nh("~"); //���������� ����������� nh � ������������ ���� NodeHandle
	ros::Subscriber scanSub; //��������� ���� result �����������
	
	nh.param("error", error, 0.20);
	nh.param("degree", degree, 10.0);
	nh.param("zero", zero, 0);
	nh.param("circle", circle, 360);
	nh.param("endElement", endElement, 359);
	nh.param("gap", gap, 1);
	nh.param("minLength", minLength, 2);
	scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan); //�������� �� ������ � Lidar

	ros::NodeHandle n;
	chatter_pub = n.advertise<obstacles>("obstacles",1000); //���������� ������������ ������ (����� �����������)
	ros::spin(); 
	
	return 0;
}




