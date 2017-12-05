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
отыскивает разрыв, начиная с A[0]. Возвращает R0-угол, где впервые втретился разрыв, между A[R0] и A[R0+1].
В противном случае вернуть признак отсутствия разрыва (NULL)
*/
int FirstBreak(float C[360], float *min1_distance, int *min_angle)
{
	int i = 0;
	*min1_distance = C[0];  //минимальная удаленность препятствия
	*min_angle = 0;
	int R0 = 0;   //Ro - первый разрыв
				  //пока не найден разрыв или не кончатся все элементы
	while (R0 == zero && i < circle)  //Будем искать пока не найден разрыв (Ro = 0) или пока не прошли весь круг
	{
		if (abs(C[i] - C[i + 1]) <= error) //Если текущий элемент и следующий примерно совпадают
		{
			    if (C[i] < *min1_distance) //Ищем минимальную удаленность 
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

	return R0; //Возвращаем первый разрыв

}


// Изучение данных
int StudyData(float B[360])
{
	int num = 360; //размер массива
	poly_ros::obstacle ms;
	poly_ros::obstacles mas;
	int *nomer= new int[num];// Выделение памяти для массива
	int *angle1= new int[num];// Выделение памяти для массива
	float *min = new float[num]; // Выделение памяти для массива
	int *angle2= new int[num];// Выделение памяти для массива
	int *angle_min = new int[num];// Выделение памяти для массива
	int First_R0;  //Ro - первый разрыв
	int R01;//Ro - первый разрыв
	float min1_distance = 0.0;//минимальнаЯ удаленность препятствия
	int min1_angle = 0; //угол до минимального расстояния
	First_R0 = FirstBreak(B, &min1_distance, &min1_angle);
	R01 = First_R0 - 1;
	float min11_distance = min1_distance;//минимальная удаленность препятствия
	int min11_angle = min1_angle; //угол до минимального расстояния
	float min_distance1;//минимальная удаленность препятствия
	int angle3; //угол до минимального расстояния
	int Ri = First_R0 + 1; //угл начала препятствия 
	First_R0 = First_R0 + 1;// угл конца препятсвия
	int NUL = 0; // признак отстуствия разрыва
	int quantityr_obstacles = 0;// количество препятствий 
	
	if (First_R0 != zero)
	{	
		min_distance1 = B[First_R0 + 1];  //минимальнаЯ удаленность препятствия
		angle3 = First_R0 + 1; //угол до минимальной удаленности
		
		while (First_R0 < circle)       //пока не найден круг
		{
			if (abs(B[First_R0] - B[First_R0 + 1]) <= error)//текущий элемент и следующий примерно равны, то будем считать, что это одно препятствие
			{
				NUL = 0;
			}
			else
			{
				NUL = 1;
			}
			if (B[First_R0] < min_distance1 && B[First_R0] > 0.2) //Ищем минимальную удаленность 
			{
				min_distance1 = B[First_R0];
				angle3 = First_R0;
			}

			if (NUL == gap) // если есть разрыв
			{
				if  (First_R0 != endElement) // смотрим все углы до 360
				{
				    if(abs(Ri-First_R0) > minLength) //если размер препятствия больше 2
					{
						angle1[quantityr_obstacles] = Ri; // запишем угол начала препятствия 
						min[quantityr_obstacles] = min_distance1;// запишем минимальную дистанцию 
						angle_min[quantityr_obstacles] = angle3; //запишем угол до минимального расстояния
						angle2[quantityr_obstacles] = First_R0;// запишем угол конца препятствия
						nomer[quantityr_obstacles] = quantityr_obstacles + 1; // запишем номер препятствия  
						quantityr_obstacles = quantityr_obstacles + 1;// увеличим количество препятствий 
					}
					NUL = 0;
					Ri = First_R0 + 1; 
					min_distance1 = B[First_R0 + 1];	
					angle3 = First_R0 + 1;
				}
			}
			First_R0 = First_R0 + 1 ;
		}
		
		if (B[First_R0] < min_distance1 && B[First_R0] > 0.2) //Ищем минимальную удаленность 
			{
				min_distance1 = B[First_R0];
				angle3 = First_R0;
			};


		if (((abs(B[359] - B[0])) > error) && (abs(B[359] - B[0]) != B[359])) //Есть разрыв  между А [360] и A[0?]
		{
			angle1[quantityr_obstacles] = Ri; // запишем угол начала препятствия
			min[quantityr_obstacles] = min_distance1;// запишем минимальную дистанцию 
			angle_min[quantityr_obstacles] = angle3; //запишем угол до минимального расстояния
			angle2[quantityr_obstacles] = First_R0;// запишем угол конца препятствия
			nomer[quantityr_obstacles] = quantityr_obstacles + 1;// запишем номер препятствия 
			quantityr_obstacles = quantityr_obstacles + 1;// увеличим количество препятствий 
			Ri = 0;
			
		};

		if(Ri==zero)
			{
				min[quantityr_obstacles] = min_distance1;// запишем минимальную дистанцию 
				angle_min[quantityr_obstacles] = angle3; //запишем угол до минимального расстояния
				angle2[quantityr_obstacles] = First_R0;// запишем угол конца препятствия
			}
		else
			{
				if (min11_distance > min_distance1) //Ищем максимальную удаленность 
				{
					min1_distance = min_distance1;
					min1_angle = angle3;
				}
				min[quantityr_obstacles] = min11_distance; // запишем минимальную дистанцию 
				angle_min[quantityr_obstacles] = min11_angle; //запишем угол до минимального расстояния
				if (abs(B[359] - B[0]) == B[359])
				    {
						angle2[quantityr_obstacles] = 359; // запишем угол конца препятствия
				    }
				    else
				    {
						angle2[quantityr_obstacles] = R01; // запишем угол конца препятствия
				    }
			}
			angle1[quantityr_obstacles] = Ri; // запишем угол начала препятствия
			nomer[quantityr_obstacles] = quantityr_obstacles + 1; // запишем номер препятствия
			quantityr_obstacles = quantityr_obstacles + 1;// увеличим количество препятствий 
	}
	else
	{
		angle1[quantityr_obstacles] = Ri; // запишем угол начала препятствия
		min[quantityr_obstacles] = min1_distance;// запишем минимальную дистанцию
		angle_min[quantityr_obstacles] = min1_angle; //запишем угол до минимального расстояния
		angle2[quantityr_obstacles] = First_R0;// запишем угол конца препятствия
		nomer[quantityr_obstacles] = quantityr_obstacles + 1;// запишем номер препятствия 
		quantityr_obstacles = quantityr_obstacles + 1;// увеличим количество препятствий 
	}

	int number_obstacle = 0; // номер препятствия 
	int Flag = 0; //флажек который следит за сокращение количества препятствий 

	/* В цикле объединяем элементы если длина препятствия меньше чем degree и 
	растояние до конечного угла предыдущего препятствия приблизительно равно 
	растоянию до начального угла другого препятствия то это одно препятствие  */
	while(number_obstacle < quantityr_obstacles)
	{
		if(abs(angle1[number_obstacle+1] - angle2[number_obstacle+1]) < degree) // если длина препятствия больше чем degree то это препятствие 
		{
			// елси растояние до конечного угла предыдущего препятствия приблизительно равно растоянию до начального угла другого препятствия то это одно препятствие 
			if((abs(B[angle2[number_obstacle - Flag]] - B[angle1[number_obstacle + 1]]) < error))   
			{
				angle2[number_obstacle - Flag] = angle1[number_obstacle + 1]; //увиличиваем длину препятствия
				if (min[number_obstacle - Flag] > min[number_obstacle + 1]) // находим минимальную дистанцию
				{
					min[number_obstacle - Flag] = min[number_obstacle + 1];
					angle_min[number_obstacle - Flag] = angle_min[number_obstacle + 1]; 
				}
				//смещяем элементы массива
				angle1[number_obstacle + 1] = angle1[number_obstacle + 2];
				angle2[number_obstacle + 1] = angle2[number_obstacle + 2];
				min[number_obstacle + 1] = min[number_obstacle + 2];
				angle_min[number_obstacle + 1] = angle_min[number_obstacle + 2];
				angle1[number_obstacle + 2] = angle1[number_obstacle + 3];
				angle2[number_obstacle + 2] = angle2[number_obstacle + 3];
				min[number_obstacle + 2] = min[number_obstacle + 3];
				angle_min[number_obstacle + 2] = angle_min[number_obstacle + 3];
				if(number_obstacle < quantityr_obstacles - 2) // чтобы не выйти за предел массива 
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
	quantityr_obstacles = quantityr_obstacles - Flag; //уменьшаем количество препятствий 
	vector<obstacle> a(quantityr_obstacles);
	mas.num = quantityr_obstacles; // публикуем количествой препятствий 
	for (int i = 0; i < quantityr_obstacles; i++) // публикуем данные опрепятствиях
	{
	    a[i].nomer = nomer[i];
	    a[i].angle1 = angle1[i];
	    a[i].angle2 = angle2[i];
	    a[i].min_distance = min[i];
		a[i].angle_min = angle_min[i];
	//    ROS_INFO("(%d, %0.2f, %0.2f, %0.2f)", a[i].nomer, a[i].angle1, a[i].angle2, a[i].min_distance);
	}
	mas.mass = a; // публикуем данные опрепятствиях
	chatter_pub.publish(mas); // публикуем данные в топик 
	return quantityr_obstacles;
}

// получаем данные с лидара 
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int r=0;
    float A[360];
    for (int i = 0; i < 360; i++) //заполняем массив данными с лидара  
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
	ros::init (argc, argv, "findObstacles"); //инициализация узла
	ros::NodeHandle nh("~"); //объявление преерменной nh в пространстве имен NodeHandle
	ros::Subscriber scanSub; //обьявляем узел result подписчиком
	
	nh.param("error", error, 0.20);
	nh.param("degree", degree, 10.0);
	nh.param("zero", zero, 0);
	nh.param("circle", circle, 360);
	nh.param("endElement", endElement, 359);
	nh.param("gap", gap, 1);
	nh.param("minLength", minLength, 2);
	scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan); //подписка на данные с Lidar

	ros::NodeHandle n;
	chatter_pub = n.advertise<obstacles>("obstacles",1000); //публикация обработанных данных (карта препятствий)
	ros::spin(); 
	
	return 0;
}




