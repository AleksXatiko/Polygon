#include "communication.h"
#include "power.h"
#include "computationMovement.h"
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <ctime>
#include "poly_ros/obstacles.h"
#include "poly_ros/points.h"
//#include "poly_ros/Num.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int algorithm, currentAction, move_mode;
int moveLocalCoordinates, range1, range2, zero;
double distinction, pos_x, pos_y;

struct Vector
{
	float X, Y;

	float GetLength()
	{
		return sqrt(pow(X, 2) + pow(Y, 2));
	}

	Vector(float x, float y)
	{
		X = x;
		Y = y;
	}
};

void ReceiveObstacleAvoidanceTrajectory(const poly_ros::points::ConstPtr& pts)
{
	int k = pts->num;
	if (move_mode == 2)
	{
		float max = -20000, step_x, step_y;
		for (float x = current_x - 1; x <= current_x + 1; x += 0.1)
		{
			for (float y = current_y - 1; y <= current_y + 1; y += 0.1)
			{
				if (y != current_y || x != current_x)
				{
					Vector v1 = Vector(target_x - x, target_y - y);
					float len = 0.1f / v1.GetLength() * k;
					for (int i = 0; i < k; i++)
					{
						Vector v2 = Vector(pts->points[i].x - x, pts->points[i].y - y);
						len += -0.2f / v2.GetLength();
					}
					if (len > max)
					{
						max = len;
						step_x = x;
						step_y = y;
					}
				}
			}
		}
		pos_x = step_x * 10;
		pos_y = step_y * 10;
		printf("											| %0.3f | %0.3f |\n", step_x, step_y);
	}
}

int main(int argc, char **argv)
{
	//������������� ROS-����, ������ ���� � ����� ������
    ros::init(argc, argv, "main");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

	nhPrivate.param("pos_x", pos_x, 0.0);
	nhPrivate.param("pos_y", pos_y, 0.0);
	nhPrivate.param("algorithm", algorithm, 1);
    nhPrivate.param("movelocalCoorginates", moveLocalCoordinates, 84);
    nhPrivate.param("distinction", distinction, 0.2);
    nhPrivate.param("range1", range1, 50);
    nhPrivate.param("range2", range2, 310);
	//nhPrivate.param("min_distance", min_distance, 0.45);
    nhPrivate.param("zero", zero, 0);
    nhPrivate.param("move_mode", move_mode, 0);
	
    ROS_INFO("movee=%d", algorithm);

	//�� ������� ���� � ��������� ������ 
    target_set=0;
    position_set=0;

    ROS_INFO("Poly ON 1");
    ROS_DEBUG("Poly ON debug");

	//���������� �� ����� ������� PixHawk �����������
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
			
    ros::Subscriber sub = nh.subscribe("obstacles" , 1000, chatterCallback); 
	
    ros::Subscriber sub_obst = nh.subscribe("points" , 1000, ReceiveObstacleAvoidanceTrajectory); //new new new
	
	ros::Subscriber test_sub = nh.subscribe("target" , 1000, test);
	
	ros::Subscriber subs = nh.subscribe("robotModel_parametrs", 1000, GetData); //UUUUUUUU
			
    //ros::Subscriber subs = nh.subscribe("Num", 1000, &chatter);
	//���������������� � �������� ��������� ��� ������, ������� �������  ����������� ������
    ros::Publisher to_gcs = nh.advertise<mavros_msgs::Mavlink>
            ("mavlink/from", 10);
			
	//���������������� � �������� ��������� ��� ������, ������� ������� ���������� � RC-�������� (*1) �� PixHawk
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 1);
			

    //��������� �������, � ������� ����� �������������� ������ � ����� ���� � 20�� (������ ���� �� ����� 2��)
    ros::Rate rate(20.0);

    ROS_INFO("Wait for FCU connection...");

    
	//�������, ���� �� ����������� � PixHawk
    while(ros::ok() && !current_state.connected)
	{
       ros::spinOnce();
       rate.sleep();
	};

    ROS_INFO("FCU connected");
	//���������� �� ����� c MAVLink-���������, ������������ � ������������ �������
    ros::Subscriber to_px = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/to", 10, to_px_cb);
			
	//���������� �� ����� c ����� ������� ����������, ������� ��������� �����
    ros::Subscriber lidar_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("slam_out_pose", 10, lidar_cb);
			
	//��������� ������ ��� ���������� ARMED ���������� ����������� PixHawk (�������� ��������, ���� !ARMED, �� ��������� ���������)
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
			
	//��������� ������ ��� ���������� MODE ���������� ����������� PixHawk (����� ������ ���������� PixHawk - ���������, ������ � �.�.)
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	//���������� ������� ��� PixHawk ��� �������� � MODE=MANUAL ��� ������������ ���������� ������� RC-��������� (*1)
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";
	
	//���������� ������� ��� PixHawk ��� �������� � ARMED-����� ��� ������������ ���������� ������� RC-��������� (*1)
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	
	//��� ����� ������� ��������� ����� �������� ������� �� ����� ������ PixHawk �����������, ����� �� ����� ���� ������ ������� �����
    ros::Time last_request = ros::Time::now();

	//���������� ���������� ��� ������
    mavros_msgs::OverrideRCIn rc; //��� ����� ����������� ������ RC-������� ��� �������� �� PixHawk
    geometry_msgs::PoseStamped local_pose;//��� �����
    mavros_msgs::Mavlink rmsg; //��� ����� ����������� ��������� ��� �������� �� ����������� ������ (���� ������� ���������)
	
	//���������� MAVLink � ����� �������� ��������� �� 1.0 ������ ��������� MAVLink.
    auto stat = mavlink_get_channel_status(0);
    stat->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;


    if( current_state.mode == "MANUAL" )
    {
	ROS_INFO("Start state: MANUAL mode enabled!");
    }
    else
    {
	ROS_INFO("Start state: No MANUAL mode enabled!");
    };
    if( current_state.armed )
    {
	ROS_INFO("Start state: Vehicle ARMED!");
    }
    else
    {
	ROS_INFO("Start state: Vehicle NOT ARMED!");
    };

    while(ros::ok())
	{
	    //���� �� ����� ����������� �� ������������� MANUAL ��� �� �� ARMED, �� ��������� ��������� ��� � ������ ���������.
	    if(ros::Time::now() - last_request > ros::Duration(2.0)){
		if( current_state.mode != "MANUAL" )
		{
			ROS_INFO("Try enable MANUAL mode");
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){ ROS_INFO("Manual enabled"); };
		};
			if( !current_state.armed && current_state.mode == "MANUAL" ){
				ROS_INFO("Try arm vehicle");
			if( arming_client.call(arm_cmd) && arm_cmd.response.success){ 
				ROS_INFO("Vehicle armed"); };
			};
				
				last_request = ros::Time::now();
			};

		//������� �� GCS ��� ��������� � ������������
			if(send_local_pos(rmsg)) to_gcs.publish(rmsg);
			if(send_attitude(rmsg)) to_gcs.publish(rmsg);

		double local_angle_to_target; //������� ���� ����������� �������� ����
		currentAction = ACTION_STAY; //������� ��������� ������:�����
		double control = 0.0;//����������� �����������
		//algorithm = 3; //��������� �������� ��������
		//ROS_INFO("ALGORITHM %d", algorithm);
		
		double x, y;
		if (pos_x == 0 && pos_y == 0)
		{
			x = target_x;
			y = target_y;
		}
		else
		{
			x = pos_x;
			y = pos_y;
			target_set = 1;
		}
		//���� �����, ��� �� � ���� ��� ���� - ���������� �����-�� ��������, ���� �����
		if( target_set && position_set )
			//������, ������ �� ���������  ������, ��������� �������� ���� � ���� ������ ������������ ����, ����� �������� ����� �����������, ����� ��������� � �������� �����
			currentAction = getAction(current_x, current_y, current_yaw, x, y, &local_angle_to_target);
			
			//������������ �� ����������� �������� ���������� ���������� ����������� �� ����� 			
			control = controlAction(algorithm, currentAction, local_angle_to_target);
			
			//������� ���������� �������� ��� ���������� ������� ����, ������� ������ ������� �� �����, ����� �� �����(��� ��������) ��������
			updateRCfromAction(algorithm, control, currentAction,rc);
			rc_pub.publish(rc);
			
			//���������� �� ��� ����� ROS
			ros::spinOnce();
			
			//������ ����� ��� ������ ��� ������� ������ (��. ����������� ���������� rate ����) � ��������� ������
			rate.sleep();
	}
    return 0;
}
