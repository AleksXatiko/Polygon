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
//#include "poly_ros/Num.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int algorithm, pos_x, pos_y;
int regular;
int moveLocalCoordinates, range1, range2, zero;
double distinction;
double min_distance;

int main(int argc, char **argv)
{
	//������������� ROS-����, ������ ���� � ����� ������
    ros::init(argc, argv, "main");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

	nhPrivate.param("pos_x", pos_x, 0);
	nhPrivate.param("pos_y", pos_y, 0);
	nhPrivate.param("algorithm", algorithm, 1);
    nhPrivate.param("regular", regular, 1);
    nhPrivate.param("movelocalCoorginates", moveLocalCoordinates, 84);
    nhPrivate.param("distinction", distinction, 0.2);
    nhPrivate.param("range1", range1, 50);
    nhPrivate.param("range2", range2, 250);
    nhPrivate.param("min_distance", min_distance,  0.45);
    nhPrivate.param("zero", zero, 0);
    ROS_INFO("movee=%d",regular );

	//�� ������� ���� � ��������� ������ 
    target_set=0;
    position_set=0;

    ROS_INFO("Poly ON 1");
    ROS_DEBUG("Poly ON debug");

	//���������� �� ����� ������� PixHawk �����������
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
			
    ros::Subscriber sub = nh.subscribe("obstacles" , 1000, chatterCallback);  //new new new
			
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
		int currentAction = ACTION_STAY; //������� ��������� ������:�����
		double control = 0.0;//����������� �����������
		//algorithm = 3; //��������� �������� ��������
		//ROS_INFO("ALGORITHM %d", algorithm);

		//���� �����, ��� �� � ���� ��� ���� - ���������� �����-�� ��������, ���� �����
		if( target_set && position_set )
			
			if (pos_x != 0 || pos_y != 0)
			{
				target_x = pos_x;
				target_y = pos_y;
			}
			//������, ������ �� ���������  ������, ��������� �������� ���� � ���� ������ ������������ ����, ����� �������� ����� �����������, ����� ��������� � �������� �����
			currentAction = getAction(current_x, current_y, current_yaw, target_x, target_y, &local_angle_to_target);
			
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
