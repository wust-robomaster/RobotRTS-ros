#include <ros/ros.h>
#include <iostream> 
#include <unistd.h> 

//#include "../roborts_base/include/gimbal.h"

#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/shoot_behavior.h"

int shoot_flag=0, count=0;
    ros::ServiceClient shoot_client;
    ros::ServiceClient FricWheel_client;

void FriWhl_ctrl(bool s)
{
  roborts_msgs::FricWhl fric_srv;

  fric_srv.request.open = s;

  if (FricWheel_client.call(fric_srv) && s == true)
    {
      ROS_INFO("Open the fri-wheel!");
    }
  else if (s == false)
    {
      ROS_INFO("Close the fri-wheel!");
    }
  else
    {
      ROS_ERROR("Failed to call fri-wheel service!");
    }
}

void Shoot_ctrl(int shoot_mode, int shoot_num)
{
  roborts_msgs::ShootCmd shoot_srv;

  shoot_srv.request.mode = shoot_mode;
  shoot_srv.request.number = shoot_num;

  if (shoot_client.call(shoot_srv))
    {
      ROS_INFO("Shoot now!");
    }
  else{
      ROS_ERROR("Failed to call shoot client");
    }
}

void SHOOT_STAR(bool fri_ctrl, int shoot_mode, int shoot_num)
{
    FriWhl_ctrl(fri_ctrl);
    printf("ARE YOU HAPPY?");
    Shoot_ctrl(shoot_mode, shoot_num);
    printf("HAPPY!");
}

void Shoot_delay(int count_limit)
{
    ros::Rate loop(10);
   
    while (count < count_limit)
    {
        count++;
        loop.sleep();
       // if (detected_enemy_ == true)
        if(count < count_limit)
	{
            SHOOT_STAR(true, 2, 5);
            printf("shoot!!");
            /*�������Ұڶ�����*/
        }
        else 
        {
            shoot_flag = 1;
            printf("shoot_flag!!!");
            break;
    }

}
}



int main(int argc, char **argv) {           //argv��һ���ַ������飬�����������д��ݽ����Ĳ���(��������)��
  ros::init(argc, argv, "wust_shoot_test");       
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  //auto gambal_executor = new roborts_decision::GambalExecutor;

  auto blackboard = new roborts_decision::Blackboard(full_path);
    ros::NodeHandle nh;

    ros::service::waitForService("cmd_fric_wheel");
    FricWheel_client = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ROS_WARN("*******");

    ros::service::waitForService("cmd_shoot");
    shoot_client = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
    ROS_WARN("what???");
    
    ros::Rate rate(10);

  // for filter noise command �����˲���������
  unsigned int count=0;
  const unsigned int count_bound = 3;


   //��⵽���ˣ�����ʮ�������û��⵽��������ȱ���̰ڶ�


  while(ros::ok()){
    ros::spinOnce();
    geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();


    if(blackboard->IsNewEnemy())
    {
      SHOOT_STAR(true, 2, 10);
      Shoot_delay(10);
    }
    else
    {
       printf(":????????????????????????????????????????/");
       
    }
    
    rate.sleep(); 
  }


  return 0;
}


