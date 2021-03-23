#include <ros/ros.h>

#include <unistd.h> 

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"


int main(int argc, char **argv) {           //argv��һ���ַ������飬�����������д��ݽ����Ĳ���(��������)��
  ros::init(argc, argv, "wust_bm");       
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/blue_wing.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  // Behavior State Enum
  BehaviorStateEnum last_state, cur_state;  // last_state �ж�ǰ��״̬  cur_state ��ǰ״̬
  last_state = BehaviorStateEnum::INIT;
  cur_state = BehaviorStateEnum::INIT;


  ros::Rate rate(10);

  // for filter noise command �����˲���������
  unsigned int count=0;
  const unsigned int count_bound = 3;

  //��һ�������ٽ�������㣬����췽�ӻ�
  Go_master_shoot_point();
  printf("go master ambush point 3");
  //��ʱʮ�룿
  Shoot_delay(10); //��⵽���ˣ�����ʮ�������û��⵽��������ȱ���̰ڶ�
  //���������
  Go_master_ambush_point();


  while(ros::ok()){
    ros::spinOnce();

    blackboard->IsInStuckArea();

    geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();

    //������ʼʱ����ȡ�ӵ�
    if (blackboard->info.strategy == "go_buff"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                // according bullet to the buff  
                //�����ӵ�ȥ��buff
                if (blackboard->info.remain_bullet > 0){
                    if ( (!blackboard->info.has_buff && blackboard->info.times_to_buff >0 
                        && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield )>= blackboard->threshold.near_dist)
                        || blackboard->info.is_shielding)   //������� ���ϣ������
                 
                        {
                            cur_state = BehaviorStateEnum::SHIELD; 
                      }
                    else if (!blackboard->info.has_my_enemy  && !blackboard->info. ){   //û�е���1��2������
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){   //���ʶ��з�װ�װ壬����
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                     else if (blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;    // ����
                        }
                    }
                }
                // not enough bullet  ��ҩ����  �����������Լ���  װ��������
                else{
                    if (last_state == BehaviorStateEnum::SHIELD  && !blackboard->info.has_buff && blackboard->info.times_to_buff>0
                       && blackboard->GetDistance(mypose, blackboard->info.my_shield)<=blackboard->threshold.near_dist
                       || blackboard->info.is_shielding){
                         cur_state = BehaviorStateEnum::SHIELD;
                    }
                    else if ( ((blackboard->info.times_to_supply >0    //����ʱ�����0   ��Σ�1.������& ���� ��2. �й���   Ѫ��û��С��600
                       && (blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_reload)>= blackboard->threshold.near_dist)) || blackboard->info.is_supplying
                       )
                       &&  !(blackboard->info.is_hitted && blackboard->info.remain_hp<=600)
                    ){
                        cur_state = BehaviorStateEnum::RELOAD;
                    }
                    else{
                        cur_state = BehaviorStateEnum::ESCAPE;
                    }

                }
                
            }
            // not enought hp     û���㹻Ѫ��
            //�ӵ�����0
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;    //û�е���1��û�е���2
                    }
                    else    //�ҵ����˵о�������Ч��װ�װ�   ������
                    {
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){  
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }  
                        else if(blackboard->info.has_ally_enemy){      // ͬ���ҵ����ˣ�����
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                
                }
                else if (blackboard->info.ally_remain_hp < blackboard->info.remain_hp && blackboard->info.times_to_supply >0  || blackboard->info.is_supplying)
                {
                    cur_state = BehaviorStateEnum::RELOAD;    //�ѷ���Ѫ��& ��ʱ��  ������װ��  ��װ�������� ����
                }
                else{
                    cur_state = BehaviorStateEnum::ESCAPE;
                }
            }
            
    }
        


    
    
    // filter
    if (!(last_state == BehaviorStateEnum::RELOAD  ||  last_state == BehaviorStateEnum::SHIELD)){
        count = last_state != cur_state ? count + 1: 0;
        cur_state = count>=count_bound? cur_state: last_state;
    }
    
   
    // cancel last state
    if ( (last_state != BehaviorStateEnum::INIT && last_state != cur_state)  || blackboard->info.remain_hp<=0 ){
         switch (last_state){
            case BehaviorStateEnum::BACKBOOT:
                back_boot_area_behavior.Cancel();
                break;
            case BehaviorStateEnum::CHASE:
                chase_behavior.Cancel();
                break;
            case BehaviorStateEnum::ESCAPE:
                escape_behavior.Cancel();
                break;
            case BehaviorStateEnum::PATROL:
                patrol_behavior.Cancel();
                break;
            case BehaviorStateEnum::RELOAD:
                reload_behavior.Cancel();
                break;
            case BehaviorStateEnum::SHIELD:
                shield_behavior.Cancel();
                break;
            case BehaviorStateEnum::SEARCH:
                search_behavior.Cancel();
                break;
            case BehaviorStateEnum::AMBUSH:
                ambush_behavior.Cancel();
                break;
            case BehaviorStateEnum::ATTACK:
                attack_behavior.Cancel();
                break;
            case BehaviorStateEnum::NN:
                nn_behavior.Cancel();
                break;
         }
    }

    // remain hp is 0, them dead!
    if (blackboard->info.remain_hp <=0){
        ros::shutdown();
        break;
    }

    switch (cur_state){
      case BehaviorStateEnum::BACKBOOT:
          back_boot_area_behavior.Run();
          std::cout<<"BackBoot" << std::endl;
          break;
      case BehaviorStateEnum::CHASE:
          chase_behavior.Run();
          std::cout<<"CHASE" << std::endl;
          break;
      case BehaviorStateEnum::ESCAPE:
          escape_behavior.Run();
          std::cout<<"ESCAPE" << std::endl;
          break;
      case BehaviorStateEnum::PATROL:
          patrol_behavior.Run();
          std::cout<<"PATROL" << std::endl;
          break;
      case BehaviorStateEnum::RELOAD:
          reload_behavior.Run();
          std::cout<<"RELOAD" << std::endl;
          break;
      case BehaviorStateEnum::SHIELD:
          shield_behavior.Run();
          std::cout<<"SHIELD" << std::endl;
          break;
      case BehaviorStateEnum::SEARCH:
          search_behavior.Run();
          std::cout<<"SEARCH" << std::endl;
          break;
      case BehaviorStateEnum::AMBUSH:
          ambush_behavior.Run();
          std::cout<<"AMBUSH" << std::endl;
          break;
      case BehaviorStateEnum::ATTACK:
          attack_behavior.Run();
          std::cout<<"ATTACK" << std::endl;
          break;

        case BehaviorStateEnum::NN:
          nn_behavior.Run();
          std::cout<<"NN" << std::endl;
          break;

    }

    last_state = cur_state;
    
    rate.sleep(); 
 
   
    
  }


  return 0;
}

void FriWhl_ctrl(bool s)
{
    roborts_msgs::FricWhl fric_srv;
    fric_srv.request.open = s;
    if (FricWheel_client.call(fric_srv) && s == true)
        ROS_INFO("Open the fri-wheel!");
    else if (s == false)
        ROS_INFO("Close the fri-wheel!");
    else
        ROS_ERROR("Failed to call fri-wheel service!");
}

void Shoot_ctrl(int shoot_mode, int shoot_num)
{
    roborts_msgs::ShootCmd shoot_srv;

    shoot_srv.request.mode = shoot_mode;
    shoot_srv.request.number = shoot_num;

    if (shoot_client.call(shoot_srv))
        ROS_INFO("Shoot now!");
    else 
        ROS_ERROR("Failed to call shoot client");
}

void SHOOT_STAR(bool fri_ctrl, int shoot_mode, int shoot_num)
{
    ros::init(argc, argv, "shoot_client");
    ros::NodeHandle nh;

    ros::service::waitForService("cmd_fric_wheel");
    FricWheel_client = nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros::service::waitForService("cmd_shoot");
    shoot_client = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    FriWhl_ctrl(fri_ctrl);
    ROS_INFO("ARE YOU HAPPY?");
    Shoot_ctrl(shoot_mode, shoot_num);
    ROS_WARN("HAPPY!");
}

void Shoot_delay(int count_limit)
{
    ros::Rate loop(10); //ros::Rate loop��50���涨��ѭ����Ƶ��50Hz

    while (count < count_limit)
    {
        count++;
        loop.sleep();
        if (detected_enemy_ == true)
        {
            SHOOT_STAR(true, 1, 5);
            /*�������Ұڶ�����*/
        }
        else break;
    }

}