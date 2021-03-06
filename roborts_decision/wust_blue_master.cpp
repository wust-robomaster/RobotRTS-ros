#include <ros/ros.h>

#include <unistd.h> 

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"


int main(int argc, char **argv) {           //argv是一个字符串数组，代表从命令行传递进来的参数(参数向量)。
  ros::init(argc, argv, "wust_bm");       
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/blue_wing.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  // Behavior State Enum
  BehaviorStateEnum last_state, cur_state;  // last_state 中断前的状态  cur_state 当前状态
  last_state = BehaviorStateEnum::INIT;
  cur_state = BehaviorStateEnum::INIT;


  ros::Rate rate(10);

  // for filter noise command 用于滤波噪声命令
  unsigned int count=0;
  const unsigned int count_bound = 3;

  //第一步，快速进入伏击点，射击红方从机
  Go_master_shoot_point();
  printf("go master ambush point 3");
  //计时十秒？
  Shoot_delay(10); //检测到敌人，进行十秒射击，没检测到则跳出，缺底盘摆动
  //返回埋伏区
  Go_master_ambush_point();


  while(ros::ok()){
    ros::spinOnce();

    blackboard->IsInStuckArea();

    geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();

    //比赛开始时，获取子弹
    if (blackboard->info.strategy == "go_buff"){
            // state decision behavior
            if (blackboard->info.remain_hp >= 400){
                // according bullet to the buff  
                //依据子弹去找buff
                if (blackboard->info.remain_bullet > 0){
                    if ( (!blackboard->info.has_buff && blackboard->info.times_to_buff >0 
                        && blackboard->GetDistance(blackboard->info.ally, blackboard->info.my_shield )>= blackboard->threshold.near_dist)
                        || blackboard->info.is_shielding)   //如果满足 以上，则防御
                 
                        {
                            cur_state = BehaviorStateEnum::SHIELD; 
                      }
                    else if (!blackboard->info.has_my_enemy  && !blackboard->info. ){   //没有敌人1，2，搜索
                        cur_state = BehaviorStateEnum::SEARCH;
                    }
                    else{
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){   //相机识别敌方装甲板，伏击
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }
                     else if (blackboard->info.has_ally_enemy){
                            cur_state = BehaviorStateEnum::ATTACK;    // 攻击
                        }
                    }
                }
                // not enough bullet  弹药不足  防护，保护自己，  装弹，逃跑
                else{
                    if (last_state == BehaviorStateEnum::SHIELD  && !blackboard->info.has_buff && blackboard->info.times_to_buff>0
                       && blackboard->GetDistance(mypose, blackboard->info.my_shield)<=blackboard->threshold.near_dist
                       || blackboard->info.is_shielding){
                         cur_state = BehaviorStateEnum::SHIELD;
                    }
                    else if ( ((blackboard->info.times_to_supply >0    //首先时间大于0   其次，1.有盟友& 弹舱 ，2. 有供给   血量没有小于600
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
            // not enought hp     没有足够血量
            //子弹大于0
            else{
                if (blackboard->info.remain_bullet > 0){
                    if (!blackboard->info.has_my_enemy  && !blackboard->info.has_ally_enemy){
                        cur_state = BehaviorStateEnum::SEARCH;    //没有敌人1，没有敌人2
                    }
                    else    //找到了了敌军，或有效的装甲板   ，伏击
                    {
                        if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){  
                            cur_state = BehaviorStateEnum::AMBUSH;
                    
                        }  
                        else if(blackboard->info.has_ally_enemy){      // 同伴找到敌人，攻击
                            cur_state = BehaviorStateEnum::ATTACK;
                        }
                    }
                
                }
                else if (blackboard->info.ally_remain_hp < blackboard->info.remain_hp && blackboard->info.times_to_supply >0  || blackboard->info.is_supplying)
                {
                    cur_state = BehaviorStateEnum::RELOAD;    //友方有血量& 有时间  或正在装弹  则装弹，否则 逃跑
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
    ros::Rate loop(10); //ros::Rate loop（50）规定了循环的频率50Hz

    while (count < count_limit)
    {
        count++;
        loop.sleep();
        if (detected_enemy_ == true)
        {
            SHOOT_STAR(true, 1, 5);
            /*底盘左右摆动函数*/
        }
        else break;
    }

}