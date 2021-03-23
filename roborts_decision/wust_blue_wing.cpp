#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"


int main(int argc, char **argv) {           //argv是一个字符串数组，代表从命令行传递进来的参数(参数向量)。
  ros::init(argc, argv, "wust_bw");       
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

  //第一步，从机+100子弹无条件执行
  Go_bullt_buff();
  printf("go bullt buff 1");
  //第二步，转回埋伏点防守，底盘左右摆动
  Go_wing_ambush_point();
  printf("go ambush point 2");
  /*底盘左右摆动函数*/


  while(ros::ok()){
    ros::spinOnce();

    blackboard->IsInStuckArea();

    // shoot and dodge command when game is on!  当游戏开始时，射击和躲避命令

    if (blackboard->CanShoot()) blackboard->Shoot(blackboard->info.shoot_hz);
    // my pose
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

