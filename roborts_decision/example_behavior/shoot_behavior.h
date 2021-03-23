#ifndef SHOOT_BEHAVIOR_H
#define SHOOT_BEHAVIOR_H

//#include "../../roborts_base/include/gimbal.h"

#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/ShootCmd.h"

void FriWhl_ctrl(bool s);

void Shoot_ctrl(int shoot_mode, int shoot_num);


#endif