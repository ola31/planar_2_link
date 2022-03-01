#ifndef TEST_THREAD_H
#define TEST_THREAD_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "dynamixel_sdk/dynamixel_sdk.h"



//Hardware Info
#define L1  0.12
#define L2  0.1

//Dynamixel Control

#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0

#define DXL1_ID                          3
#define DXL2_ID                          11
#define BAUDRATE                        2000000
#define ADDR_TORQUE_ENABLE              64
#define TORQUE_ENABLE                   1
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132
// Data Byte Length
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_PRESENT_POSITION         4



/*********************
 * Structures
 * *******************/
struct End_point
{
  double x = 0.0;
  double y = 0.0;
};

struct Joint
{
  double TH1 = 0.0;
  double TH2 = 0.0;
};


/*************************************************
 * Functions
 * ***********************************************/
void *p_function(void * data);
void process(void);
void dxl_add_param(void);
void dxls_torque_on(void);
void dxl_initailize(void);
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);
void dxl_go(void);
struct Joint Compute_IK(struct End_point EP);
struct End_point Compute_FK(struct Joint J);
int radian_to_tick1(double radian);
int radian_to_tick2(double radian);
double tick_to_radian_1(int tick);
double tick_to_radian_2(int tick);
struct End_point getPresentXY(void);
void read_dxl_postion(void);
void clear_param(void);




#endif // TEST_THREAD_H
