#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "test_thread/test_thread.h"


int present_dxl1_posi = 0;
int present_dxl2_posi = 0;

uint8_t param_goal_position[4];

static struct End_point target_goal;     //target goal
static struct End_point traj_goal;       //trajectory goal
static struct End_point present_posi;    //present end point position

struct End_point presentXY_fromFK;


//Declare object
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
 // Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


bool is_run = true;
int Control_Cycle = 10; //ms

//time variable
double t = 0;
double dt = 10; //ms
double T = 3000; //ms


void process(void){

  static struct Joint J_goal;              //joint goal from IK
  present_posi = presentXY_fromFK;

  target_goal.x = 0.0;
  target_goal.y = 0.15;

  if(t<=T){
    traj_goal.x = present_posi.x + (target_goal.x - present_posi.x) * 0.5*(1-cos(PI*(t/T)));
    traj_goal.y = present_posi.y + (target_goal.y - present_posi.y) * 0.5*(1-cos(PI*(t/T)));

    J_goal = Compute_IK(traj_goal);

    t+=dt;
  }
  else{
    //t = 0;
    //target_goal = present_posi;

    //do nothing
  }


   set_dxl_goal(radian_to_tick1(J_goal.TH1),radian_to_tick2(J_goal.TH2));
   dxl_go();
   groupSyncWrite.clearParam();


   ROS_INFO("goal1 : %lf, goal2 : %lf",traj_goal.x,traj_goal.y);
   //ROS_INFO("j1 : %d,j2 : %d",radian_to_tick1(J_goal.TH1),radian_to_tick2(J_goal.TH2));
   ROS_INFO("j1 : %lf,j2 : %lf",J_goal.TH1, J_goal.TH2);
}


void *p_function(void * data)
{

  dxl_initailize();

  read_dxl_postion();
  presentXY_fromFK = getPresentXY();

  static struct timespec next_time;
  //static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC,&next_time);


  while(is_run)
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next_time,NULL);

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thread_node");
  ros::NodeHandle nh;

  pthread_t pthread;
  int thr_id;
  int status;
  char p1[] = "thread_1";


  sleep(1);

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
  if(thr_id < 0)
  {
    ROS_ERROR("pthread create error");
    exit(EXIT_FAILURE);
  }

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  is_run = false;
  return 0;

}



void dxl_initailize(void){  //open port, set baud, torque on dxl 1,2
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  dxl_add_param();
}

void set_dxl_goal(int dxl_1_posi, int dxl_2_posi){
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);


}

void dxl_go(void){
  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
}

struct Joint Compute_IK(struct End_point EP){

  //IK solution
  double x = EP.x;
  double y = EP.y;
  double alpha = atan2(y,x);
  double L = sqrt(pow(x,2)+pow(y,2));
  double beta = acos((pow(L1,2)+pow(L2,2)-pow(L,2))/(2*L1*L2));
  double gamma = atan2(x,y);
  double delta = acos((pow(L1,2)+pow(L,2)-pow(L2,2))/(2*L1*L));

  double th2 = PI - beta;
  double th1 = (PI)/2 - gamma - delta;


  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;

  return J;

}

struct End_point Compute_FK(struct Joint J){

  //FK solution
  double th1 = J.TH1;
  double th2 = J.TH2;
  double x = L1*cos(th1) + L2*cos(th1 + th2);
  double y = L1*sin(th1) + L2*sin(th1 + th2);

  struct End_point E;
  E.x = x;
  E.y = y;
  return E;
}

int radian_to_tick1(double radian){
  /*
   * Motor 1 is assembled in the state of 2048.
   * 90 degrees becomes 2048,
   * and 0 degrees becomes 1024.
   * -90 degrees becomes 0.
   * Therefore, 1024 must be added to the converted tick from radian.
   */

  int tick = radian*(2048/PI);
  tick = (tick + 1024);
  return tick;
}

int radian_to_tick2(double radian){
  /*
   * Motor 2 is assembled in the 0-tick state.
   * Therefore, the converted tick value from radians is used without change.
   */
  int tick = radian*(2048/PI);

  return tick;
}

double tick_to_radian_1(int tick){

  /*
   * Motor 1's 0 degree is 1024 ticks.
   * So, after subtracting 1024 from ticks,
   * convert ticks to radians.
   */

   double radian = (PI/(double)2048)*(tick-1024);
   return radian;
}

double tick_to_radian_2(int tick){

  /*
   * 0 degree of motor 2 is 0 tick.
   * So we convert ticks to radians without change.
   */
  double radian = (PI/(double)2048)*tick;
  return radian;

}

struct End_point getPresentXY(void){
  double th1 = tick_to_radian_1(present_dxl1_posi);
  double th2 = tick_to_radian_2(present_dxl2_posi);
  struct Joint j;
  j.TH1 = th1;
  j.TH2 = th2;
  struct End_point E;
  E = Compute_FK(j);

  return E;
}

void read_dxl_postion(void){
  // Syncread present position
  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  usleep(1000);

  // Get Dynamixel#1 present position value
  present_dxl1_posi = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  // Get Dynamixel#2 present position value
  present_dxl2_posi = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  ROS_WARN("present_tick : %d, %d",present_dxl1_posi,present_dxl2_posi);
  ROS_WARN("present_rad : %lf, %lf",tick_to_radian_1(present_dxl1_posi),tick_to_radian_2(present_dxl2_posi));

}

void dxl_add_param(void){

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
  }

  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
  }


}
