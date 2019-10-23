/*
 * gradient_bug.c
 *
 *  Created on: Aug 9, 2018
 *      Author: knmcguire
 */


#include <string.h>
#include <errno.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "commander.h"
#include "sensors.h"
#include "stabilizer_types.h"

#include "estimator_kalman.h"
#include "stabilizer.h"

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"
#include "SGBA.h"
#include "usec_time.h"


#include "range.h"
#include "radiolink.h"
#include "median_filter.h"
#include "configblock.h"


#define STATE_MACHINE_COMMANDER_PRI 3

static bool keep_flying = false;


float height;

static bool taken_off = false;
static float nominal_height = 0.3;

//1= wall_following,
#define METHOD 2


void p2pcallbackHandler(P2PPacket *p);
static uint8_t rssi_inter;
static uint8_t rssi_inter_filtered;
static uint8_t rssi_inter_closest;

float rssi_angle_inter_ext;
float rssi_angle_inter_closest;
uint8_t rssi_beacon;
uint8_t rssi_beacon_filtered;

uint8_t id_inter_ext;
setpoint_t setpoint_BG;
float vel_x_cmd, vel_y_cmd, vel_w_cmd;
float heading_rad;
float right_range;
float front_range;
float left_range;
float up_range;
float back_range;
float rssi_angle;
int state;
int state_wf;
float up_range_filtered;
uint8_t send_to_number = 0;
int varid;
bool manual_startup = false;
bool on_the_ground = true;
uint32_t time_stamp_manual_startup_command = 0;
bool correctly_initialized;
static uint8_t rssi_array_other_drones[9] = {150, 150, 150, 150, 150, 150, 150, 150, 150};
static uint64_t time_array_other_drones[9] = {0};
static float rssi_angle_array_other_drones[9] = {500.0f};
uint8_t id_inter_closest=100;

#define MANUAL_STARTUP_TIMEOUT  M2T(3000)



static void take_off(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = - velocity;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}


static void hover(setpoint_t *sp, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void vel_command(setpoint_t *sp, float vel_x, float vel_y, float yaw_rate, float height)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeAbs;
  sp->velocity.x = vel_x;
  sp->velocity.y = vel_y;
  sp->position.z = height;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = yaw_rate;
  sp->velocity_body = true;

}

static void shut_off_engines(setpoint_t *sp)
{
  sp->mode.x = modeDisable;
  sp->mode.y = modeDisable;
  sp->mode.z = modeDisable;
  sp->mode.yaw = modeDisable;

}

static int32_t find_minimum(uint8_t a[], int32_t n)
{
  int32_t c, min, index;

  min = a[0];
  index = 0;

  for (c = 1; c < n; c++) {
    if (a[c] < min) {
      index = c;
      min = a[c];
    }
  }

  return index;
}

/*static double wraptopi(double number)
{

  if(number>(double)M_PI)
    return (number-(double)(2*M_PI));
  else if(number< (double)(-1*M_PI))
    return (number+(double)(2*M_PI));
  else
    return (number);

}*/
void appMain(void *param)
{
  struct MedianFilterFloat medFilt;
  init_median_filter_f(&medFilt, 5);
  struct MedianFilterFloat medFilt_2;
  init_median_filter_f(&medFilt_2, 5);
  struct MedianFilterFloat medFilt_3;
  init_median_filter_f(&medFilt_3, 5);
  p2pRegisterCB(p2pcallbackHandler);
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
  P2PPacket p_reply;
  p_reply.port=0x00;
  p_reply.data[0]=my_id;
  memcpy(&p_reply.data[1], &rssi_angle, sizeof(float));
  p_reply.size=5;

  uint64_t radioSendBroadcastTime=0;

  systemWaitStart();
  vTaskDelay(M2T(3000));
  while (1) {
	// some delay before the whole thing starts
    vTaskDelay(10);

    // For every 1 second, reset the RSSI value to high if it hasn't been received for a while
    for (uint8_t it = 0; it < 9; it++) if (usecTimestamp() >= time_array_other_drones[it] + 1000*1000) {
        time_array_other_drones[it] = usecTimestamp() + 1000*1000+1;
        rssi_array_other_drones[it] = 150;
        rssi_angle_array_other_drones[it] = 500.0f;
    }

    // get RSSI, id and angle of closests crazyflie.
    id_inter_closest = (uint8_t)find_minimum(rssi_array_other_drones, 9);
    rssi_inter_closest = rssi_array_other_drones[id_inter_closest];
    rssi_angle_inter_closest = rssi_angle_array_other_drones[id_inter_closest];


    // filter rssi
    static int pos_avg = 0;
    static long sum = 0;
    static int arrNumbers[76] = {35};
    static int len = sizeof(arrNumbers) / sizeof(int);
    rssi_beacon_filtered = (uint8_t)movingAvg(arrNumbers, &sum, pos_avg, len, (int)rssi_beacon);


    /*static int arrNumbers_inter[10] = {35};
    static int len_inter = 10;//sizeof(arrNumbers_inter) / sizeof(int);
    static int pos_avg_inter = 0;
    static long sum_inter = 0;
    rssi_inter_filtered = (uint8_t)movingAvg(arrNumbers_inter, &sum_inter, pos_avg_inter, len_inter, (int)rssi_inter_closest);*/
    rssi_inter_filtered =  (uint8_t)update_median_filter_f(&medFilt_2, (float)rssi_inter_closest);

    //checking init of multiranger and flowdeck
    varid = paramGetVarId("deck", "bcMultiranger");
    uint8_t multiranger_isinit=paramGetInt(varid);
    varid = paramGetVarId("deck", "bcFlow2");
    uint8_t flowdeck_isinit=paramGetUint(varid);

    // get current height and heading
    varid = logGetVarId("kalman", "stateZ");
    height = logGetFloat(varid);
    varid = logGetVarId("stabilizer", "yaw");
    float heading_deg = logGetFloat(varid);
    heading_rad = heading_deg * (float)M_PI / 180.0f;

    /* Get RSSI of beacon
    varid = logGetVarId("radio", "rssi");
    rssi_beacon = logGetFloat(varid);*/


    // Select which laser range sensor readings to use
    if (multiranger_isinit) {
      front_range = (float)rangeGet(rangeFront) / 1000.0f;
      right_range = (float)rangeGet(rangeRight) / 1000.0f;
      left_range = (float)rangeGet(rangeLeft) / 1000.0f;
      back_range = (float)rangeGet(rangeBack) / 1000.0f;
      up_range = (float)rangeGet(rangeUp) / 1000.0f;
    }


    // Get position estimate of kalman filter
    point_t pos;
    estimatorKalmanGetEstimatedPos(&pos);

    // Initialize setpoint
    memset(&setpoint_BG, 0, sizeof(setpoint_BG));

    // Filtere uprange, since it sometimes gives a low spike that
    up_range_filtered = update_median_filter_f(&medFilt, up_range);
    if (up_range_filtered < 0.05f) {
      up_range_filtered = up_range;
    }
    //up_range_filtered = 1.0f;
    //***************** Manual Startup procedure*************//

    //TODO: shut off engines when crazyflie is on it's back.

    /*    // indicate if top range is hit while it is not flying yet, then start counting
        if (keep_flying == false && manual_startup==false && up_range <0.2f && on_the_ground == true)
        {
          manual_startup = true;
          time_stamp_manual_startup_command = xTaskGetTickCount();
        }

        // While still on the ground, but indicated that manual startup is requested, keep checking the time
        if (keep_flying == false && manual_startup == true)
        {
            uint32_t currentTime = xTaskGetTickCount();
            // If 3 seconds has passed, start flying.
            if ((currentTime -time_stamp_manual_startup_command) > MANUAL_STARTUP_TIMEOUT)
            {
              keep_flying = true;
              manual_startup = false;
            }
        }*/

    if (flowdeck_isinit && multiranger_isinit ) {
      correctly_initialized = true;
    }
    // Don't fly if multiranger/updownlaser is not connected or the uprange is activated
    //TODO: add flowdeck init here

    if (keep_flying == true && (!correctly_initialized || up_range < 0.2f )) {
      keep_flying = 0;
    }

    state = 0;


    // Main flying code
    if (keep_flying) {
      if (taken_off) {
        /*
         * If the flight is given a OK
         *  and the crazyflie has taken off
         *   then perform state machine
         */
    	  vel_w_cmd = 0;
        hover(&setpoint_BG, nominal_height);

#if METHOD == 1 //WALL_FOLLOWING
        // wall following state machine
        state = wall_follower(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, right_range, heading_rad, 1);
#endif
#if METHOD ==2 //WALL_FOLLOWER_AND_AVOID
        if (id_inter_closest > my_id) {
            rssi_inter_filtered = 140;
        }

        state = wall_follower_and_avoid_controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range, right_range,
                heading_rad, rssi_inter_filtered);
#endif
#if METHOD==3 // SwWARM GRADIENT BUG ALGORITHM



        bool priority = false;
        if (id_inter_closest > my_id) {
          priority = true;
        } else {
          priority = false;

        }
        //TODO make outbound depended on battery.
        bool outbound = true;
        state = SGBA_controller(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, &rssi_angle, &state_wf, front_range,
                                             left_range, right_range, back_range, heading_rad,
                                             (float)pos.x, (float)pos.y, rssi_beacon_filtered, rssi_inter_filtered, rssi_angle_inter_closest, priority, outbound);

        memcpy(&p_reply->data[1],&rssi_angle sizeof(float));



#endif

        // convert yaw rate commands to degrees
        float vel_w_cmd_convert = vel_w_cmd * 180.0f / (float)M_PI;

        // Convert relative commands to world commands (not necessary anymore)
        /*float psi = heading_rad;
        float vel_x_cmd_convert =  cosf(-psi) * vel_x_cmd + sinf(-psi) * vel_y_cmd;
        float vel_y_cmd_convert = -sinf(-psi) * vel_x_cmd + cosf(-psi) * vel_y_cmd;*/
        //float vel_y_cmd_convert = -1 * vel_y_cmd;
        vel_command(&setpoint_BG, vel_x_cmd, vel_y_cmd, vel_w_cmd_convert, nominal_height);
        on_the_ground = false;
      } else {
        /*
         * If the flight is given a OK
         *  but the crazyflie  has not taken off
         *   then take off
         */
          vTaskDelay(M2T(1000*my_id));

        take_off(&setpoint_BG, nominal_height);
        if (height > nominal_height) {
          taken_off = true;

#if METHOD==1
          wall_follower_init(0.4, 0.5);
#endif
#if METHOD==2
          if (my_id==1)
          init_wall_follower_and_avoid_controller(0.4, 0.5, -1);
          else
          init_wall_follower_and_avoid_controller(0.4, 0.5, 1);

#endif
#if METHOD==3
          if (my_id == 4 || my_id == 8) {
              init_SGBA_controller(0.4, 0.5, -0.8);
          } else if (my_id == 2 || my_id == 6) {
              init_SGBA_controller(0.4, 0.5, 0.8);
          } else if (my_id == 3 || my_id == 7) {
              init_SGBA_controller(0.4, 0.5, -2.4);
          } else if (my_id == 5 || my_id == 9) {
              init_SGBA_controller(0.4, 0.5, 2.4);
          } else {
              init_SGBA_controller(0.4, 0.5, 0.8);
          }


#endif

        }
        on_the_ground = false;

      }
    } else {
      if (taken_off) {
        /*
         * If the flight is given a not OK
         *  but the crazyflie  has already taken off
         *   then land
         */
        land(&setpoint_BG, 0.2f);
        if (height < 0.1f) {
          shut_off_engines(&setpoint_BG);
          taken_off = false;
        }
        on_the_ground = false;

      } else {
        /*
         * If the flight is given a not OK
         *  and crazyflie has landed
         *   then keep engines off
         */
        shut_off_engines(&setpoint_BG);
        on_the_ground = true;
      }
    }

    if (usecTimestamp() >= radioSendBroadcastTime + 1000*500) {
        radiolinkSendP2PPacketBroadcast(&p_reply);
        radioSendBroadcastTime = usecTimestamp();
    }
    commanderSetSetpoint(&setpoint_BG, STATE_MACHINE_COMMANDER_PRI);

  }
}

void p2pcallbackHandler(P2PPacket *p)
{
    id_inter_ext = p->data[0];
    rssi_inter = p->rssi;

    if (id_inter_ext == 0x63)
    {
        rssi_beacon =rssi_inter;
        keep_flying =  p->data[1];
    }else
    {
        memcpy(&rssi_angle_inter_ext, &p->data[1], sizeof(float));

        rssi_array_other_drones[id_inter_ext] = rssi_inter;
        time_array_other_drones[id_inter_ext] = usecTimestamp();
        rssi_angle_array_other_drones[id_inter_ext] = rssi_angle_inter_ext;


    }



}

PARAM_GROUP_START(statemach)
PARAM_ADD(PARAM_UINT8, keep_flying, &keep_flying)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, corinit, &correctly_initialized)
PARAM_GROUP_STOP(statemach)

LOG_GROUP_START(statemach)
LOG_ADD(LOG_UINT8, state, &state)
LOG_ADD(LOG_UINT8, rssi_inter, &rssi_inter_closest)
LOG_GROUP_STOP(statemach)
