/*
 * wallfollowing_with_avoid.c
 *
 *  Created on: Nov 12, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"

#include <math.h>
#include "usec_time.h"

float state_start_time;

//static variables only used for initialization
static bool first_run = true;
static float ref_distance_from_wall = 0.5;
static float max_speed = 0.5;
static float local_direction = 1;


static int transition(int new_state)
{
    float t =  usecTimestamp() / 1e6;
    state_start_time = t;
    return new_state;
}

// statemachine functions
void init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall, float max_speed_ref,
        float starting_local_direction)
{
    ref_distance_from_wall = new_ref_distance_from_wall;
    max_speed = max_speed_ref;
    local_direction = starting_local_direction;
    first_run = true;
}


int wall_follower_and_avoid_controller(float *vel_x, float *vel_y, float *vel_w, float front_range, float left_range,
        float right_range,  float current_heading, uint8_t rssi_other_drone)
{

    // Initalize static variables
    static int state = 1;
    static int rssi_collision_threshold = 43;

    // if it is reinitialized
    if (first_run) {
        state = 1;
        float t =  usecTimestamp() / 1e6;
        state_start_time = t;
        first_run = false;
    }


    /***********************************************************
     * State definitions
     ***********************************************************/
    // 1 = forward
    // 2 = wall_following
    // 3 = move_out_of_way


    /***********************************************************
     * Handle state transitions
     ***********************************************************/

    if (state == 1) {     //FORWARD
        // if front range is close, start wallfollowing
        if (front_range < ref_distance_from_wall + 0.2f) {
            wall_follower_init(ref_distance_from_wall, 0.5, 3);
            state = transition(2); //wall_following
        }
    } else if (state == 2) {      //WALL_FOLLOWING

        if (rssi_other_drone < rssi_collision_threshold) {
            state = transition(3);
        }
    } else if (state == 3) { //MOVE_OUT_OF_WAY
        if (rssi_other_drone > rssi_collision_threshold) {
            state = transition(1);
        }

    }
    /***********************************************************
     * Handle state actions
     ***********************************************************/

    float temp_vel_x = 0;
    float temp_vel_y = 0;
    float temp_vel_w = 0;

    if (state == 1) {        //FORWARD
        // forward max speed
        temp_vel_x = 0.5;

    } else  if (state == 2) {       //WALL_FOLLOWING
        //Get the values from the wallfollowing
        if (local_direction == 1) {
            wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, local_direction);
        } else if (local_direction == -1) {
            wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, local_direction);
        }
    } else if (state == 3) {       //MOVE_OUT_OF_WAY

        float save_distance = 0.7f;
        if (left_range < save_distance) {
            temp_vel_y = temp_vel_y - 0.5f;
        }
        if (right_range < save_distance) {
            temp_vel_y = temp_vel_y + 0.5f;
        }
        if (front_range < save_distance) {
            temp_vel_x = temp_vel_x - 0.5f;
        }
    }

    *vel_x = temp_vel_x;
    *vel_y = temp_vel_y;
    *vel_w = temp_vel_w;

    return state;
}

