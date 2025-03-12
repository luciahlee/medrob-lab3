/**
 * University of Michigan - Robotics Department
 * ROB 599-006 MedRob, WN2025
 * Profs. Greg Formosa & Mark Draelos, GSI Andy Qin
 * Lab-3: 1-DOF Medical Robot Teleoperation
 */

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include <mbot/encoder/encoder.h>
#include <math.h>

// Notation: Leader = user/operator , Follower = teleoperated robot/agent

// ---------Your code here---------
// TO-DO: Tune your controller gains and haptics parameters below. We suggest starting only with the follower

// basic PD controller parameters
#define Kp_lead .0025       // proportional gain, leader [PWM% / mm error]
#define Kd_lead 0       // derivative gain, leader [PWM% / mm/s error]
#define Kp_follow 0.01     // proportional gain, follower [PWM% / mm error]
#define Kd_follow 0.0001    // derivative gain, follower [PWM% / mm/s error]
#define Kpsv_lead 0    // passivity/damping constant, leader
#define Kpsv_follow 0   // passivity/damping constant, follower

// haptics deadband
#define DEADBAND 0      // deadband, below which to ignore haptic response (leader command) [mm error]

// haptic saturation effects/pulse
#define THRESHOLD 0       // lead motor command threshold to trigger saturation effects [% of max command]

// latency effects
#define LATENCY_FLAG 0      // 0 = no latency effects, 1 = with latency effects
#define DELAY_MS 500       // chosen latency [ms]
// --------End of your code---------


// ----------DO NOT REVISE----------
#define MOTOR_ID_LEADER 0
#define MOTOR_ID_FOLLOWER 1
#define MOTOR_LEAD_RES ((64*6.25) / (40*2))
#define MOTOR_FOLLOW_RES ((64*70) / (40*2))
#define MAX_COMMAND_LEAD 0.5
#define MAX_COMMAND_FOLLOW 1.0
#define THRESHOLD_COMMAND (THRESHOLD*MAX_COMMAND_LEAD)
#define PI 3.141592653589793
#define TIME_STEP 5
#define BUFFER_SIZE (DELAY_MS / TIME_STEP)

typedef struct {
    float lead_pos[BUFFER_SIZE];
    float lead_vel[BUFFER_SIZE];
    float follow_pos[BUFFER_SIZE];
    float follow_vel[BUFFER_SIZE];
    int head;
} StateBuffer;

void init_buffer(StateBuffer *buffer) {
    buffer->head = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer->lead_pos[i] = 0.0;
        buffer->lead_vel[i] = 0.0;
        buffer->follow_pos[i] = 0.0;
        buffer->follow_vel[i] = 0.0;
    }
}

void add_state(StateBuffer *buffer, float lead_pos, float lead_vel, float follow_pos, float follow_vel) {
    buffer->head = (buffer->head + 1) % BUFFER_SIZE;  // Increment head circularly
    buffer->lead_pos[buffer->head] = lead_pos;
    buffer->lead_vel[buffer->head] = lead_vel;
    buffer->follow_pos[buffer->head] = follow_pos;
    buffer->follow_vel[buffer->head] = follow_vel;
}

float clipCommand(float command, float MAX_COMMAND) {
    if (command > MAX_COMMAND) command = MAX_COMMAND;
    if (command < -MAX_COMMAND) command = -MAX_COMMAND;
    return command;
}

// main control loop, you will mostly add/edit code in here!
void teleoperation(int reverse, int delay) {
    int d_lead, d_follow, t_lead, t_follow = 0;
    StateBuffer buffer;
    init_buffer(&buffer);
    float t_sec = 0.0;

    while (1) {
        // read encoder data from Pico
        t_lead = mbot_encoder_read_count(MOTOR_ID_LEADER);      // encoder counts/ticks (position)
        t_follow = mbot_encoder_read_count(MOTOR_ID_FOLLOWER);
        d_lead = mbot_encoder_read_delta(MOTOR_ID_LEADER);      // encoder count delta (speed)
        d_follow = mbot_encoder_read_delta(MOTOR_ID_FOLLOWER);

        // convert encoder values to linear translations
        float motor_lead_pos = t_lead / MOTOR_LEAD_RES;         // device position [mm]
        float motor_follow_pos = t_follow / MOTOR_FOLLOW_RES;
        float motor_lead_vel = d_lead / MOTOR_LEAD_RES;         // device velocity [mm/s]
        float motor_follow_vel = d_follow / MOTOR_FOLLOW_RES;
        
        // add states to buffer for latency mode
        add_state(&buffer, motor_lead_pos, motor_lead_vel, motor_follow_pos, motor_follow_vel);
        // retrieve delayed states 
        int delayed_index = (buffer.head + 1) % BUFFER_SIZE;
        float delayed_lead_pos = buffer.lead_pos[delayed_index];
        float delayed_lead_vel = buffer.lead_vel[delayed_index];
        float delayed_follow_pos = buffer.follow_pos[delayed_index];
        float delayed_follow_vel = buffer.follow_vel[delayed_index];

        // compute the errors
        float position_error = motor_follow_pos - motor_lead_pos;
        float velocity_error = motor_follow_vel - motor_lead_vel;
        // reverse mode
        if (reverse) {
            position_error = -position_error;
            velocity_error = -velocity_error;
        }

        // --------------Your code here--------------
        // TO-DO: Design control signals for your follower (tracking) & leader (haptics)
        // You are welcome to edit or add any code you would like here, just be careful of your devices' positions!

        // basic PD motor control (no need to edit, unless adding terms/changing methods)
        float u_lead_p = Kp_lead * position_error;
        float u_lead_d = Kd_lead * velocity_error;
        float u_lead_psv = Kpsv_lead * motor_lead_vel;
        float u_follow_p = Kp_follow * (-position_error);
        float u_follow_d = Kd_follow * (-velocity_error);
        float u_follow_psv = Kpsv_follow * motor_follow_vel;
        // basic PD motor control for latency mode (no need to edit, unless adding terms/changing methods)
        if (delay) {
            u_lead_p = Kp_lead * (delayed_follow_pos - motor_lead_pos);
            u_lead_d = Kd_lead * (delayed_follow_vel - motor_lead_vel);
            u_follow_p = Kp_follow * (delayed_lead_pos - motor_follow_pos);
            u_follow_d = Kd_follow * (delayed_lead_vel - motor_follow_vel);
        }

        float u_lead = u_lead_p + u_lead_d - u_lead_psv;
        float u_follow = u_follow_p + u_follow_d - u_follow_psv;

        // TO-DO: introduce a deadband for the leader control (i.e., no haptics/resistance under a threshold)


        // TO-DO: add saturation effects to haptics (e.g., pulsing, early saturation, etc.)


        // ----------End of your code-----------
        
        // serial output
        printf("| %7d| %7d| %.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|\n", motor_lead_pos, motor_follow_pos, position_error, clipCommand(u_lead,MAX_COMMAND_LEAD), clipCommand(u_follow,MAX_COMMAND_FOLLOW), u_lead_p, u_lead_d, u_follow_p, u_follow_d, t_sec);

        // send motor commands
        mbot_motor_set_duty(MOTOR_ID_LEADER, clipCommand(u_lead,MAX_COMMAND_LEAD));
        mbot_motor_set_duty(MOTOR_ID_FOLLOWER, clipCommand(u_follow,MAX_COMMAND_FOLLOW));

        sleep_ms(TIME_STEP);
    }
}

int main() {
    stdio_init_all();
    mbot_encoder_init();
    sleep_ms(2000);

    mbot_motor_init(MOTOR_ID_LEADER);
    mbot_motor_init(MOTOR_ID_FOLLOWER);
    sleep_ms(2000);
    printf("\033[2J\r");

    // TO-DO (optional): Feel free to add a start-up routine here, for calibration or zeroing

    int reverse = 0; // 0 = normal direction, 1 = reverse
    teleoperation(reverse, LATENCY_FLAG);   // start main loop for control
}
