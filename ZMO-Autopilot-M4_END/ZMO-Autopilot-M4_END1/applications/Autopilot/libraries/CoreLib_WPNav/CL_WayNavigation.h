/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-11     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_WAYNAVIGATION_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_WAYNAVIGATION_H_

#include "loco_config.h"
#include "arming.h"
#include "CL_Vector.h"

// waypoint controller internal variables
struct wpnav_internal_variables {
    uint32_t    last_update_ms;            // time of last update_wpnav call
    float       wp_desired_speed_xy_cms;   // desired wp speed in cm/sec
    Vector3f    origin;                    // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    destination;                // target destination in cm from ekf origin
}_wp;

// terrain following variables
bool        _terrain_alt;   // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
bool        _rangefinder_available; // true if rangefinder is enabled (user switch can turn this true/false)
uint8_t     _rangefinder_use;       // parameter that specifies if the range finder should be used for terrain following commands
bool        _rangefinder_healthy;   // true if rangefinder distance is healthy (i.e. between min and maximum)
float       _rangefinder_alt_cm;    // latest distance from the rangefinder

//flags structure
struct wpnav_flags {
    uint8_t reached_destination     : 1;    // true if we have reached the destination
    uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
    uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
}wp_flags;

bool set_waypoint_destination(const Vector3f* destination, bool terrain_alt);
bool is_reached_waypoint_destination(void);  //是否到达航点
bool is_WpNav_active(void);
bool update_waypoint_navigation(void);
bool advance_waypoint_target_along_track(float dt);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_WAYNAVIGATION_H_ */
