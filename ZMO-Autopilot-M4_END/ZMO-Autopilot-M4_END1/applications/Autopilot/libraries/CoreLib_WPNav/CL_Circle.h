/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-02     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_CIRCLE_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_CIRCLE_H_

#include "CL_Vector.h"

// loiter maximum velocities and accelerations
#define AC_CIRCLE_RADIUS_DEFAULT    1000.0f     // radius of the circle in cm that the vehicle will fly
#define AC_CIRCLE_RATE_DEFAULT      20.0f       // turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
#define AC_CIRCLE_ANGULAR_ACCEL_MIN 2.0f        // angular acceleration should never be less than 2deg/sec
#define AC_CIRCLE_RADIUS_MAX        200000.0f   // maximum allowed circle radius of 2km

// parameters
float    _radius_parm;   // radius of circle in cm loaded from params
float    _rate;          // rotation speed in deg/sec
int16_t  _options;       // stick control enable/disable

// internal variables
Vector3f    _center;        // center of circle in cm from home
float       _radius;        // radius of circle in cm
float       _yaw;           // yaw heading (normally towards circle center)
float       _angle;         // current angular position around circle in radians (0=directly north of the center of the circle)
float       _angle_total;   // total angle traveled in radians
float       _angular_vel;   // angular velocity in radians/sec
float       _angular_vel_max;   // maximum velocity in radians/sec
float       _angular_accel; // angular acceleration in radians/sec/sec
uint32_t    _last_update_ms;    // system time of last update
float       _last_radius_param; // last value of radius param, used to update radius on param change

// terrain following variables
bool        _terrain_alt;           // true if _center.z is alt-above-terrain, false if alt-above-ekf-origin
bool        _rangefinder_available; // true if range finder could be used
bool        _rangefinder_healthy;   // true if range finder is healthy
float       _rangefinder_alt_cm;    // latest rangefinder altitude





#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_WPNAV_CL_CIRCLE_H_ */
