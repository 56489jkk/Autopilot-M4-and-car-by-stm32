/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 */
#pragma once

#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_IDX 2

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 1
#endif

#include <MAVLINK/mavlink_v2/version.h>
#include <MAVLINK/mavlink_v2/common.h>


typedef enum control_mode_t {
    STABILIZE =     0,  // manual angle with manual depth/throttle
    ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19,  // Pass-through input with no stabilization
    MOTOR_DETECT = 20   // Automatically detect motors orientation
} CONTROL_MODE;

typedef enum MAV_AUTOPILOT
{
   MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
   MAV_AUTOPILOT_RESERVED=1, /* Reserved for future use. | */
   MAV_AUTOPILOT_SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
   MAV_AUTOPILOT_ARDUPILOTMEGA=3, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
   MAV_AUTOPILOT_OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
   MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
   MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
   MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
   MAV_AUTOPILOT_INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
   MAV_AUTOPILOT_PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
   MAV_AUTOPILOT_UDB=10, /* UAV Dev Board | */
   MAV_AUTOPILOT_FP=11, /* FlexiPilot | */
   MAV_AUTOPILOT_PX4=12, /* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
   MAV_AUTOPILOT_SMACCMPILOT=13, /* SMACCMPilot - http://smaccmpilot.org | */
   MAV_AUTOPILOT_AUTOQUAD=14, /* AutoQuad -- http://autoquad.org | */
   MAV_AUTOPILOT_ARMAZILA=15, /* Armazila -- http://armazila.com | */
   MAV_AUTOPILOT_AEROB=16, /* Aerob -- http://aerob.ru | */
   MAV_AUTOPILOT_ASLUAV=17, /* ASLUAV autopilot -- http://www.asl.ethz.ch | */
   MAV_AUTOPILOT_ENUM_END=18, /*  | */
} MAV_AUTOPILOT;
typedef enum MAV_TYPE
{
    MAV_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
    MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
    MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
    MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
    MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
    MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
    MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
    MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
    MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
    MAV_TYPE_ROCKET=9, /* Rocket | */
    MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
    MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
    MAV_TYPE_SUBMARINE=12, /* Submarine | */
    MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
    MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
    MAV_TYPE_TRICOPTER=15, /* Octorotor | */
    MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
    MAV_TYPE_ENUM_END=17, /*  | */
} MAV_TYPE;
#endif // MAVLINK_H
