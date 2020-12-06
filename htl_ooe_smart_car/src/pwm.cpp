/**
 *
   \file
   \brief      controller for I2C interfaced 16 channel PWM boards with PCA9685 chip
   \author     Bradan Lane Studio <info@bradanlane.com>
   \author     Reused for htl_ooe_smart_car 2020 Franz Parzer <Franz.Parzer@htl-steyr.ac.at>
   \copyright  Copyright (c) 2016, Bradan Lane Studio
   \copyright  Copyright (c) 2020, HTL Steyr

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      - Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      - The name of Bradan Lane, Bradan Lane Studio nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL BRADAN LANE STUDIOS BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Please send comments, questions, or patches to info@bradanlane.com

*/

/**
\mainpage

 Controller for I2C interfaced 16 channel PWM boards with PCA9685 chip<br/>
 Bradan Lane Studio <info@bradanlane.com><br/>
 Copyright (c) 2016, Bradan Lane Studio<br/>
 Franz Parzer <Franz.Parzer@htl-steyr.ac.at><br/>
 Copyright (c) 2020, HTL Steyr<br/>
 Licensed under GPLv3<br/>

--------


\section overview FILE STRUCTURE AND CODE

  The file is broken into sections:
    - private properties (all private properties have a leading underscore)
    - private methods (all private methods have a leading underscore)
    - public topic subscribers:
        - servos_absolute()
        - servos_proportional()
        - servos_drive()
    - public services:
        - set_pwm_frequency()
        - config_servos()
        - config_drive_mode()
        - stop_servos()

  The code is currently authored in C and should be rewritten as proper C++.

  This documentation refers to 'servo' and 'RC servo' but is equally applicable to any PWM or PPM controlled DC motor.

  All published services and topics use a zero-based count syntax. For example, the first servo is '0'.
  The hardware is accessed via sysfs virtual file system.
  The switch from one-based to zero-based is done at the lowest level of this code. All public interactions should assume zero-based values.

  _WARNING_: The code has only been tested with a single board. Once testing has been done with additional configurations, this warning will be removed or amended.


\section pwmservos USING PWM WITH SERVOS

  While the PCA9685 chip and related boards are called "PWM" for pulse width modulation, their use with servos
  is more accurately PPM for pulse position modulation.

  For standard 180 degree servos with a motion arc of ±90 degrees the pulse moves the servo arm to specifc position and holds it.
  For continuous motion servos, the pulse moves the servo at a specific speed.

  The documentation will refer to positon, speed or position/speed.
  These are interchangeable as the two terms are dependent on whether the servo is a fixed rotation servo or a continuous rotation servo and independent of the board itself.

  Analog RC servos are most often designed for 20ms pulses. This is achieved with a frequency of 50Hz.
  This software defaults to 50Hz. Use the set_pwm_frequncy() to change this frequency value.
  It may be necessary or convenient to change the PWM frequency if using DC motors connected to PWM controllers.
  It may also be convenient if using PWM to control LEDs.

  A commonly available board is the Adafruit 16 channel 12-bit PWM board or the similarly named HAT. There are similar boards as well as clones.
  All of these boards use the PCA9685 chip. Not all boards have been tested.

  The PWM boards drive LED and servos using pulse width modulation. The 12 bit interface means values are in the range of 0..4096.
  The pulse is defined as a start value and end value. When driving servos, the start point is typically 0 and the end point is the duration.

\section configuration CONFIGURING SERVOS

  The tolerance of the resistors in RC servos means each servo may have a slightly different center point.

  The servos_absolute() topic controls servos with absolute pulse start and stop values.
  This topic subscriber is not generally useful in robot operations, however it is convenient for testing and for determining configuration values.
  Use this topic to determine the center position for a standard servo or the stop position for a continuous servo.

  Also use this topic to determine the range of each servo -
  either the ±90 postion for a standard servo or
  the max forward and reverse speed for a continuous rotation servo.

  __note:__ The centering value and range of servos is dependent on the pulse frequency.
  If you use set_pwm_frequency() to change the system value, you will need to determine new center and range values.

  The servos_proportional() topic controls servos through their range of motion using values between -1.0 and 1.0.
  Use the config_servos() service to set the center values, range values, and directions of rotation for servos.
  This enables servo motion to be generalized to a standard proportion of ±1.0.
  Use of the config_servos() service is required before proportional control is available.


\section drivemode ROBOT DRIVE MODE

  In addition to absolute and proportional topics for controling servos, this package provides support for the geometry 'Twist' message.
  The servos_drive() topic handles the calculations to convert linear and angular data to servo drive data.

  Drive mode requires details for each servo. Use config_servos() before using drive mode.

  The geometry_msgs::Twist provides linear and angular velocity measured in m/s (meters per second) and r/s (radians per second) respectively.
  To perform the necessary calculations, data about the drive system is required.
  Use config_drive_mode() to provide the RPM of the drive wheels, their radius, and the track distance between left and right. These values determine speed and turn rates.

  Specifiy the desired drive mode with the `mode` property to cofig_drive_mode(). This package supports three drive modes:

    -# __ackerman__ - A single velocity drive using one or more servos with a non-drive servo controlling steering.
    -# __differential__ - Skid steer or track steer using one or more servos for each of the left and right sides.
    The result is full speed forward or reverse, min/max radius turns, and the ability to perform zero-radius turns
    -# __mecanum__ - Independent drive on all four wheel capable of holonomic drive - moving in any combination of forward, reverse, turning, and sideways.
    The servos are assigned positons for left-front, right-front, left-rear, and right-rear wheels. This mode supports
    similar drive characteristics to differential drive with the addition of lateral motion.


  The servos on the PWM board are assigned to their respective drive positons using the config_drive_mode() service.
  The applicable servos are assigned positions as follows:

    positon | ackerman | differential | mecanum
    --------|----------|--------------|--------
    position 0 corresponds to | drive | left-front | left-front
    position 1 corresponds to | | right-front | right-front
    position 2 corresponds to | | left-rear | left-rear
    position 3 corresponds to | | right-rear | right-rear

  The drive topic requires that use of both the config_servos() service and config_drive_mode() service before drive mode will result in expected behavior.

  All non-drive servos may still be operated with the proportion or absolute topics.

  __Note:__ _This controller does not implement encoders for PWM motors. There is no guarantee that the drive velocity will exactly match the Twist message input.
  While this may not be acceptable for a commercial or scientific application, it may not be of convern for education and amatuer competitions.
  Encoders could be added and feedback applied to the PWM values.
  Additionally, when drive control is a product of positional feedback - such as line following and navigation via camera vision - drive encoders are usually not required._

  The stop_servos() service is provided as convenience to stop all servos and place them in a powered off state. This is different from setting each servo to its center value.
  The stop service is useful as a safety operation.

\section testing TESTING

  Basic testing is available from the command line.
  Start the node with `roslaunch htl_ooe_smart_car htl_ooe_smart_car.launch` (or `roscore` and `rosrun htl_ooe_smart_car htl_ooe_smart_car`) and then proceed with
  example commands contained within the documentation for each service and topic subscriber.

 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <pigpiod_if2.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "sysfs.hpp"

// messages used for any service with no parameters
#include <std_srvs/Empty.h>
// messages used for drive movement topic
#include <geometry_msgs/Twist.h>

// messages used for the absolute and proportional movement topics
#include "htl_ooe_smart_car/Servo.h"
#include "htl_ooe_smart_car/ServoArray.h"
// messages used for the servo setup service
#include "htl_ooe_smart_car/ServoConfig.h"
#include "htl_ooe_smart_car/ServoConfigArray.h"
// request/response of the servo setup service
#include "htl_ooe_smart_car/ServosConfig.h"
// request/response of the drive mode service
#include "htl_ooe_smart_car/DriveMode.h"
#include "htl_ooe_smart_car/Position.h"
#include "htl_ooe_smart_car/PositionArray.h"
// request/response of the integer parameter services
#include "htl_ooe_smart_car/IntValue.h"

/// @cond PRIVATE_NO_PUBLIC DOC

typedef struct _servo_config {
    int center;
    int range;
    int direction;
    int mode_pos;
    fs::path sysfs_path;
} servo_config;

typedef struct _drive_mode {
    int mode;
    float rpm;
    float radius;
    float track;
    float scale;
} drive_mode;

enum drive_modes {
    MODE_UNDEFINED      = 0,
    MODE_ACKERMAN       = 1,
    MODE_DIFFERENTIAL   = 2,
    MODE_MECANUM        = 3,
    MODE_INVALID        = 4
};

enum drive_mode_positions {
    POSITION_UNDEFINED          = 0,
    POSITION_LEFTFRONT_PHASE_A  = 1,
    POSITION_LEFTFRONT_PHASE_B  = 2,
    POSITION_RIGHTFRONT_PHASE_A = 3,
    POSITION_RIGHTFRONT_PHASE_B = 4,
    POSITION_LEFTREAR_PHASE_A   = 5,
    POSITION_LEFTREAR_PHASE_B   = 6,
    POSITION_RIGHTREAR_PHASE_A  = 7,
    POSITION_RIGHTREAR_PHASE_B  = 8,
    POSITION_INVALID            = 9
};

#define IIO_DEVICE_NAME     "pca9685-pwm"
#define DRIVE_POWER_ENABLE  23
#define PWM_OUTPUT_N_ENABLE 27

#ifndef _PI
#define _PI          3.14159265358979323846
#endif
#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000
#endif

#define _CONST(s)          ((char*)(s))

#define CHANNELS_PER_BOARD 16
#define MAX_BOARDS         2
#define MAX_SERVOS         (CHANNELS_PER_BOARD*MAX_BOARDS)

servo_config _servo_configs[MAX_SERVOS];
drive_mode _active_drive; // used when converting Twist geometry to PWM values and which servos are for motion

int _pwm_frequency = 50;                    // frequency determines the size of a pulse width; higher numbers make RC servos buzz

/// @endcond PRIVATE_NO_PUBLIC DOC

//* ------------------------------------------------------------------------------------------------------------------------------------
// local private methods
//* ------------------------------------------------------------------------------------------------------------------------------------
static float _abs (float v1) {
    if(v1 < 0)
        return (0 - v1);
    return v1;
}

static float _min (float v1, float v2) {
    if(v1 > v2)
        return v2;
    return v1;
}

static float _max (float v1, float v2) {
    if(v1 < v2)
        return v2;
    return v1;
}

static float _absmin (float v1, float v2) {
    float a1, a2;
    float sign = 1.0;
    //    if(v1 < 0)
    //        sign = -1.0;
    a1 = _abs(v1);
    a2 = _abs(v2);
    if(a1 > a2)
        return (sign * a2);
    return v1;
}

static float _absmax (float v1, float v2) {
    float a1, a2;
    float sign = 1.0;
    //    if(v1 < 0)
    //        sign = -1.0;
    a1 = _abs(v1);
    a2 = _abs(v2);
    if(a1 < a2)
        return (sign * a2);
    return v1;
}

/**
 \private method to smooth a speed value

 we calculate each speed using a cosine 'curve',  this results in the output curve
 being shallow at 'stop', full forward, and full reverse and becoming
 more aggressive in the middle or each direction

 @param speed an int value (±1.0) indicating original speed
 @returns an integer value (±1.0) smoothed for more gentle acceleration
 */
static int _smoothing (float speed) {
    /* if smoothing is desired, then remove the commented code  */
    // speed = (cos(_PI*(((float)1.0 - speed))) + 1) / 2;
    return speed;
}

/**
   \private method to convert meters per second to a proportional value in the range of ±1.0

   @param speed float requested speed in meters per second
   @returns float value (±1.0) for servo speed
 */
static float _convert_mps_to_proportional (float speed) {
    /* we use the drive mouter output rpm and wheel radius to compute the conversion */

    float initial, max_rate;    // the max m/s is ((rpm/60) * (2*PI*radius))

    initial = speed;

    if(_active_drive.rpm <= 0.0) {
        ROS_ERROR("Invalid active drive mode RPM %6.4f :: RPM must be greater than 0", _active_drive.rpm);
        return 0.0;
    }
    if(_active_drive.radius <= 0.0) {
        ROS_ERROR("Invalid active drive mode radius %6.4f :: wheel radius must be greater than 0", _active_drive.radius);
        return 0.0;
    }

    max_rate = (_active_drive.radius * _PI * 2) * (_active_drive.rpm / 60.0);

    speed = speed / max_rate;
    // speed = _absmin (speed, 1.0);

    ROS_DEBUG("%6.4f = convert_mps_to_proportional ( speed(%6.4f) / ((radus(%6.4f) * pi(%6.4f) * 2) * (rpm(%6.4f) / 60.0)) )",
              speed, initial, _active_drive.radius, _PI, _active_drive.rpm);
    return speed;
}

/**
 * \private method to set pulse frequency
 *
 *The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 *@param frequency an int value (1..15000) indicating the pulse frequency where 50 is typical for RC servos
 *Example _set_frequency (68)  // set the pulse frequency to 68Hz
 */
static int _set_pwm_frequency (int freq) {
    _pwm_frequency = freq;   // save to global

    ROS_INFO("Setting PWM frequency to %d Hz", freq);

    for(auto &servo_config : _servo_configs)
        if(!servo_config.sysfs_path.empty())
            sysfs::write_attribute(servo_config.sysfs_path / "period", std::to_string(NSEC_PER_SEC/freq));

    return 0;
}

/**
 * \private method to set a duty_cycle for a PWM channel
 *
 *The pulse defined by duty_cycle will be active on the specified servo channel until any subsequent call changes it.
 *@param servo an struct _servo_config indicating which channel to change
 *@param duty_cycle an int value indicating the pulse width in ns.
 *Example _set_duty_cycle (servo, 35000) // set servo with a pulse of 35000ns
 */
static void _set_duty_cycle (struct _servo_config servo, unsigned int duty_cycle) {
    ROS_DEBUG("%s enter", __func__);

    if(servo.sysfs_path.empty()) {
        ROS_ERROR("This Servo is not available");
        return;
    }

    ROS_DEBUG("%s %s duty_cycle=%d", __func__, servo.sysfs_path.string().c_str(), duty_cycle);
    if(NSEC_PER_SEC/_pwm_frequency < duty_cycle) {
        ROS_ERROR("%s %s duty_cycle(%d) is larger than period(%d)",
                  __func__,
                  servo.sysfs_path.string().c_str(),
                  duty_cycle,
                  NSEC_PER_SEC/_pwm_frequency);
        return;
    }
    sysfs::write_attribute(servo.sysfs_path / "duty_cycle", std::to_string(duty_cycle));
}

/**
 * \private method to set a value for a PWM channel, based on a range of ±1.0
 *
 *The pulse defined by duty_cycle will be active on the specified servo channel until any subsequent call changes it.
 *@param servo an struct _servo_config indicating which channel to change power
 *@param value an int value (±1.0) indicating when the size of the pulse for the channel.
 *Example _set_pwm_interval_proportional (servo, 0.5) // set servo with half pulse width
 */
static void _set_pwm_interval_proportional (struct _servo_config servo, float value) {
    // need a little wiggle room to allow for accuracy of a floating point value
    if((value < -1.0001) || (value > 1.0001)) {
        ROS_ERROR("Invalid proportion value %f :: proportion values must be between -1.0 and 1.0", value);
        return;
    }

    if((servo.center < 0) || (servo.range < 0)) {
        ROS_ERROR_STREAM("Missing servo configuration for servo " << servo.sysfs_path);
        return;
    }

    int pos = (servo.direction * (((float)(servo.range) / 2) * value)) + servo.center;

    ROS_DEBUG("Invalid computed position servo [%s] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
              servo.sysfs_path.string().c_str(), servo.direction, servo.range, value, servo.center, pos);

    _set_duty_cycle (servo, pos);
    ROS_DEBUG("servo [%s] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d",
              servo.sysfs_path.string().c_str(), servo.direction, servo.range, value, servo.center, pos);
}

/**
 * \private method to configure a servo
 *
 *@param servo an int value (0..MAX_SERVOS)
 *@param center an int value gt 1
 *@param range int value gt 1
 *@param direction an int  either -1 or 1
 *Example _config_server (1, 300, 100, -1)   // configure servo number 1 with a center of 300 and range of 100 and reversed direction
 */
static void _config_servo (int servo, int center, int range, int direction) {
    if((servo < 0) || (servo > (MAX_SERVOS))) {
        ROS_ERROR("Invalid servo number %d :: servo numbers must be between 0 and %d", servo, MAX_SERVOS);
        return;
    }

    if((center < 0) || (center > NSEC_PER_SEC/_pwm_frequency))
        ROS_ERROR("Invalid center value %d :: center values must be between 0 and %d", center, NSEC_PER_SEC/_pwm_frequency);

    if((center < 0) || (center > NSEC_PER_SEC/_pwm_frequency))
        ROS_ERROR("Invalid range value %d :: range values must be between 0 and %d", range, NSEC_PER_SEC/_pwm_frequency);

    if(((center - (range/2)) < 0) || (((range/2) + center) > NSEC_PER_SEC/_pwm_frequency))
        ROS_ERROR("Invalid range center combination %d ± %d :: range/2 ± center must be between 0 and %d", center, (range/2), NSEC_PER_SEC/_pwm_frequency);

    _servo_configs[servo].center = center;
    _servo_configs[servo].range = range;
    _servo_configs[servo].direction = direction;
    // _servo_configs[servo].mode_pos = POSITION_UNDEFINED;

    ROS_INFO("Servo #%d configured: center=%d, range=%d, direction=%d", servo, center, range, direction);
}

static int _config_servo_position (int servo, int position) {
    if((servo < 0) || (servo > (MAX_SERVOS))) {
        ROS_ERROR("Invalid servo number %d :: servo numbers must be between 0 and %d", servo, MAX_SERVOS);
        return -1;
    }
    if((position < POSITION_UNDEFINED) || (position >= POSITION_INVALID)) {
        ROS_ERROR("Invalid drive mode position %d :: positions are 0 = non-drive, 1 = left front, 2 = right front, 3 = left rear, and 4 = right rear", position);
        return -1;
    }
    _servo_configs[servo].mode_pos = position;
    ROS_INFO("Servo #%d configured: position=%d", servo, position);
    return 0;
}

static int _config_drive_mode (std::string mode, float rpm, float radius, float track, float scale) {
    int mode_val = MODE_UNDEFINED;

    // assumes the parameter was provided in the proper case
    if      (0 == strcmp (mode.c_str(), _CONST("ackerman")))
        mode_val = MODE_ACKERMAN;
    else if(0 == strcmp (mode.c_str(), _CONST("differential")))
        mode_val = MODE_DIFFERENTIAL;
    else if(0 == strcmp (mode.c_str(), _CONST("mecanum")))
        mode_val = MODE_MECANUM;
    else {
        mode_val = MODE_INVALID;
        ROS_ERROR("Invalid drive mode %s :: drive mode must be one of ackerman, differential, or mecanum", mode.c_str());
        return -1;
    }

    if(rpm <= 0.0) {
        ROS_ERROR("Invalid RPM %6.4f :: the motor's output RPM must be greater than 0.0", rpm);
        return -1;
    }

    if(radius <= 0.0) {
        ROS_ERROR("Invalid radius %6.4f :: the wheel radius must be greater than 0.0 meters", radius);
        return -1;
    }

    if(track <= 0.0) {
        ROS_ERROR("Invalid track %6.4f :: the axel track must be greater than 0.0 meters", track);
        return -1;
    }

    if(scale <= 0.0) {
        ROS_ERROR("Invalid scale %6.4f :: the scalar for Twist messages must be greater than 0.0", scale);
        return -1;
    }

    _active_drive.mode = mode_val;
    _active_drive.rpm = rpm;
    _active_drive.radius = radius;    // the service takes the radius in meters
    _active_drive.track = track;      // the service takes the track in meters
    _active_drive.scale = scale;

    ROS_INFO("Drive mode configured: mode=%s, rpm=%6.4f, radius=%6.4f, track=%6.4f, scale=%6.4f", mode.c_str(), rpm, radius, track, scale);
    return 0;
}

/**
 \private method to initialize private internal data structures at startup
 */
static int _init (void) {
    int i, channel;

    /* initialize all of the global data objects */
    for(auto &servo_config : _servo_configs) {
        servo_config.sysfs_path.clear();
        // these values have not useful meaning
        servo_config.center = -1;
        servo_config.range = -1;
        servo_config.direction = 1;
        servo_config.mode_pos = -1;
    }

    i=0;
    for(fs::path p : sysfs::find_devices(IIO_DEVICE_NAME)) {
        for(fs::path e : sysfs::find_dirs_containing(p, "export")) {
            for(channel=0; channel<CHANNELS_PER_BOARD; channel++) {
                sysfs::write_attribute(e / "export", std::to_string(channel));
                _servo_configs[i + channel].sysfs_path = e / ((std::string)"pwm" + std::to_string(channel));
            }
            sysfs::write_attribute(e / "uevent", "change"); // trigger udev
            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(1.0); // Timeout of 1 second
            int access_possible;
            while(1) {
                access_possible = 0;
                for(channel=0; channel<CHANNELS_PER_BOARD; channel++) {
                    if(access((_servo_configs[i + channel].sysfs_path).string().c_str(), W_OK) == 0)
                        access_possible++;
                }
                if(access_possible == CHANNELS_PER_BOARD)
                    break;
                if(ros::Time::now() - start_time > timeout) {
                    ROS_ERROR("pwm sysfs system not writeable! Did you forget to modify /etc/udev/rules.d/99-com.rules");
                    return 1;
                }
                ros::Duration(0.01).sleep();  // Sleep for 10 miliseconds
            }
        }
        i += CHANNELS_PER_BOARD;
        if(i >= MAX_SERVOS) {
            ROS_ERROR("Found more PCA9685s than configured (%d)", MAX_BOARDS);
            break; // maximum number of servos reached
        }
    }

    for(auto &servo_config : _servo_configs)
        if(!servo_config.sysfs_path.empty())
            sysfs::write_attribute(servo_config.sysfs_path / "enable", "1");

    _active_drive.mode = MODE_UNDEFINED;
    _active_drive.rpm = -1.0;
    _active_drive.radius = -1.0;
    _active_drive.track = -1.0;
    _active_drive.scale = -1.0;

    return 0;
}

/**
 \private method to cleanup
 */
static void _exit (void) {
    int i, channel;

    for(auto &servo_config : _servo_configs)
        if(!servo_config.sysfs_path.empty())
            sysfs::write_attribute(servo_config.sysfs_path / "enable", "0");

    for(auto &servo_config : _servo_configs)
        if(!servo_config.sysfs_path.empty()) {
            size_t last_index = servo_config.sysfs_path.string().find_last_not_of("0123456789");
            sysfs::write_attribute(servo_config.sysfs_path / "../unexport",
                                   servo_config.sysfs_path.string().substr(last_index + 1));
        }

    for(auto &servo_config : _servo_configs) {
        // these values have not useful meaning
        servo_config.center = -1;
        servo_config.range = -1;
        servo_config.direction = 1;
        servo_config.mode_pos = -1;
    }

    _active_drive.mode = MODE_UNDEFINED;
    _active_drive.rpm = -1.0;
    _active_drive.radius = -1.0;
    _active_drive.track = -1.0;
    _active_drive.scale = -1.0;
}

// ------------------------------------------------------------------------------------------------------------------------------------
/**
\defgroup Topics Topics with subscribers provided by this package
@{ */
// ------------------------------------------------------------------------------------------------------------------------------------

/**
   \brief subscriber topic to move servos in a physical position

   Subscriber for the servos_absolute topic which processes one or more servos and sets their physical pulse value.

   When working with a continuous rotation servo,
   the topic is used to find the center position of a servo by sending successive values until a stopped position is identified.
   The topic is also used to find the range of a servo - the fastest forward and fasted reverse values. The difference between these two is the servo's range.
   Due to variences in servos, each will likely have a slightly different center value.

   When working with a fixed 180 degree rotation servo,
   the topic is used to find the center position of a servo by sending successive values until a the desired middle is identified.
   The topic is also used to find the range of a servo - the maximum clockwise and anti clockwise positions. The difference between these two is the servo's range.
   If the servo rotates slightly more in one direction from center than the other, then '2X' the lesser value should be used as the range to preserve the middle stop position.

   Hint: setting the servo pulse value to zero (0) causes the servo to power off.
   This is refered to as 'coast'. setting a servo to its center value leaves the servo powered and is refered to as 'brake'.

   @param msg  a 'ServoArray' message (array of one or more 'Servo') where the servo:value is the pulse position/speed

   __htl_ooe_smart_car::ServoArray__
   \include "ServoArray.msg"
   __htl_ooe_smart_car::Servo__
   \include "Servo.msg"

   __Example__
   \code{.sh}
   # this example makes the following assumptions about the hardware
   # the servos have been connected to the PWM board starting with the first connector and proceeding in order
   # any drive servos used for the left side servos are mounted opposite of those for the right side

   # the follow message sets the PWM value of the first two servos to 250 and 350 respectively.
   # depending on center value of each servo, these values may casue forward or backward rotation

   rostopic pub -1 /pwm/servos_absolute htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: 1500000}, {servo: 13, value: 1550000}]}"

   # the following messages are an example of finding a continuous servo's center
   # in this example the center is found to be 1500300

   rostopic pub -1 /pwm/servos_absolute htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: 1500000}]}"
   .
   .
   .
   rostopic pub -1 /pwm/servos_absolute htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: 1500300}]}"

   # power off servo - eg 'coast' rather than 'brake'
   rostopic pub -1 /servos_absolute pwm/ServoArray "{servos:[{servo: 1, value: 0}]}"
   \endcode

 */
void servos_absolute (const htl_ooe_smart_car::ServoArray::ConstPtr& msg) {

    for(std::vector<htl_ooe_smart_car::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
        int servo = sp->servo;
        int value = sp->value;

        if((value < 0) || (value > (NSEC_PER_SEC/_pwm_frequency))) {
            ROS_ERROR("Invalid PWM value %d :: PWM values must be between 0 and %d", value, NSEC_PER_SEC/_pwm_frequency);
            continue;
        }
        _set_duty_cycle (_servo_configs[servo], value);
        ROS_DEBUG("servo[%d] = %d", servo, value);
    }
}

/**
   \brief subscriber topic to move servos in the range of ±1.0

   Subscriber for controlling servos using proportional values.
   This topic processes one or more servos and sets their physical pulse value based on
   each servos physical range proportional to a range of ±1.0.

   When working with a continuous rotation servo,
   the topic is used to adjust the speed of the servo.

   When working with a fixed 180 degree rotation servo,
   the topic is used to adjust the position of the servo.

   This topic requires the use of the config_servos() service.
   Once the configuration of the servos - center position, direction of rotation, and PWM range - has been set,
   these data are to convert the proportional value to a physical PWM value specific to each servo.

   @param msg  a 'ServoArray' message (array of one or more 'Servo') where the servo:value is a relative position/speed

   __htl_ooe_smart_car::ServoArray Message__
   \include "ServoArray.msg"
   __htl_ooe_smart_car::Servo Message__
   \include "Servo.msg"

   __Example__
   \code{.sh}
   # this example makes the following assumptions about the hardware
   # the servos have been connected to the PWM board starting with the first connector and proceeding in order
   # any drive servos used for the left side servos are mounted opposite of those for the right side

   # this example uses 2 servos
   # the first servo is the left and the second servo is the right

   # configure two continuous rotation servos associated with the drive system - these servos were determined to have a ragee of ±50

   rosservice call /config_servos "servos: [{servo: 0, center: 1500000, range: 1000000, direction: -1}, \
                                            {servo: 1, center: 1500000, range: 1000000, direction: 1}]"

   # drive both servos forward at 40% of maximum speed

   rostopic pub -1 /pwm/servos_proportional htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: 0.40}, {servo: 15, value: 0.40}]}"

   # additionally configure one 180 degree servo (±90) used for a robot arm - this servo was determine to have a ragee of ±188

   rosservice call /config_servos "servos: [{servo: 12, center: 1500000, range: 1000000, direction: -1}]"

   # drive the arm servo to its 45 degree position and then to its -45 degree position

   rostopic pub -1 /pwm/servos_proportional htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: 0.50}]}"
   rostopic pub -1 /pwm/servos_proportional htl_ooe_smart_car/ServoArray "{servos:[{servo: 12, value: -0.50}]}"
   \endcode
 */
void servos_proportional (const htl_ooe_smart_car::ServoArray::ConstPtr& msg) {

    for(std::vector<htl_ooe_smart_car::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
        int servo = sp->servo;
        float value = sp->value;
        _set_pwm_interval_proportional (_servo_configs[servo], value);
    }
}

/**
   \brief subscriber topic to move servos based on a drive mode

   Subscriber for controlling a group of servos in concert based on a geometry_msgs::Twist message.

   This topic requires the use the config_servos() service to configure the servos for proportional control
   and the use of the config_drive_mode() to speccify the desired type of drive and to assign individual servos to the positions in the drive system.

   @param msg a geometry Twist message

   __geometry_msgs::Twist__
   \include "Twist.msg"
   __geometry_msgs::Vector3__
   \include "Vector3.msg"

   __Example__
   \code{.sh}
   # this example makes the following assumptions about the hardware
   # the servos have been connected to the PWM board starting with the first connector and proceeding in order
   # any drive servos used for the left side servos are mounted opposite of those for the right side

   # ROS used m/s (meters per second) as th standard unit for velocity.
   # The geometry_msgs::Twist linear and angular vectors are in m/s and radians/s respectively.

   # a typical high-speed servo is capable of 0.16-0.2 sec/60deg or aproximately 50-65 RPM.
   # using wheel with 11 cm diameter results in a circumference of 0.35 meters
   # the theoretical maximum speed of this combination is 0.30 m/s - 0.40 m/s


   # differential drive example
   # this example uses 2 servos
   # the first servo is the left and the second servo is the right

   # configure drive mode for two RC servos attached to 110mm diameter wheels

   rosservice call /config_servos "servos: [{servo: 0, center: 1500000, range: 100000, direction: -1}, \
                                            {servo: 1, center: 1500000, range: 100000, direction: 1}]"

   rosservice call /config_drive_mode "{mode: differential, rpm: 60.0, radius: 5.5, track: 5, scale: 1.0, \
                                        servos: [{servo: 0, value: 0}, {servo: 1, value: 1}]}"

   # moving forward at 0.35 m/s (or best speed)

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.35, 0.0, 0.0], angular: [0.0, 0.0, 0.0]}"

   # left 45 degrees per second turn while moving forward at 0.35 m/s (or best speed)

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.35, 0.0, 0.0], angular: [0.0, 0.0, -0.7854]}"

   # pivoting clockwise at 90 degrees per second

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.0, 0.0, 0.0], angular: [0.0, 0.0, 1.5708]}"


   # a mecanum drive example
   # this example assumes there are 4 servos
   # the servos are 0, 1, 2, & 3 and correspond to left-front, right-front, left-rear, and right-rear

   # configure drive mode for four servos

   rosservice call /config_servos "servos: [{servo: 0, center: 1500000, range: 100000, direction: 1},  \
                                            {servo: 1, center: 1500000, range: 100000, direction: -1}, \
                                            {servo: 2, center: 1500000, range: 100000, direction: -1}, \
                                            {servo: 3, center: 1500000, range: 100000, direction: -1}]"

   rosservice call /config_drive_mode "{mode: mecanum,  rpm: 60.0, radius: 5.5, track: 5, scale: 1.0, \
                                        servos: [{servo: 0, value: 2}, {servo: 1, value: 1}, \
                                                 {servo: 2, value: 3}, {servo: 3, value: 4}]}"

   # moving forward at full speed

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.35, 0.0, 0.0], angular: [0.0, 0.0, 0.0]}"

   # moving forward and to the right while always facing forward

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.35, 0.15, 0.0], angular: [0.0, 0.0, 0.0]}"

   # right turn while moving forward

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.35, 0.0, 0.0], angular: [0.0, 0.0, 0.7854]}"

   # pivoting clockwise

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.0, 0.0, 0.0], angular: [0.0, 0.0, 1.5708]}"

   # moving sideways to the right

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.0, 0.2, 0.0], angular: [0.0, 0.0, 0.0]}"

   # moving sideways to the left

   rostopic pub -1 /servos_drive geometry_msgs/Twist "{linear: [0.0, -0.2, 0.0], angular: [0.0, 0.0, 0.0]}"

   \endcode
 */
void servos_drive (const geometry_msgs::Twist::ConstPtr& msg) {

    int i;
    float delta, range, ratio;
    float temp_x, temp_y, temp_r;
    float dir_x, dir_y, dir_r;
    float speed[4];

    /* msg is a pointer to a Twist message: msg->linear and msg->angular each of which have members .x .y .z */
    /* the subscriber uses the maths from: http://robotsforroboticists.com/drive-kinematics/ */

    ROS_DEBUG("servos_drive Twist = [%5.2f %5.2f %5.2f] [%5.2f %5.2f %5.2f]",
             msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

    if(_active_drive.mode == MODE_UNDEFINED) {
        ROS_ERROR("drive mode not set");
        return;
    }
    if((_active_drive.mode < MODE_UNDEFINED) || (_active_drive.mode >= MODE_INVALID)) {
        ROS_ERROR("unrecognized drive mode set %d", _active_drive.mode);
        return;
    }

    dir_x = ((msg->linear.x  < 0) ? -1 : 1);
    dir_y = ((msg->linear.y  < 0) ? -1 : 1);
    dir_r = ((msg->angular.z < 0) ? -1 : 1);

    temp_x = _active_drive.scale * _abs(msg->linear.x);
    temp_y = _active_drive.scale * _abs(msg->linear.y);
    temp_r = _abs(msg->angular.z);    // radians

    // temp_x = _smoothing (temp_x);
    // temp_y = _smoothing (temp_y);
    // temp_r = _smoothing (temp_r) / 2;

    // the differential rate is the robot rotational circumference / angular velocity
    // since the differential rate is applied to both sides in opposite amounts it is halved
    delta = (_active_drive.track / 2) * temp_r;
    // delta is now in meters/sec

    // determine if we will over-speed the motor and scal accordingly
    ratio = _convert_mps_to_proportional(temp_x + delta);
    if(ratio > 1.0)
        temp_x /= ratio;


    switch (_active_drive.mode) {

    case MODE_ACKERMAN:
        /*
          with ackerman drive, steering is handled by a separate servo
          we drive assigned servos exclusively by the linear.x
        */
        speed[0] = temp_x * dir_x;
        speed[0] = _convert_mps_to_proportional(speed[0]);
        if(_abs(speed[0]) > 1.0)
            speed[0] = 1.0 * dir_x;

        ROS_DEBUG("ackerman drive mode speed=%6.4f", speed[0]);
        break;

    case MODE_DIFFERENTIAL:
        /*
          with differential drive, steering is handled by the relative speed of left and right servos
          we drive assigned servos by mixing linear.x and angular.z
          we compute the delta for left and right components
          we use the sign of the angular velocity to determine which is the faster / slower
        */

        /* the delta is the angular velocity * half the drive track */

        if(dir_r > 0) {    // turning right
            speed[0] = (temp_x + delta) * dir_x;
            speed[1] = (temp_x - delta) * dir_x;
        } else {        // turning left
            speed[0] = (temp_x - delta) * dir_x;
            speed[1] = (temp_x + delta) * dir_x;
        }

        ROS_DEBUG("computed differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

        /* if any of the results are greater that 1.0, we need to scale all the results down */
        range = _max (_abs(speed[0]), _abs(speed[1]));

        ratio = _convert_mps_to_proportional(range);
        if(ratio > 1.0) {
            speed[0] /= ratio;
            speed[1] /= ratio;
        }
        ROS_DEBUG("adjusted differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

        speed[0] = _convert_mps_to_proportional(speed[0]);
        speed[1] = _convert_mps_to_proportional(speed[1]);

        ROS_DEBUG("differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);
        break;

    case MODE_MECANUM:
        /*
          with mecanum drive, steering is handled by the relative speed of left and right servos
          with mecanum drive, lateral motion is handled by the rotation of front and rear servos
          we drive assigned servos by mixing linear.x and angular.z  and linear.y
        */

        if(dir_r > 0) {    // turning right
            speed[0] = speed[2] = (temp_x + delta) * dir_x;
            speed[1] = speed[3] = (temp_x - delta) * dir_x;
        } else {        // turning left
            speed[0] = speed[2] = (temp_x - delta) * dir_x;
            speed[1] = speed[3] = (temp_x + delta) * dir_x;
        }

        speed[0] += temp_y * dir_y;
        speed[3] += temp_y * dir_y;
        speed[1] -= temp_y * dir_y;
        speed[2] -= temp_y * dir_y;
        ROS_DEBUG("computed mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

        range = _max (_max (_max (_abs(speed[0]), _abs(speed[1])), _abs(speed[2])), _abs(speed[3]));
        ratio = _convert_mps_to_proportional(range);
        if(ratio > 1.0) {
            speed[0] /= ratio;
            speed[1] /= ratio;
            speed[2] /= ratio;
            speed[3] /= ratio;
        }
        ROS_DEBUG("adjusted mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

        speed[0] = _convert_mps_to_proportional(speed[0]);
        speed[1] = _convert_mps_to_proportional(speed[1]);
        speed[2] = _convert_mps_to_proportional(speed[2]);
        speed[3] = _convert_mps_to_proportional(speed[3]);

        ROS_DEBUG("mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);
        break;

    default:
        break;

    }

    /* find all drive servos and set their new speed */
    for(auto &servo_config : _servo_configs)
        switch (_active_drive.mode) {
        case MODE_MECANUM:
            if(servo_config.mode_pos == POSITION_RIGHTREAR_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[3]);
            if(servo_config.mode_pos == POSITION_LEFTREAR_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[2]);
            if(servo_config.mode_pos == POSITION_RIGHTFRONT_PHASE_A)
            _set_pwm_interval_proportional (servo_config, speed[1]);
            if(servo_config.mode_pos == POSITION_LEFTFRONT_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            break;
        case MODE_DIFFERENTIAL:
            if(servo_config.mode_pos == POSITION_LEFTFRONT_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            if(servo_config.mode_pos == POSITION_LEFTFRONT_PHASE_B)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            if(servo_config.mode_pos == POSITION_LEFTREAR_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            if(servo_config.mode_pos == POSITION_LEFTREAR_PHASE_B)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            if(servo_config.mode_pos == POSITION_RIGHTFRONT_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[1]);
            if(servo_config.mode_pos == POSITION_RIGHTFRONT_PHASE_B)
                _set_pwm_interval_proportional (servo_config, speed[1]);
            if(servo_config.mode_pos == POSITION_RIGHTREAR_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[1]);
            if(servo_config.mode_pos == POSITION_RIGHTREAR_PHASE_B)
                _set_pwm_interval_proportional (servo_config, speed[1]);
            break;
        case MODE_ACKERMAN:
            if(servo_config.mode_pos == POSITION_LEFTFRONT_PHASE_A)
                _set_pwm_interval_proportional (servo_config, speed[0]);
            break;
        }
}

// ------------------------------------------------------------------------------------------------------------------------------------
/**@}*/
/**
\defgroup Services Services interfaces provided by this package
@{ */
// services
// ------------------------------------------------------------------------------------------------------------------------------------

/**
   \brief service to set set PWM frequency

   The PWM boards drive LED and servos using pulse width modulation. The 12 bit interface means values are 0..4096.
   The size of the minimum width is determined by the frequency.

   __Warning:__ Changing the frequency will affect any active servos.

   @param [in] req an Int32 value for the requested pulse frequency
   @param [out] res the return value will be the new active frequency
   @returns true

   __htl_ooe_smart_car::IntValue__
   \include "IntValue.srv"

   __Example__
   \code{.sh}
   # Analog RC servos are most often designed for 20ms pulses. This is achieved with a frequency of 50Hz.
   # This software defaults to 50Hz. Use the set_pwm_frequncy() to change this frequency value.
   # It may be necessary or convenient to change the PWM frequency if using DC motors connected to PWM controllers.
   # It may also be convenient if using PWM to control LEDs.

   rosservice call /set_pwm_frequency "{value: 50}"
   \endcode

 */
bool set_pwm_frequency (htl_ooe_smart_car::IntValue::Request &req, htl_ooe_smart_car::IntValue::Response &res) {
    int freq;
    freq = req.value;
    if((freq<12) || (freq>1024)) {
        ROS_ERROR("Invalid PWM frequency %d :: PWM frequencies should be between 12 and 1024", freq);
        freq = 50;    // most analog RC servos are designed for 20ms pulses.
        res.error = freq;
    }
    _set_pwm_frequency (freq);    // I think we must reset frequency when we change boards
    res.error = freq;
    return true;
}

/**
   \brief store configuration data for servos

   A service to set each servo's center value, direction of rotation (1 for forward and -1 for reverse motion),
   the center or nul value, range, and direction of one or more servos.
   and range between full left and right or maximun forward and backward speed.

   Setting these data are required before sending messages to the servos_proportional() topic as well as the servos_drive() topic.

   @param [in] req an array of 'ServoConfig' which consists of a servo number (one based), center(0..4096), range(0..4096), and direction (±1).
   @param [out] res integer non-zero if an error occured
   @returns true

   __htl_ooe_smart_car::ServosConfig__
   \include "ServosConfig.srv"
   __htl_ooe_smart_car::ServoConfig__
   \include "ServoConfig.msg"


   __Example__
   \code{.sh}
   # this example makes the following assumptions about the hardware
   # the servos have been connected to the PWM board starting with the first connector and proceeding in order
   # any drive servos used for the left side servos are mounted opposite of those for the right side

   # this example uses 2 servos
   # the first servo is the left and the second servo is the right

   # configure two continuous rotation servos associated with the drive system - these servos were determined to have a ragee of ±50

   rosservice call /config_servos "servos: [{servo: 1, center: 1500000, range: 100000, direction: -1}, \
                                            {servo: 2, center: 1500000, range: 100000, direction: 1}]"

   # additionally configure one 180 degree servo (±90) used for a robot arm - this servo was determine to have a ragee of ±188

   rosservice call /config_servos "servos: [{servo: 9, center: 1500000, range: 100000, direction: -1}]"

   \endcode
 */
bool config_servos (htl_ooe_smart_car::ServosConfig::Request &req, htl_ooe_smart_car::ServosConfig::Response &res) {

    int i;

    res.error = 0;

    for(i=0;i<req.servos.size();i++) {
        int servo = req.servos[i].servo;
        int center = req.servos[i].center;
        int range = req.servos[i].range;
        int direction = req.servos[i].direction;

        _config_servo (servo, center, range, direction);
    }

    return true;
}

/**
   \brief set drive mode and drive servos

   A service to set the desired drive mode. It must be called  before messages are handled by the servos_drive() topic.
   Setting these data are required before sending messages to the servos_proportional() topic.

   The drive mode consists of a string value for the type of drive desired: ackerman, differential, or mecanum.
   For each mode, the drive servos must be specified.

   @param req [in] DriveMode configuration data
   @param res [out] non-zero on error
   @returns true

   The DriveMode input requires drive system details included: wheel RPM, wheel radius, and track width.
   ROS uses meters for measurements. The values of radius and track are expected in meters.

   _A scale factor is available if necessary to compensate for linear vector values._

   The mode string is one of the following drive systems:
    -# 'ackerman' - (automobile steering) requires minimum of one servo for drive and uses some other servo for stearing..
    -# 'differential' - requires multiples of two servos, designated as left and right.
    -# 'mecanum' - requires multiples of four servos, designated as left-front, right-front, left-rear, and right-rear.

    The servo message is used for indicating which servos are used for the drive system.
    The message consists of 'servo' number, and data 'value'.

    The 'value' field indicates the positon the corresponding servo within the drive system.
    The applicable servos are assigned positions as follows:

    positon | ackerman | differential | mecanum
    --------|----------|--------------|--------
    position 0 corresponds to | drive | left-front | left-front
    position 1 corresponds to | | right-front | right-front
    position 2 corresponds to | | left-rear | left-rear
    position 3 corresponds to | | right-rear | right-rear

    __htl_ooe_smart_car::DriveMode__
    \include "DriveMode.srv"
    __htl_ooe_smart_car::Position__
    \include "Position.msg"

   __Example__
   \code{.sh}
   # this example makes the following assumptions about the hardware
   # the servos have been connected to the PWM board starting with the first connector and proceeding in order
   # any drive servos used for the left side servos are mounted opposite of those for the right side

   # ROS used m/s (meters per second) as th standard unit for velocity.
   # The geometry_msgs::Twist linear and angular vectors are in m/s and radians/s respectively.

   # differential drive example
   # this example uses 2 servos
   # the first servo is the left and the second servo is the right

   rosservice call /config_drive_mode "{mode: differential, rpm: 56.0, radius: 0.0055, track: 0.015, scale: 1.0, \
                                        servos: [{servo: 1, value: 1}, {servo: 2, value: 2}]}"

   # this example uses 4 servos
   # there are two servos for the left and two  fors the right

   rosservice call /config_drive_mode "{mode: differential, rpm: 56.0, radius: 0.0055, track: 0.015, scale: 1.0, \
                                        servos: [{servo: 1, value: 1}, {servo: 2, value: 2}, \
                                                 {servo: 3, value: 1}, {servo: 4, value: 2}]}"

   # mecanum drive example
   # this example uses 4 servos

   rosservice call /config_drive_mode "{mode: differential, rpm: 56.0, radius: 0.0055, track: 0.015, scale: 1.0, \
                                        servos: [{servo: 1, value: 1}, {servo: 2, value: 2}, \
                                                 {servo: 3, value: 1}, {servo: 4, value: 2}]}"

   \endcode
 */
bool config_drive_mode (htl_ooe_smart_car::DriveMode::Request &req, htl_ooe_smart_car::DriveMode::Response &res) {
    res.error = 0;

    int i;

    if((res.error = _config_drive_mode (req.mode, req.rpm, req.radius, req.track, req.scale)))
        return true;

    for(i=0;i<req.servos.size();i++) {
        int servo = req.servos[i].servo;
        int position = req.servos[i].position;

        if(_config_servo_position (servo, position) != 0) {
            res.error = servo; /* this needs to be more specific and indicate a bad server ID was provided */
            continue;
        }
    }

    return true;
}

/**
   \brief service to stop all servos on all boards

   A service to stop all of the servos on all of the PWM boards and set their power state to off / coast.

   This is different from setting each servo to its center value. A centered servo is still under power and it's in a brake state.

   @param req is empty
   @param res is empty
   @returns true

   __Example__
   \code{.sh}
   # stop all servos on all boards and setting them to coast rather than brake

   rosservice call /stop_servos
   \endcode

 */
bool stop_servos (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    for(auto &servo_config : _servo_configs)
        _set_duty_cycle(servo_config, 0);
    return true;
}

static std::string _get_string_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
    XmlRpc::XmlRpcValue &item = obj[param_name];
    if(item.getType() == XmlRpc::XmlRpcValue::TypeString)
        return item;

    ROS_WARN("invalid paramter type for %s - expected TypeString", param_name.c_str());
    return 0;
}

static int _get_int_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
    XmlRpc::XmlRpcValue &item = obj[param_name];
    if(item.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return item;

    ROS_WARN("invalid paramter type for %s - expected TypeInt", param_name.c_str());
    return 0;
}

static double _get_float_param (XmlRpc::XmlRpcValue obj, std::string param_name) {
    XmlRpc::XmlRpcValue &item = obj[param_name];
    if(item.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        return item;

    ROS_WARN("invalid paramter type for %s - expected TypeDouble", param_name.c_str());
    return 0;
}

static int _load_params (void) {
    ros::NodeHandle nhp;                    // not currently private namespace

    if(_init ()) {
        ROS_ERROR("Cannot initalize PWM");
        return 1;
    }

    int pwm;
    nhp.param ("pwm_frequency", pwm, 50);
    if(_set_pwm_frequency (pwm)) {
        ROS_ERROR("Cannot set PWM frequency");
        return 1;
    }

    /*
      // note: servos are numbered sequntially with '0'

      servo_config:
        - {servo: 0, center: 1500000, direction: -1, range: 100000}
        - {servo: 1, center: 1500000, direction: 1, range: 100000}

    */
    // attempt to load configuration for servos
    if(nhp.hasParam ("servo_config")) {
        XmlRpc::XmlRpcValue servos;
        nhp.getParam ("servo_config", servos);

        if(servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            ROS_DEBUG("Retrieving members from 'servo_config' in namespace(%s)", nhp.getNamespace().c_str());

            for(int32_t i = 0; i < servos.size(); i++) {
                XmlRpc::XmlRpcValue servo;
                servo = servos[i];    // get the data from the iterator
                if(servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    ROS_DEBUG("Retrieving items from 'servo_config' member %d in namespace(%s)", i, nhp.getNamespace().c_str());

                    // get the servo settings
                    int id, center, direction, range;
                    id = _get_int_param (servo, "servo");
                    center = _get_int_param (servo, "center");
                    direction = _get_int_param (servo, "direction");
                    range = _get_int_param (servo, "range");

                    if(id && center && direction && range) {
                        if((id >= 1) && (id <= MAX_SERVOS)) {
                            _set_pwm_frequency (pwm);
                            _config_servo (id, center, range, direction);
                        }
                        else
                            ROS_WARN("Parameter servo=%d is out of bounds", id);
                    }
                    else
                        ROS_WARN("Invalid parameters for servo=%d'", id);
                }
                else
                    ROS_WARN("Invalid type %d for member of 'servo_config' - expected TypeStruct(%d)", servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
            }
        }
        else
            ROS_WARN("Invalid type %d for 'servo_config' - expected TypeArray(%d)", servos.getType(), XmlRpc::XmlRpcValue::TypeArray);
    }
    else
        ROS_DEBUG("Parameter Server namespace[%s] does not contain 'servo_config", nhp.getNamespace().c_str());

    /*
      drive_config:
          mode: mecanum
        radius: 0.062
        rpm: 60.0
        scale: 0.3
        track: 0.2
        servos:
            - {servo: 0, position: 0}
            - {servo: 1, position: 1}
            - {servo: 2, position: 2}
            - {servo: 3, position: 3}
    */

    // attempt to load configuration for drive mode
    if(nhp.hasParam ("drive_config")) {
        XmlRpc::XmlRpcValue drive;
        nhp.getParam ("drive_config", drive);

        if(drive.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_DEBUG("Retrieving members from 'drive_config' in namespace(%s)", nhp.getNamespace().c_str());

            // get the drive mode settings
            std::string mode;
            float radius, rpm, scale, track;
            int id, position;

            mode = _get_string_param (drive, "mode");
            rpm = _get_float_param (drive, "rpm");
            radius = _get_float_param (drive, "radius");
            track = _get_float_param (drive, "track");
            scale = _get_float_param (drive, "scale");

            _config_drive_mode (mode, rpm, radius, track, scale);

            XmlRpc::XmlRpcValue &servos = drive["servos"];
            if(servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                ROS_DEBUG("Retrieving members from 'drive_config/servos' in namespace(%s)", nhp.getNamespace().c_str());

                for(int32_t i = 0; i < servos.size(); i++) {
                    XmlRpc::XmlRpcValue servo;
                    servo = servos[i];    // get the data from the iterator
                    if(servo.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                        ROS_DEBUG("Retrieving items from 'drive_config/servos' member %d in namespace(%s)", i, nhp.getNamespace().c_str());

                        // get the servo position settings
                        int id, position;
                        id = _get_int_param (servo, "servo");
                        position = _get_int_param (servo, "position");

                        if(id && position)
                            _config_servo_position (id, position); // had its own error reporting
                    }
                    else
                        ROS_WARN("Invalid type %d for member %d of 'drive_config/servos' - expected TypeStruct(%d)", i, servo.getType(), XmlRpc::XmlRpcValue::TypeStruct);
                }
            }
            else
                ROS_WARN("Invalid type %d for 'drive_config/servos' - expected TypeArray(%d)", servos.getType(), XmlRpc::XmlRpcValue::TypeArray);
        }
        else
            ROS_WARN("Invalid type %d for 'drive_config' - expected TypeStruct(%d)", drive.getType(), XmlRpc::XmlRpcValue::TypeStruct);
    }
    else
        ROS_DEBUG("Parameter Server namespace[%s] does not contain 'drive_config", nhp.getNamespace().c_str());
    return 0;
}

// ------------------------------------------------------------------------------------------------------------------------------------
/**@}*/
// main
// ------------------------------------------------------------------------------------------------------------------------------------
int main (int argc, char **argv) {
    int gpio = -1;
    // globals
    _pwm_frequency = 50; // set the initial pulse frequency to 50 Hz which is standard for RC servos

    ros::init (argc, argv, "i2cpwm_controller");

    ros::NodeHandle n("~");

    ros::ServiceServer freq_srv =   n.advertiseService ("set_pwm_frequency", set_pwm_frequency);
    ros::ServiceServer config_srv = n.advertiseService ("config_servos", config_servos);         // 'config' will setup the necessary properties of continuous servos and is helpful for standard servos
    ros::ServiceServer mode_srv =   n.advertiseService ("config_drive_mode", config_drive_mode); // 'mode' specifies which servos are used for motion and which behavior will be applied when driving
    ros::ServiceServer stop_srv =   n.advertiseService ("stop_servos", stop_servos);             // the 'stop' service can be used at any time

    ros::Subscriber abs_sub =   n.subscribe ("servos_absolute", 500, servos_absolute);         // the 'absolute' topic will be used for standard servo motion and testing of continuous servos
    ros::Subscriber rel_sub =   n.subscribe ("servos_proportional", 500, servos_proportional); // the 'proportion' topic will be used for standard servos and continuous rotation aka drive servos
    ros::Subscriber drive_sub = n.subscribe ("servos_drive", 500, servos_drive);               // the 'drive' topic will be used for continuous rotation aka drive servos controlled by Twist messages

    if(_load_params()) { // loads parameters and performs initialization
        ROS_ERROR("Cannot load Parameters");
        return 1;
    }

    gpio = pigpio_start(NULL, NULL);
    if(gpio < 0) {
        ROS_ERROR("Cannot initalize gpio Error %d", gpio);
        return 1;
    }
    set_mode(gpio, DRIVE_POWER_ENABLE, PI_OUTPUT);
    gpio_write(gpio, DRIVE_POWER_ENABLE, PI_ON);
    set_mode(gpio, PWM_OUTPUT_N_ENABLE, PI_OUTPUT);
    gpio_write(gpio, PWM_OUTPUT_N_ENABLE, PI_OFF);

    ros::spin();

    set_mode(gpio, DRIVE_POWER_ENABLE, PI_INPUT);
    set_mode(gpio, PWM_OUTPUT_N_ENABLE, PI_INPUT);
    pigpio_stop(gpio);
    _exit();

    return 0;
}
