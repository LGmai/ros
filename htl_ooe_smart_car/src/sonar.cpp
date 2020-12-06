/*
 * Copyright (c) 2020, Franz Parzer
 * based on pi_sonar from Ubiquity Robotics
 * https://github.com/UbiquityRobotics/pi_sonar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */
 
#ifdef __arm__

#include <stdio.h>
#include <cerrno> // ETIMEDOUT

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "sysfs.hpp"

#define IIO_DEVICE_NAME "srf04"
#define STACK_SIZE (256*1024)

double min_freq = 0.5;
double max_freq = 60;

namespace fs = boost::filesystem;

class Sonar {
public:
    fs::path iio_device_path;
    fs::path distance_raw_path;
    fs::path of_node;
    int id;

    std::string frame;
    ros::Publisher pub;

    bool range_error = false;

    std::unique_ptr<diagnostic_updater::Updater> updater;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> pub_freq;

    Sonar(fs::path iio_device_path, fs::path of_node, int id, ros::NodeHandle& nh) {
        this->iio_device_path = iio_device_path;
        this->distance_raw_path = iio_device_path / "in_distance_raw";
        this->of_node = of_node;
        this->id = id;

        frame = str(boost::format{"%1%_%2%"} % of_node.string() % id);
        boost::replace_all(frame, "@", "_");
        pub = nh.advertise<sensor_msgs::Range>(frame, 1);

        updater.reset(new diagnostic_updater::Updater());
        updater->setHardwareIDf("sonar_%d",id);
        pub_freq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
            str(boost::format{"/pi_sonar/%1%"} % id), 
            *updater,
            diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10))
        );
        updater->add(str(boost::format{"Sonar %1% Range Checker"} % id),
            this,
            &Sonar::range_check
        );
    }

    void range_check(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (range_error) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Range out of bounds!");
        }
        else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Range within bounds!");
        }
    }
};

static std::vector<Sonar> sonars;

bool find_srf04( std::vector<Sonar> *sonars, ros::NodeHandle nh) { // placing devices here if found
    int id = 0;

    for(fs::path p: sysfs::find_devices(IIO_DEVICE_NAME)) {
        fs::path of_node_path = read_symlink(p / "of_node");
        sonars->push_back(Sonar(p,
                                of_node_path.filename(),
                                id,
                                nh));
        id++;
    }
    return id;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "Sonar");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(50);

    double field_of_view;
    double min_range;
    double max_range;
    std::string attribute; 

    nh.param<double>("field_of_view", field_of_view, 0.43632347);
    nh.param<double>("min_range", min_range, 0.05);
    nh.param<double>("max_range", max_range, 6.45);

    if(!find_srf04(&sonars, nh)) {
        ROS_ERROR("No IIO proximity devices found");
        return false;
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::Range>("/sonars", sonars.size());

    sensor_msgs::Range msg;
    msg.field_of_view = field_of_view;
    msg.min_range = min_range;
    msg.max_range = max_range;
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

    ROS_INFO("sonar Sonar node ready");

    /* update sonar 20 times a second, timer #0 */
    while (ros::ok()) {
        for (auto& sonar: sonars) {
            attribute = sysfs::read_attribute(sonar.distance_raw_path);
            if (attribute == sysfs::emptyString)
                sonar.range_error = true;
            else {
                msg.range = std::stof(attribute) / 1000;
                sonar.range_error = false;
            }
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = sonar.frame;
            pub.publish(msg);
            sonar.pub.publish(msg);
            sonar.pub_freq->tick();
            sonar.updater->update();
        }
        loop_rate.sleep();
    }
}

#else

#include <stdio.h>

int main(int argc, char **argv) {
    fprintf(stderr, "sonar only works on the Raspberry Pi\n");
    return 1;
}

#endif // __arm__