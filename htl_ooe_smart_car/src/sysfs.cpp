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

#include <cerrno> // ETIMEDOUT

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include <ros/ros.h>

#include "sysfs.hpp"

namespace sysfs
{
std::string read_attribute(fs::path p) {
    try {
        if(!fs::exists(p) || fs::is_directory(p))
              return emptyString;
        fs::ifstream ifile(p);
        std::string str;
        if(ifile && ifile.peek() == EOF)
            return emptyString;
        else
            ifile >> str;
        ifile.close();
        return str;
    }

    catch (const fs::filesystem_error& ex) {
        ROS_ERROR_STREAM( ex.what() );
        return emptyString;
    }
}

bool write_attribute(fs::path p, const std::string& content) {
    try {
        if(!fs::exists(p) || fs::is_directory(p)) {
            ROS_ERROR_STREAM(p << ": does not exist or is a directory");
            return false;
        }
        fs::ofstream ofile(p);
        if (!ofile) {
            ROS_ERROR_STREAM(p << ": could not open for writing");
            return false;
        }
        ofile.write(content.data(), static_cast<std::streamsize>(content.size()));
        if (!ofile) {
            ROS_ERROR_STREAM(p << ": write error");
            return false;
        }
        ofile.close();
        return true;
    }

    catch (const fs::filesystem_error& ex) {
        ROS_ERROR_STREAM( ex.what() );
        return false;
    }
}

std::vector<fs::path> find_devices(const std::string device_name) {
    static constexpr auto sysfs_path = "/sys/bus";
    fs::path p;
    auto all_matching_paths = std::vector<fs::path>{};

    for (auto const & device_path : fs::recursive_directory_iterator(sysfs_path)) {
        p = device_path.path();
        if (p.string().find("devices") != std::string::npos) {
            fs::path p_name = p / "name";
            if (fs::exists(p_name)) {
                if(read_attribute(p_name).compare(device_name) == 0) {
                    try {
                        p = fs::canonical(p);
                    }
                    catch (const std::system_error& e) {
                        continue;
                    }
                    all_matching_paths.push_back(p);
                }
            }
        }
    }
    return all_matching_paths;
}

std::vector<fs::path> find_dirs_containing(fs::path const & dir,
                                           const std::string file_name) {

    auto result = std::vector<fs::path>{};

    if(fs::exists(dir)) {
        for(auto const & entry :
            fs::recursive_directory_iterator(dir)) {
            if(fs::is_regular_file(entry) &&
               entry.path().filename().string() == file_name)
                result.push_back(entry.path().parent_path());
        }
    }
    return result;
}

#endif // __arm__

} // namespace sysfs
