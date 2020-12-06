#ifndef SYSFS_HPP_INCLUDED
#define SYSFS_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

namespace sysfs
{

static const auto emptyString = "";

std::vector<fs::path> find_dirs_containing(fs::path const & dir,
                                           const std::string file_name);
std::vector<fs::path> find_devices(const std::string device_name);
std::string read_attribute(fs::path p);
bool write_attribute(fs::path p, const std::string& content);

} // namespace sysfs

#endif // #ifndef SYSFS_HPP_INCLUDED
