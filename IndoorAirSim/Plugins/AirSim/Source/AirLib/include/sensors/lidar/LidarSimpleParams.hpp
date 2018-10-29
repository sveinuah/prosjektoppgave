// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarSimpleParams_hpp
#define msr_airlib_LidarSimpleParams_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {

struct LidarSimpleParams {

    // Velodyne VLP-16 Puck config
    // https://velodynelidar.com/vlp-16.html

    // default settings
    // TODO: enable reading of these params from AirSim settings

    uint number_of_channels = 16; 
    real_T range = 10000.0f / 100;            // meters
    uint points_per_second = 100000;  
    uint horizontal_rotation_frequency = 10;  // rotations/sec
    int vertical_FOV_Upper = -15;             // drones -15, car +10
    int vertical_FOV_Lower = -45;             // drones -45, car -10

    Pose relative_pose {
        Vector3r(0,0,-1),                     // position - a little above vehicle (especially for cars) or Vector3r::Zero()
        Quaternionr::Identity()               // orientation - by default Quaternionr(1, 0, 0, 0) 
        };                       

    real_T update_frequency = 10;             // Hz
    real_T startup_delay = 0;                 // sec
};

}} //namespace
#endif
