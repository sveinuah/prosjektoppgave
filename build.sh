#! /bin/bash

#################################################################################
# NOTE! This script assumes installed ROS with catlkin and CMAKE v3.5 or higher #
#################################################################################


#Get path of current script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

#Exit immediately on non-zero status
set -e

#**************************************
#    AirSim prerequisites
#**************************************

#sync with AirSim repo.
if [[ ! -d "../AirSim" ]]; then
    echo "Build AirSim first in parent folder"
    exit 1
fi

# check for libc++
if [[ ! -d "./AirLib/deps/eigen3" ]]; then
    echo "ERROR: Eigen 3 not found. Please rebuild AirSim"
    exit 1
fi

# check for rpclib
if [ ! -d "./AirLib/deps/rpclib" ]; then
    echo "ERROR: rpclib not found. Please rebuild AirSim"
    exit 1
fi

# check for cmake build
if [ ! -d "./AirLib/deps/MavLinkCom" ]; then
    echo "ERROR: MavLinkCom dependency not found. Rebuild AirSim"
    exit 1
fi

rsync -a --delete ../AirSim/AirLib/ AirLib
rsync -a --delete ../AirSim/external/ external

echo ""
echo "************************************"
echo "Building ROS packages with catkin"
echo "************************************"

export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

pushd catkin_ws >/dev/null
catkin_make	
popd >/dev/null

popd >/dev/null




