#! /bin/bash

#################################################################################
# NOTE! This script assumes installed ROS with catlkin and CMAKE v3.5 or higher #
#################################################################################


#Get path of current script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

#Exit immediately on non-zero status
set -e

echo ""
echo "************************************"
echo "Building ROS packages with catkin"
echo "************************************"

if [ "$1" == "gcc" ]; then
	export CC="gcc"
    export CXX="g++"
else
	export CC="clang-5.0"
	export CXX="clang++-5.0"
fi

pushd catkin_ws >/dev/null
catkin_make	--cmake-args -DCMAKE_BUILD_TYPE="debug" -Wall -Wextra
popd >/dev/null

popd >/dev/null
