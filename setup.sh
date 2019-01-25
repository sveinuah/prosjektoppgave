#Get path of current script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo ${SCRIPT_DIR}
pushd "$SCRIPT_DIR" >/dev/null

#Exit immediately on non-zero status
set -e

rsync -a --delete ../AirSim/AirLib/ AirLib
rsync -a --delete ../AirSim/external/ external

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

# set up paths of cc and cxx compiler
if [ "$1" == "gcc" ]; then
	export C_COMPILER="gcc"
    export COMPILER="g++"
else
	export C_COMPILER=clang-5.0
	export COMPILER=clang++-5.0
fi

if [ "${COMPILER}" == "clang++-5.0" ]; then
	#get libc++ source
	if [[ ! -d "llvm-source-50" ]]; then 
    	git clone --depth=1 -b release_50  https://github.com/llvm-mirror/llvm.git llvm-source-50
    	git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxx.git llvm-source-50/projects/libcxx
    	git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxxabi.git llvm-source-50/projects/libcxxabi
	else
    	echo "folder llvm-source-50 already exists, skipping git clone..."
	fi

	#(re)build libc++
	rm -rf llvm-build
	mkdir -p llvm-build
	pushd llvm-build >/dev/null


	cmake -DCMAKE_C_COMPILER=${C_COMPILER} \
		-DCMAKE_CXX_COMPILER=${COMPILER} \
      	-LIBCXX_ENABLE_EXPERIMENTAL_LIBRARY=OFF \
      	-DLIBCXX_INSTALL_EXPERIMENTAL_LIBRARY=OFF \
      	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
      	-DCMAKE_INSTALL_PREFIX=./output \
      	../llvm-source-50

	make cxx

	#install libc++ locally in output folder
	make install-libcxx install-libcxxabi

	popd >/dev/null
fi

popd >/dev/null