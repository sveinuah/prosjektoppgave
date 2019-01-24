#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
# set -x

# set up paths of cc and cxx compiler
if [ "$1" == "gcc" ]; then
    export CC="gcc"
    export CXX="g++"
else
    if [ "$(uname)" == "Darwin" ]; then

        export CC=/usr/local/opt/llvm-5.0/bin/clang-5.0
        export CXX=/usr/local/opt/llvm-5.0/bin/clang++-5.0
    else

        export CC="clang-5.0"
        export CXX="clang++-5.0"
    fi
fi

#check for correct verion of llvm
if [[ ! -d "llvm-source-50" ]]; then 
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/llvm.git llvm-source-50
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxx.git llvm-source-50/projects/libcxx
    git clone --depth=1 -b release_50  https://github.com/llvm-mirror/libcxxabi.git llvm-source-50/projects/libcxxabi
else
    echo "folder llvm-source-50 already exists, skipping git clone..."
fi

# check for libc++
if [[ !(-d "./llvm-build/output/lib") ]]; then
    echo "Building libc++"
    rm -rf llvm-build
    mkdir -p llvm-build
    pushd llvm-build >/dev/null

    cmake -DCMAKE_C_COMPILER=${CC} -DCMAKE_CXX_COMPILER=${CXX} \
      -LIBCXX_ENABLE_EXPERIMENTAL_LIBRARY=OFF -DLIBCXX_INSTALL_EXPERIMENTAL_LIBRARY=OFF \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=./output \
            ../llvm-source-50

    make cxx

    #install libc++ locally in output folder
    make install-libcxx install-libcxxabi
    popd >/dev/null
fi

# check for rpclib
if [ ! -d "external/rpclib/rpclib-2.2.1" ]; then
    echo "*********************************************************************************************"
    echo "Downloading rpclib..."
    echo "*********************************************************************************************"

    wget  https://github.com/rpclib/rpclib/archive/v2.2.1.zip

    # remove previous versions
    rm -rf "external/rpclib"

    mkdir -p "external/rpclib"
    unzip v2.2.1.zip -d external/rpclib
    rm v2.2.1.zip
fi

#install EIGEN library
if [[ !(-d "./AirLib/deps/eigen3/Eigen") ]]; then 
    rm -rf ./AirLib/deps/eigen3/Eigen
    echo "downloading eigen..."
    wget http://bitbucket.org/eigen/eigen/get/3.3.2.zip
    unzip 3.3.2.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen
    rm 3.3.2.zip
fi


# variable for build output
build_dir=build_debug
echo "putting build in build_debug folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi

if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
    pushd $build_dir  >/dev/null

    cmake ../cmake -DCMAKE_BUILD_TYPE=Debug \
        || (popd && rm -r $build_dir && exit 1)
    popd >/dev/null
fi

pushd $build_dir  >/dev/null
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make 
popd >/dev/null


mkdir -p AirLib/lib/x64/Debug
mkdir -p AirLib/deps/rpclib/lib
cp $build_dir/output/lib/libAirLib.a AirLib/lib
cp $build_dir/output/lib/librpc.a AirLib/deps/rpclib/lib/librpc.a

# Update AirLib/lib, AirLib/deps, Plugins folders with new binaries
rsync -a --delete $build_dir/output/lib/ AirLib/lib/x64/Debug
rsync -a --delete external/rpclib/rpclib-2.2.1/include AirLib/deps/rpclib


set +x

echo ""
echo ""
echo "=================================================================="
echo " FisheyeCapture is built! "
echo "=================================================================="


popd >/dev/null
