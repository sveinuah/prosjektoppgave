macro(AddExecutableSource)
    SET(PROJECT_CPP ${PROJECT_NAME}_SOURCES)
    file(GLOB_RECURSE PROJECT_CPP "${FISHEYECAPTURE_ROOT}/${PROJECT_NAME}/src/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
endmacro(AddExecutableSource)

macro(SetupConsoleBuild)
    IF(UNIX)
    ELSE()
        SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
        SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
    ENDIF()
endmacro(SetupConsoleBuild)

macro(CommonSetup)
    message(STATUS "Running CommonSetup...")

    find_package(Threads REQUIRED)

    find_path(FISHEYECAPTURE_ROOT NAMES README.md PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
    message(STATUS "found FISHEYECAPTURE_ROOT=${FISHEYECAPTURE_ROOT}")

    #setup output paths
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    #libcxx which we will use with specific version of clang
    SET(LIBCXX_INC_PATH ${FISHEYECAPTURE_ROOT}/llvm-build/output/include/c++/v1)
    SET(LIBCXX_LIB_PATH ${FISHEYECAPTURE_ROOT}/llvm-build/output/lib)

    #setup include and lib for rpclib which will be referenced by other projects
    SET(RPCLIB_VERSION_FOLDER rpclib-2.2.1)
    SET(RPC_LIB_INCLUDES " ${FISHEYECAPTURE_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
    #name of .a file with lib prefix
    SET(RPC_LIB rpc)

    #what is our build type debug or release?
    string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

    IF(UNIX)
        SET(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        SET(BUILD_TYPE "linux")

        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
            #TODO: need to check why below is needed
            SET(CMAKE_CXX_STANDARD 14)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__CLANG__")
        else ()
            # other flags used in Unreal: -funwind-tables  -fdiagnostics-format=msvc -fno-inline  -Werror -fno-omit-frame-pointer  -fstack-protector -O2
            # TODO: add back -Wunused-parameter -Wno-documentation after rpclib can be compiled
            SET(CMAKE_CXX_FLAGS "\
                -std=c++14 -ggdb -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
                -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
                -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
                -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused -Wno-documentation -fdiagnostics-show-option \
                -pthread \
                ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                # make sure to match the compiler flags with which the Unreal
                # Engine is built with
                SET(CMAKE_CXX_FLAGS "\
                    -nostdinc++ -ferror-limit=10 -isystem ${LIBCXX_INC_PATH} \
                    -D__CLANG__ ${CMAKE_CXX_FLAGS}")

                # removed -lsupc++ from below (Git issue # 678)
                SET(CMAKE_EXE_LINKER_FLAGS "\
                    ${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++ -lc++abi -lm -lc \
                    -L ${LIBCXX_LIB_PATH} -rpath ${LIBCXX_LIB_PATH}")

                #do not use experimental as it might potentially cause ABI issues
                #set(CXX_EXP_LIB "-lc++experimental")

                if("${BUILD_TYPE}" STREQUAL "debug")
                    # set same options that Unreal sets in debug builds
                    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funwind-tables -fdiagnostics-format=msvc -fno-inline -fno-omit-frame-pointer -fstack-protector")
                endif()

            else()
                SET(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
            endif ()
        endif ()

        SET(BUILD_PLATFORM "x64")
        SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

    ELSE()
        #windows cmake build is experimental
        SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x0600 /GS /W4 /wd4100 /wd4505 /wd4820 /wd4464 /wd4514 /wd4710 /wd4571 /Zc:wchar_t /ZI /Zc:inline /fp:precise /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS /D_UNICODE /DUNICODE /WX- /Zc:forScope /Gd /EHsc ")
        SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT /DYNAMICBASE /INCREMENTAL:NO ")

        if("${BUILD_TYPE}" STREQUAL "debug")
          SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_DEBUG /MDd /RTC1 /Gm /Od ")
        elseif("${BUILD_TYPE}" STREQUAL "release")
          SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD /O2 /Oi /GL /Gm- /Gy /TP ")
        else()
          message(FATAL_ERROR "Please specify '-D CMAKE_BUILD_TYPE=Debug' or Release on the cmake command line")
        endif()
    ENDIF()

    ## TODO: we are not using Boost any more so below shouldn't be needed
    ## common boost settings to make sure we are all on the same page
    SET(Boost_USE_STATIC_LIBS ON)
    SET(Boost_USE_MULTITHREADED ON)
    #set(Boost_USE_STATIC_RUNTIME ON)

    ## TODO: probably should set x64 explicitly
    ## strip x64 from /machine:x64 from CMAKE_STATIC_LINKER_FLAGS and set in BUILD_PLATFORM
    if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
      string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
    endif()

endmacro(CommonSetup)