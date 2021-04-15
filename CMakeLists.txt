cmake_minimum_required(VERSION 3.14)
project("ROS2 RCCAR")

# usually I put this in a separate file include("/opt/ros/_common/Colcon.cmake")
function(colcon_add_subdirectories)
    cmake_parse_arguments(PARSE_ARGV 0 "ARG" "" "BUILD_BASE;BASE_PATHS" "")

    message("search criteria: ${ARGV}")

    execute_process(COMMAND colcon list
            --paths-only
            --base-paths ${ARG_BASE_PATHS}
            --topological-order
            ${ARG_UNPARSED_ARGUMENTS}
            OUTPUT_VARIABLE paths)
    string(STRIP "${paths}" paths)
    string(REPLACE "\n" ";" paths "${paths}")

    MESSAGE("colcon shows paths ${paths}")

    foreach(path IN LISTS paths)
        message("...examining ${path}")
        # if(EXISTS "${path}/CMakeLists.txt")
        execute_process(COMMAND colcon info --paths "${path}" OUTPUT_VARIABLE package_info)
        if(NOT "${package_info}" MATCHES "type:[ \t]+(cmake|ros.ament_cmake|ros.cmake)")
            message("skipping non-cmake project")
        elseif(NOT "${package_info}" MATCHES "name:[ \t]+([^ \r\n\t]*)")
            message(WARNING "could not identify package at ${path}")
        else()
            set(name "${CMAKE_MATCH_1}")
            message("...adding package ${name} from path ${path}")
            MESSAGE("package info: ${package_info}")

            get_filename_component(BUILD_PATH "${name}" ABSOLUTE BASE_DIR "${ARG_BUILD_BASE}")

            add_subdirectory("${path}" "${BUILD_PATH}")
        endif()
    endforeach()
endfunction()


if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^(arm|ARM)v7l$")
    message(STATUS ">>> Raspberry Pi")
    colcon_add_subdirectories(
        BUILD_BASE "${PROJECT_SOURCE_DIR}/build"
        BASE_PATHS "${PROJECT_SOURCE_DIR}/src/"
    )
endif()

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^(arm|ARM)v6l$")
    message(STATUS ">>> Raspberry Pi Zero")
    colcon_add_subdirectories(
            BUILD_BASE "${PROJECT_SOURCE_DIR}/build"
            BASE_PATHS "${PROJECT_SOURCE_DIR}/src/"
            --packages-skip arducam_stereo rplidar_ros
    )
endif()

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^(x86_|x86-|AMD|amd|x)64$")
    message(STATUS ">>> Linux")
    colcon_add_subdirectories(
        BUILD_BASE "${PROJECT_SOURCE_DIR}/build"
        BASE_PATHS "${PROJECT_SOURCE_DIR}/src/"
        --packages-skip arducam_stereo rplidar_ros
    )
endif()
