cmake_minimum_required(VERSION 3.8)
project(urXe_driver_crospi)

message("--------------urXe_driver_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)

find_package(std_srvs REQUIRED)


# # Add this at the top of your CMakeLists.txt file
# find_package(pybind11 REQUIRED)

# # Add this before the pybind11_add_module calls
# # include_directories(${pybind11_INCLUDE_DIRS})

# -- start --Stuff for triple buffer implementation of boost 1.89
if(DEFINED ENV{BOOST_189_ROOT} AND NOT "$ENV{BOOST_189_ROOT}" STREQUAL "")
  set(_BOOST_1_89_CUSTOM_ROOT "$ENV{BOOST_189_ROOT}")
  message(STATUS "Using Boost 1.89 from BOOST_189_ROOT=${_BOOST_1_89_CUSTOM_ROOT}")
else()
  if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    set(_BOOST_1_89_CUSTOM_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
    message(STATUS "Using Boost 1.89 from HOME fallback: ${_BOOST_1_89_CUSTOM_ROOT}")
  else()
    message(FATAL_ERROR
      "Boost 1.89 requested, but neither BOOST_189_ROOT nor HOME is defined")
  endif()
endif()

# --------------------------------------------------
# Find Boost (headers-only or full, as needed)
# --------------------------------------------------
# find_package(Boost 1.89 REQUIRED
#   HINTS
#     ${_BOOST_1_89_CUSTOM_ROOT}
#   PATHS
#     ${_BOOST_1_89_CUSTOM_ROOT}
# )

# Optional but useful for debugging


set(BOOST_ROOT "${_BOOST_1_89_CUSTOM_ROOT}")
set(Boost_INCLUDE_DIR "${_BOOST_1_89_CUSTOM_ROOT}/include")
message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${_BOOST_1_89_CUSTOM_ROOT}" ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
message(STATUS "Using Boost version ${Boost_VERSION}")

find_package(Boost 1.89 REQUIRED)
message(STATUS "Boost include dirs after find_package: ${Boost_INCLUDE_DIR}")
# -- end -- Stuff for triple buffer implementation of boost 1.89


include_directories(
   include
)

set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

set(src_urXe_driver_crospi
    src/urXe_driver_crospi.cpp
)

add_library(urXe_driver_crospi 
            SHARED
            ${src_urXe_driver_crospi})

# set(MY_EXTERNAL_LIB "") #sets it empty such that it can be redefined later to include external lib

# add_subdirectory(external/ur_rtde)
# UNCOMENT THE FOLLOWING LINES AND MODIFY IF AN EXTERNAL LIBRARY IS DESIRED TO BE INCLUDED WITHIN THIS PACKAGE AND COMPILED BEFORE THE REST OF THE PACKAGE AS A DEPENDENCY
set(MY_EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/ur_rtde/)
set(MY_EXTERNAL_LIB_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/ur_rtde/include)
set(MY_EXTERNAL_LIB_DIR_SO_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/external/ur_rtde/build)

message("--------------MY_EXTERNAL_LIB_DIR: ${MY_EXTERNAL_LIB_DIR}----------------")

# add_custom_target(external_compiled_lib COMMAND mkdir -p build && cd build && cmake .. && make
#                    WORKING_DIRECTORY ${MY_EXTERNAL_LIB_DIR}
# )

# Custom command to build the external library
# add_custom_command(
#   OUTPUT ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}/librtde.so
#   COMMAND mkdir -p build && cd build && cmake .. && make
#   WORKING_DIRECTORY ${MY_EXTERNAL_LIB_DIR}
#   COMMENT "Building external library"
# )

add_custom_command(
  OUTPUT ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}/librtde.so
  COMMAND mkdir -p build && cd build && cmake .. 
          -DBOOST_ROOT=${BOOST_ROOT} 
          -DBOOST_INCLUDEDIR=${BOOST_INCLUDEDIR} 
          -DBOOST_LIBRARYDIR=${BOOST_LIBRARYDIR} 
      && make
  WORKING_DIRECTORY ${MY_EXTERNAL_LIB_DIR}
  COMMENT "Building external library"
)

# Custom target that depends on the output of the custom command
add_custom_target(external_compiled_lib ALL DEPENDS ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}/librtde.so)


include_directories(${MY_EXTERNAL_LIB_INCLUDE})

add_dependencies(urXe_driver_crospi external_compiled_lib) #Makes sure to compile first external_compiled_lib before the urXe_driver_crospi

message("-------------- MY_EXTERNAL_LIB_DIR compiled----------------")


find_library(MY_EXTERNAL_LIB
             NAMES librtde.so
             PATHS ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}
)
# set(ur_rtde_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/ur_rtde/build/ur_rtde)
# UR_RTDE
# find_package(ur_rtde REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/external/ur_rtde/build/ur_rtde) 
# include_directories(${ur_rtde_INCLUDE_DIRS})
# link_directories(${ur_rtde_LIBRARY_DIRS})



# find_package(ur_rtde REQUIRED)
# print the location of the library
# message("--------------ur_rtde_LIB: ${ur_rtde_LIBRARY_DIRS}----------------")
message("--------------MY_EXTERNAL_LIB: ${MY_EXTERNAL_LIB}----------------")



set(rosdependencies
  "crospi_core"
  "pluginlib"
  "rclcpp"
  "std_srvs"
)


set(dependencies
    fmt   
    # pugixml
    jsoncpp
    # ur_rtde
    # ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}/libFRIClient.so
    ${MY_EXTERNAL_LIB}
)

pluginlib_export_plugin_description_file(crospi_core plugins.xml)



target_include_directories(urXe_driver_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

ament_target_dependencies(urXe_driver_crospi ${rosdependencies})  
target_link_libraries(urXe_driver_crospi ${dependencies})

ament_export_dependencies(crospi_core)

ament_export_libraries(
  urXe_driver_crospi
)

install(
  TARGETS urXe_driver_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})

# install(TARGETS urXe_driver_crospi
#   DESTINATION lib/${PROJECT_NAME})


# ament_export_include_directories(
#   include
# )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
