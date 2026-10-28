cmake_minimum_required(VERSION 3.16)
project(airground_mux LANGUAGES CXX)

# Gazebo Harmonic = gz-sim8 / transport13 / msgs10 / plugin2
find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(airground_mux SHARED AirGroundMux.cc)

target_link_libraries(airground_mux
  PRIVATE
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-plugin${GZ_PLUGIN_VER}::register
)

set_target_properties(airground_mux PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ""
)

install(TARGETS airground_mux
  LIBRARY DESTINATION lib)
