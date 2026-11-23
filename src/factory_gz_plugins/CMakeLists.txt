cmake_minimum_required(VERSION 3.10)
project(factory_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

add_library(factory_grasp_attach SHARED src/grasp_attach.cpp)
target_compile_features(factory_grasp_attach PRIVATE cxx_std_17)
target_link_libraries(factory_grasp_attach gz-sim::gz-sim gz-plugin::register)
install(TARGETS factory_grasp_attach LIBRARY DESTINATION lib)

# let Gazebo find the plugin
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
