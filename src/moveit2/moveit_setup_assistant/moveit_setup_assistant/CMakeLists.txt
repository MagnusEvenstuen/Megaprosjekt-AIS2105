cmake_minimum_required(VERSION 3.22)
project(moveit_setup_assistant LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED program_options)
find_package(moveit_setup_framework REQUIRED)
find_package(moveit_setup_srdf_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    moveit_setup_framework
    moveit_setup_srdf_plugins
    pluginlib
    Qt5Core
    Qt5Widgets
    rclcpp)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS include/moveit_setup_assistant/navigation_widget.hpp
            include/moveit_setup_assistant/setup_assistant_widget.hpp)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES ${HEADERS})

add_executable(
  moveit_setup_assistant src/main.cpp src/setup_assistant_widget.cpp
                         src/navigation_widget.cpp ${MOC_FILES})
target_include_directories(
  moveit_setup_assistant
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_assistant>)
ament_target_dependencies(moveit_setup_assistant
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_setup_assistant ${Boost_LIBRARIES})

add_executable(moveit_setup_assistant_updater src/collisions_updater.cpp)
target_include_directories(
  moveit_setup_assistant_updater
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_assistant>)

ament_target_dependencies(moveit_setup_assistant_updater
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

set_target_properties(moveit_setup_assistant_updater
                      PROPERTIES OUTPUT_NAME collisions_updater PREFIX "")

install(
  TARGETS moveit_setup_assistant moveit_setup_assistant_updater
  EXPORT moveit_setup_assistantTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/moveit_setup_assistant
  INCLUDES
  DESTINATION include/moveit_setup_assistant)
install(DIRECTORY include/ DESTINATION include/moveit_setup_assistant)

ament_export_include_directories(include)
install(DIRECTORY launch DESTINATION share/moveit_setup_assistant)
install(DIRECTORY resources DESTINATION share/moveit_setup_assistant)
ament_export_targets(moveit_setup_assistantTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
