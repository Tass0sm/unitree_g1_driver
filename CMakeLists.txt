cmake_minimum_required(VERSION 3.8)
project(unitree_g1_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  unitree_hg
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/g1_arm_sdk_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface unitree_g1_driver.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/release_arm_sdk.py scripts/inspire_hand.py scripts/inspire_hand_joint_states.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
