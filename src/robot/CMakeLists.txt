cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_program(IGN_BIN ign REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# set(ROBOT_URDF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf)
# set(ROBOT_SDF_PATH ${CMAKE_CURRENT_BINARY_DIR}/../../install/${PROJECT_NAME}/share/${PROJECT_NAME}/urdf/robot.sdf)

# exec_program(
#   ${IGN_BIN} ${CMAKE_CURRENT_SOURCE_DIR} 
#   ARGS "sdf -p ${ROBOT_URDF_PATH} >> ${ROBOT_SDF_PATH}"
#   OUTPUT_VARIABLE OUT
#   RETURN_VALUE RET
# )

# message(${OUT})
# message(${RET})