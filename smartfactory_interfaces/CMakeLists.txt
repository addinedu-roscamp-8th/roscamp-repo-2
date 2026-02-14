cmake_minimum_required(VERSION 3.8)
project(smartfactory_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)   
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------
# Topic / Service / Action Message Files
# --------------------------------------------------
set(msg_files
  "msg/Part.msg"
  "msg/PartArray.msg"
  "msg/SectionResult.msg"
  "msg/StorageResponse.msg"
  "msg/StorageRequest.msg"
  "msg/ManualRequest.msg"
  "msg/ManualResponse.msg"
  )

set(srv_files

  )

set(action_files

)

# --------------------------------------------------
# Generate Interface Code
# --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
  )

# --------------------------------------------------
# Testing (optional)
# --------------------------------------------------
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

# --------------------------------------------------
# Export & Package
# --------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
