cmake_minimum_required(VERSION 3.8)
# project(jetcobot_interfaces)
project(smartfactory_interfaces)


# 기본 빌드 옵션 (있어도 되고 없어도 됨)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 필수 =====
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ===== 인터페이스 정의 (ament_package 이전!) =====
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/ObserveMarkerPose.srv
  srv/ExecutePnP.srv

  # ---- msgs ----
  msg/SectionResult.msg

  # ---- Actions ----
  action/ExecutePnP.action
  action/ObserveMarker.action
  action/ExecuteTask.action

  # ---- Services ----
  srv/GetCurrentCoords.srv
)

# 런타임 의존성 export
ament_export_dependencies(rosidl_default_runtime)

# 테스트 (선택)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ===== 반드시 마지막 =====
ament_package()