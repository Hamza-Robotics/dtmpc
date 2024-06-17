cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS

scripts/mobile_robot_model.py
scripts/mpc_controller_robot1.py
scripts/mpc_controller_robot2.py
scripts/mpc_controller_robot3.py


scripts/mpc_commander_robot1.py
scripts/mpc_commander_robot2.py

scripts/mpc_real_robot1.py
scripts/mpc_real_robot2.py
scripts/NMPC.py
scripts/TNMPC.py
scripts/ATNMPC.py
scripts/conversion_functions.py
scripts/solution_plotter.py
scripts/mpc_in_ros.py
scripts/robot_system.py
DESTINATION lib/${PROJECT_NAME}
)


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
