cmake_minimum_required(VERSION 2.8.3)
project(mofpy)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/mofpy_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
