cmake_minimum_required(VERSION 3.0.2)

project(staubli_resources)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
