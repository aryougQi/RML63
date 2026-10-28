cmake_minimum_required(VERSION 3.5)
project(rml63_tag_touch)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/touch_tag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/touch_geometry.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
