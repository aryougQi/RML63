cmake_minimum_required(VERSION 3.5)
project(rml63_handeye)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/rectify_color.py scripts/auto_calibrate.py scripts/publish_calibration.py scripts/image_debug_view.py scripts/sync_camera_time.py scripts/validate_calibration.py scripts/backup_samples.py scripts/collect_joint_calibration.py scripts/optimize_joint_calibration.py scripts/auto_collect_joints.py scripts/publish_joint_calibration.py scripts/export_joint_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/joint_calibration_model.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md JOINT_CALIBRATION.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
