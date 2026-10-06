target_sources(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer/buzzer.c
  ${CMAKE_CURRENT_SOURCE_DIR}/led/led.c
  ${CMAKE_CURRENT_SOURCE_DIR}/plotter_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/plotter/plotter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu/yaw_estimator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/linkage/linkage_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/linkage/linkage_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/motor/gm6020_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/motor/motor_state.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/remote/dbus.cpp
)

target_include_directories(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer
  ${CMAKE_CURRENT_SOURCE_DIR}/led
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/plotter
  ${CMAKE_CURRENT_SOURCE_DIR}/imu
  ${CMAKE_CURRENT_SOURCE_DIR}/linkage
  ${CMAKE_CURRENT_SOURCE_DIR}/motor
  ${CMAKE_CURRENT_SOURCE_DIR}/remote
)
