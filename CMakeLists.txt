cmake_minimum_required(VERSION 3.0.2)

project(ic_barcode_scanner)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  image_transport
  cv_bridge
  dynamic_reconfigure
)
find_package(IcBarcode REQUIRED)

add_message_files(
   FILES
   IcBarcodeResult.msg
   IcBarcodeResults.msg
)

add_service_files(
   FILES
   find_barcodes.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

generate_dynamic_reconfigure_options(cfg/IcBarcodeConfig.cfg)

catkin_package(
    DEPENDS IcBarcode
    CATKIN_DEPENDS roscpp nodelet std_msgs geometry_msgs sensor_msgs message_runtime image_transport cv_bridge dynamic_reconfigure
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(cfg
  ${catkin_INCLUDE_DIRS}
  ${IcBarcode_INCLUDE_DIRS})

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${IcBarcode_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/ic_barcode_scanner_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ic_barcode_scanner_node
  src/ic_barcode_scanner_node.cpp
)
target_link_libraries(ic_barcode_scanner_node ${PROJECT_NAME} ${LIBRARIES})
add_dependencies(ic_barcode_scanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} ic_barcode_scanner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
