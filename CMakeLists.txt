cmake_minimum_required(VERSION 2.8.3)

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
    DEPENDS ic_barcode
    CATKIN_DEPENDS roscpp nodelet std_msgs geometry_msgs sensor_msgs message_runtime image_transport cv_bridge dynamic_reconfigure
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(cfg/cpp ${catkin_INCLUDE_DIRS} ${ic_barcode_INCLUDE_DIRS})

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${ic_barcode_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/ic_barcode_scanner_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(ic_barcode_scanner_node
  src/ic_barcode_scanner_node.cpp
)
target_link_libraries(ic_barcode_scanner_node ${PROJECT_NAME} ${LIBRARIES})
add_dependencies(ic_barcode_scanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

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
