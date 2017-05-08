cmake_minimum_required(VERSION 2.8.3)
project(aruco_gridboard)

find_package( OpenCV REQUIRED )


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  geometry_msgs
  message_filters
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs

)



#find_package(Boost REQUIRED
#  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


catkin_package(
   CATKIN_DEPENDS
   cv_bridge
   geometry_msgs
   message_filters
   roscpp
   sensor_msgs
   std_msgs
  )

add_executable(aruco_gridboard
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

target_link_libraries(aruco_gridboard
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )


add_executable(create_board
   src/create_board.cpp
  )
target_link_libraries(create_board
    ${OpenCV_LIBS}
    )

