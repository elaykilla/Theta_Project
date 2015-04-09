cmake_minimum_required(VERSION 2.8)
project( load )

find_package(PCL 1.6 REQUIRED)
find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED)

SET(Boost_USE_STATIC_LIBS ON)
FIND_PACKAGE(Boost 1.54 COMPONENTS log REQUIRED)
#find_package(Boost 1.54 COMPONENTS filesystem REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})


#PCL Incldes
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS} )
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} )


add_executable( load load.cpp MathCalcs.cpp OcvManip.cpp PclManip.cpp WarpImage.cpp EquiTrans.cpp PointFeature.cpp PersCamera.cpp Triangle.cpp  )
#add_executable (trans trans_main.cpp MathCalcs.cpp OcvManip.cpp PclManip.cpp WarpImage.cpp EquiTrans.cpp PointFeature.cpp PersCamera.cpp Triangle.cpp)
#add_executable( test test.cpp )
target_link_libraries( load -lpthread ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
#target_link_libraries( trans -lpthread ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
#target_link_libraries( test ${PCL_LIBRARIES} ${OpenCV_LIBS} )
