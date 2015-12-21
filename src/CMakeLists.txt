# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
 

FIND_PACKAGE( Boost COMPONENTS thread REQUIRED)

ADD_DEFINITIONS(-DBOOST_THREAD_USE_LIB=ON)


LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib ${Boost_LIBRARY_DIR} )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include /usr/include/flycapture ${Boost_INCLUDE_DIR})

ADD_EXECUTABLE( cap Capture.cpp flycap.cpp ImageDisplay_OpenCV.cpp)

TARGET_LINK_LIBRARIES( cap ${OpenCV_LIBS} flycapture boost_system ${Boost_LIBRARIES} )
