# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

#FIND_PACKAGE( Boost COMPONENTS thread REQUIRED)

#ADD_DEFINITIONS(-DBOOST_THREAD_USE_LIB=ON)


#LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include /usr/include/flycapture )

ADD_EXECUTABLE( cap Capture.cpp flycap.cpp)

TARGET_LINK_LIBRARIES( cap ${OpenCV_LIBS} flycapture boost_system )
