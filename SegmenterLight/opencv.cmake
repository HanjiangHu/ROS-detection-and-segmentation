# (with opencv-2.2 ) FIND_PACKAGE( OpenCV REQUIRED )
# export OpenCV_DIR=/opt/ros/diamondback/stacks/vision_opencv/opencv2/opencv/share/opencv
FIND_PACKAGE( OpenCV REQUIRED )
SET( OPENCV_LIBRARIES ${OpenCV_LIBS})
SET( OPENCV_INCLUDE_PATH ${OpenCV_INCLUDE_DIRS})

include(LibFindMacros.cmake) # additional macros needed to read the pkconfig stuff into cmake
libfind_pkg_check_modules(OpenCV_PKGCONF opencv)
SET( OPENCV_LIBRARIES ${OpenCV_LIBRARIES})

# Include dir
find_path(OpenCV_INCLUDE_DIRS
  NAMES opencv.h
  PATHS ${OpenCV_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(OpenCV_LIBRARY  
  PATHS ${OpenCV_PKGCONF_LIBRARY_DIRS}
)

SET( OPENCV_LIBRARIES ${OpenCV_LIBS})
SET( OPENCV_INCLUDE_PATH ${OpenCV_INCLUDE_DIRS})

MESSAGE( STATUS "OPENCV_LIBRARIES_DIRS: ${OpenCV_PKGCONF_LIBRARY_DIRS}")
MESSAGE( STATUS "OPENCV_INCLUDE_PATH: ${OPENCV_INCLUDE_PATH}")
MESSAGE( STATUS "OPENCV_LIBRARIES   : ${OPENCV_LIBRARIES}")


LINK_DIRECTORIES(${OpenCV_PKGCONF_LIBRARY_DIRS})
include_directories(${OPENCV_INCLUDE_PATH})
