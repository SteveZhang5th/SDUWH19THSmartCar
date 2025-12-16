cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR  "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/src"         )


link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )


set(SRC_LIST
        src/cap.c
        src/comm.c
        src/serial.cpp
        src/MyUart.cpp
        src/trvision.cpp
        src/MyMath.cpp
)
# add_executable(${PROJECT_NAME} ${SRC_LIST})
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})


#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(RUNIMAGES_NAME "RunImages")
set(RUNIMAGES_SOURCES ${SRC_LIST} RunImages.cpp)
add_executable(${RUNIMAGES_NAME} ${RUNIMAGES_SOURCES})
target_link_libraries(${RUNIMAGES_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${RUNIMAGES_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${RUNIMAGES_NAME}  ${OpenCV_LIBS})
target_link_libraries(${RUNIMAGES_NAME}  pthread )
target_link_libraries(${RUNIMAGES_NAME}  ${SERIAL_LIBRARIES})

set(STATICTEST_NAME "StaticTest")
set(STATICTEST_SOURCES ${SRC_LIST} StaticTest.cpp)
add_executable(${STATICTEST_NAME} ${STATICTEST_SOURCES})
target_link_libraries(${STATICTEST_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${STATICTEST_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${STATICTEST_NAME}  ${OpenCV_LIBS})
target_link_libraries(${STATICTEST_NAME}  pthread )
target_link_libraries(${STATICTEST_NAME}  ${SERIAL_LIBRARIES})

set(PICTURE_NAME "PictureCap")
set(PICTURE_SOURCES ${PROJECT_SOURCE_DIR} PictureCap.cpp)
add_executable(${PICTURE_NAME} ${PICTURE_SOURCES})
target_link_libraries(${PICTURE_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${PICTURE_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${PICTURE_NAME}  ${OpenCV_LIBS})
target_link_libraries(${PICTURE_NAME}  pthread )
target_link_libraries(${PICTURE_NAME}  ${SERIAL_LIBRARIES})

set(PTHREADCAP_NAME "pthreadcap")
set(PTHREADCAP_SOURCES ${SRC_LIST} pthreadcap.cpp)
add_executable(${PTHREADCAP_NAME} ${PTHREADCAP_SOURCES})
target_link_libraries(${PTHREADCAP_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${PTHREADCAP_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${PTHREADCAP_NAME}  ${OpenCV_LIBS})
target_link_libraries(${PTHREADCAP_NAME}  pthread )
target_link_libraries(${PTHREADCAP_NAME}  ${SERIAL_LIBRARIES})

set(MAINPRO_NAME "mainpro")
set(MAINPRO_SOURCES ${SRC_LIST} mainpro.cpp)
add_executable(${MAINPRO_NAME} ${MAINPRO_SOURCES})
target_link_libraries(${MAINPRO_NAME}  ${PPNC_LIBRARIES})
target_link_libraries(${MAINPRO_NAME}  ${ONNX_LIBRARIES})
target_link_libraries(${MAINPRO_NAME}  ${OpenCV_LIBS})
target_link_libraries(${MAINPRO_NAME}  pthread )
target_link_libraries(${MAINPRO_NAME}  ${SERIAL_LIBRARIES})
