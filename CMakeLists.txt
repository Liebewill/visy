cmake_minimum_required (VERSION 2.6)

project (visy)
cmake_policy(SET CMP0012 NEW)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

#Eigen
#include_directories(/usr/include/eigen3/Eigen) 


#OpenCV

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIRS})
#set(OPENCV_24_INCLUDE /Downloads/opencv-2.4.10/build/lib)
#set(OPENCV_24_LIBS /Downloads/opencv-2.4.10/include)
#include_directories(${OPENCV_24_INCLUDE})
#link_directories(${OPENCV_24_LIBS})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Bold
set(BOLD_INCLUDE_DIR /home/daniele/work/workspace_cpp/bold/include)
set(BOLD_LIBRARIES /home/daniele/work/workspace_cpp/bold/build/libmyboldlib.a)
include_directories (${BOLD_INCLUDE_DIR})     # make sure your .h all inside.

#Custom
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/extractors 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/detectors 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dataset 
    ${CMAKE_CURRENT_SOURCE_DIR})


add_library(visy_tools src/tools.cpp)
target_link_libraries (visy_tools ${PCL_LIBRARIES})
target_link_libraries (visy_tools ${OpenCV_LIBS})
target_link_libraries (visy_tools ${BOLD_LIBRARIES})


add_library(visy_extractors 
            src/extractors/KeyPoint3D.cpp
            src/extractors/Extractor.cpp
            src/extractors/Bold3DExtractor.cpp
            src/extractors/Utils.cpp
            )
target_link_libraries (visy_extractors ${PCL_LIBRARIES})
target_link_libraries (visy_extractors ${OpenCV_LIBS})
target_link_libraries (visy_extractors ${BOLD_LIBRARIES})
target_link_libraries (visy_extractors visy_tools)


add_library(visy_dataset 
    src/dataset/Dataset.cpp
    src/dataset/IrosDataset.cpp
)


add_executable(visy_testing src/testing.cpp)
target_link_libraries(
    visy_testing 
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_tools 
    visy_extractors
    visy_dataset
)

#
#add_executable(bunch_analyzer bunch_analyzer.cpp)
#target_link_libraries(bunch_analyzer ${OpenCV_LIBS} ${PCL_LIBRARIES} commons)
