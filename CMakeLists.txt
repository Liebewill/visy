cmake_minimum_required (VERSION 2.6)

project (visy)
cmake_policy(SET CMP0012 NEW)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

#OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#Boost
find_package(Boost)     
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

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
    ${CMAKE_CURRENT_SOURCE_DIR}/include/commons 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/extractors 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/descriptors
    ${CMAKE_CURRENT_SOURCE_DIR}/include/descriptors/dfunctions
    ${CMAKE_CURRENT_SOURCE_DIR}/include/detectors 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dataset 
    ${CMAKE_CURRENT_SOURCE_DIR})

#VISY COMMONS
add_library(visy_commons 
    src/commons/tools.cpp
    src/commons/Parameters.cpp
)
target_link_libraries (visy_commons ${PCL_LIBRARIES})
target_link_libraries (visy_commons ${OpenCV_LIBS})
target_link_libraries (visy_commons ${BOLD_LIBRARIES})

#VISY EXTRACTORS
add_library(visy_extractors 
            src/extractors/KeyPoint3D.cpp
            src/extractors/Extractor.cpp
            src/extractors/Bold3DExtractor.cpp
            src/extractors/SiftExtractor.cpp
            src/extractors/extrators_utils.cpp
            )
target_link_libraries (visy_extractors ${PCL_LIBRARIES})
target_link_libraries (visy_extractors ${OpenCV_LIBS})
target_link_libraries (visy_extractors ${BOLD_LIBRARIES})
target_link_libraries (visy_extractors visy_commons)

#VISY DESCRIPTORS
add_library(visy_descriptors
            src/descriptors/signatures/Signature.cpp
            src/descriptors/signatures/Histogram1D.cpp
            src/descriptors/signatures/HistogramND.cpp
            src/descriptors/Descriptor.cpp
            src/descriptors/Bold3DDescriptorMultiBunch.cpp
            src/descriptors/dfunctions/DFunction.cpp
            src/descriptors/dfunctions/DFunctionB2D.cpp
            src/descriptors/dfunctions/DFunctionB3DV1.cpp
            src/descriptors/dfunctions/DFunctionB3DV2.cpp
            src/descriptors/dfunctions/DFunctionB3DV1Multi.cpp
            src/descriptors/dfunctions/DFunctionB3DV2Multi.cpp
            src/descriptors/dfunctions/DFunctionFPFH.cpp
            src/descriptors/dfunctions/DFunctionFPFH2.cpp
            src/descriptors/dfunctions/DFunctionB3D4H.cpp
            src/descriptors/dfunctions/DFunctionB3DZ.cpp
            src/descriptors/dfunctions/DFunctionPPF.cpp
            src/descriptors/descriptors_utils.cpp
            )
target_link_libraries (visy_descriptors ${PCL_LIBRARIES})
target_link_libraries (visy_descriptors ${OpenCV_LIBS})
target_link_libraries (visy_descriptors ${BOLD_LIBRARIES})
target_link_libraries (visy_descriptors visy_commons)

#VISY DETECTORS
add_library(visy_detectors
            src/detectors/Detector.cpp
            src/detectors/Bold3DMDetector.cpp
            src/detectors/Bold3DM2Detector.cpp
            src/detectors/Bold3DM2MultiDetector.cpp
            src/detectors/Bold3DRDetector.cpp
            src/detectors/Bold3DR2Detector.cpp
            src/detectors/Bold3DR2MultiDetector.cpp
            src/detectors/BoldDetector.cpp
            src/detectors/Bold3DXDetector.cpp
            src/detectors/HybridDetector.cpp
            src/detectors/detectors_utils.cpp
            )
target_link_libraries (visy_detectors ${PCL_LIBRARIES})
target_link_libraries (visy_detectors ${OpenCV_LIBS})
target_link_libraries (visy_detectors ${BOLD_LIBRARIES})
target_link_libraries (visy_detectors visy_commons)
target_link_libraries (visy_detectors visy_extractors)
target_link_libraries (visy_detectors visy_descriptors)



#VISY DATASET
add_library(visy_dataset 
    src/dataset/Dataset.cpp
    src/dataset/IrosDataset.cpp
    src/dataset/TestData.cpp
)
target_link_libraries (visy_dataset ${PCL_LIBRARIES})
target_link_libraries (visy_dataset ${OpenCV_LIBS})
target_link_libraries (visy_dataset ${BOLD_LIBRARIES})
target_link_libraries (visy_dataset visy_commons)
target_link_libraries (visy_dataset visy_extractors)
target_link_libraries (visy_dataset visy_descriptors)
target_link_libraries (visy_dataset visy_detectors)

############ EXECUTABLES #########################

#VISY TESTING
add_executable(visy_testing src/testing.cpp)
target_link_libraries(
    visy_testing 
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#KEYPOINT TESTER
add_executable(keypoint_tester src/keypoint_tester.cpp)
target_link_libraries(
    keypoint_tester
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#MODEL MAKER
add_executable(model_maker src/model_maker.cpp)
target_link_libraries(
    model_maker
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#MODEL MAKER ALL
add_executable(model_maker_all src/model_maker_all.cpp)
target_link_libraries(
    model_maker_all
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#MATCHER
add_executable(match src/match.cpp)
target_link_libraries(
    match
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#MATCHER
add_executable(match_hv src/match_hv.cpp)
target_link_libraries(
    match_hv
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)

#MATCHER
add_executable(results src/results.cpp)
target_link_libraries(
    results
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)


#PLY
add_executable(ply_to_pcd src/ply_to_pcd.cpp)
target_link_libraries(
    ply_to_pcd
    ${PCL_LIBRARIES} 
    ${BOLD_LIBRARIES} 
    ${OpenCV_LIBS} 
    visy_commons 
    visy_extractors
    visy_descriptors
    visy_detectors
    visy_dataset
)


