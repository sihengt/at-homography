# Get that OpenCV
find_package(OpenCV REQUIRED)


set(TAG_DETECTOR
    include/homography/tagdetector.hpp
    include/homography/homography.hpp
    src/tagdetector.cpp
)

add_library(tag_detector_baselib SHARED ${TAG_DETECTOR})
target_include_directories(tag_detector_baselib PRIVATE third_party/yaml-cpp-0.6.0/include third_party/apriltags/include)
target_link_libraries(tag_detector_baselib PRIVATE yaml-cpp2 apriltags ${OpenCV_LIBS})

add_executable(homography src/homography.cpp)
target_include_directories(homography PRIVATE src include/homography third_party/yaml-cpp-0.6.0/include third_party/apriltags/include)
target_link_libraries(homography PRIVATE tag_detector_baselib yaml-cpp2 apriltags ${OpenCV_LIBS})

# Packaged third-party library: YAML-CPP
add_subdirectory(third_party/yaml-cpp-0.6.0)
add_subdirectory(third_party/apriltags)