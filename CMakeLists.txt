project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h src/sensor_data.cpp src/sensor_data.h)

# gtest (commenting out for submission)
#set(test_sources test/gtest_main.cpp test/FusionEKF_test.cpp test/kalman_filter_test.cpp test/tools_test.cpp test/util.cpp test/util.h test/util_test.cpp test/sensor_data_test.cpp src/tools.cpp src/tools.h src/kalman_filter.cpp src/kalman_filter.h src/FusionEKF.cpp src/FusionEKF.h src/sensor_data.cpp src/sensor_data.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(src)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.19.2/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtendedKF ${sources})

# gtest (commenting out for submission)
#add_executable(Test_ExtendedKF ${test_sources})
#set_target_properties(Test_ExtendedKF PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(ExtendedKF z ssl uv uWS)

# gtest (commenting out for submission)
#target_link_libraries(Test_ExtendedKF gtest gmock z ssl uv uWS)
