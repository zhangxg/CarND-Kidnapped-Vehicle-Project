project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources 
#    14_Particle_Filter_CPP/src/initialize.cpp

#     11_Markov_Localization/src/code2/main.cpp
#     11_Markov_Localization/src/code2/help_functions.h
#     11_Markov_Localization/src/code2/map.h
#     11_Markov_Localization/src/code2/measurement_package.h
#     11_Markov_Localization/src/code2/bayesianFilter.h
#     11_Markov_Localization/src/code2/bayesianFilter.cpp

#     11_Markov_Localization/src/main.cpp
#     11_Markov_Localization/src/help_functions.h
#     11_Markov_Localization/src/map.h
#     11_Markov_Localization/src/measurement_package.h

    src/particle_filter.cpp 
#    src/test_main.cpp
    src/main.cpp
#   src/test_boot.cpp
    )


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter ${sources})


target_link_libraries(particle_filter z ssl uv uWS)

