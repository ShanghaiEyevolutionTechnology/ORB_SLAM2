
find_package(EvoBinoSDK REQUIRED)

include_directories(${EvoBinoSDK_INCLUDE_DIRS})

add_executable(leadsense leadsense.cpp)
target_link_libraries(leadsense ${STEREOCAMERA_LIBS} ${PROJECT_NAME})
