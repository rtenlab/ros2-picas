find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_topic_cache test_topic_cache.cpp)
if(TARGET test_topic_cache)
    ament_target_dependencies(test_topic_cache)
    target_link_libraries(test_topic_cache ${PROJECT_NAME})
endif()

ament_add_gmock(test_security_logging test_security_logging.cpp)
if(TARGET test_security_logging)
    ament_target_dependencies(test_security_logging)
    target_link_libraries(test_security_logging ${PROJECT_NAME})
endif()
