set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

catkin_add_gtest(${PROJECT_NAME}_utest test.cpp)
target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

catkin_add_nosetests(test.py)
