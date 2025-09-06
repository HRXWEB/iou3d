# 示例程序的CMakeLists.txt

# 简单示例
add_executable(simple_example simple_example.cpp)
target_link_libraries(simple_example iou3d)

# 高级示例
add_executable(advanced_example advanced_example.cpp)
target_link_libraries(advanced_example iou3d)

# 如果找到数学库，也链接到示例
if(MATH_LIBRARY)
    target_link_libraries(simple_example ${MATH_LIBRARY})
    target_link_libraries(advanced_example ${MATH_LIBRARY})
endif()
