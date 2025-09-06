cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(iou3d VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 项目源文件
set(SOURCES
    iou3d.cpp
)

set(HEADERS
    iou3d.h
)

# 创建静态库
add_library(iou3d STATIC ${SOURCES} ${HEADERS})

# 设置包含目录
target_include_directories(iou3d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# 为库设置属性
set_target_properties(iou3d PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "iou3d.h"
)

# 如果需要数学库（某些系统需要）
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(iou3d ${MATH_LIBRARY})
endif()

# 选项：是否构建测试
option(BUILD_TESTS "Build test executable" ON)

if(BUILD_TESTS)
    # 创建主测试可执行文件
    add_executable(test_iou3d test_iou3d.cpp)
    
    # 链接到iou3d库
    target_link_libraries(test_iou3d iou3d)
    
    # 如果找到数学库，也链接到测试
    if(MATH_LIBRARY)
        target_link_libraries(test_iou3d ${MATH_LIBRARY})
    endif()
    
    # 创建旋转测试可执行文件
    add_executable(rotation_test rotation_test.cpp)
    target_link_libraries(rotation_test iou3d)
    if(MATH_LIBRARY)
        target_link_libraries(rotation_test ${MATH_LIBRARY})
    endif()
    
    # 添加测试目标
    enable_testing()
    add_test(NAME iou3d_test COMMAND test_iou3d)
    add_test(NAME rotation_test COMMAND rotation_test)
    
    # 设置测试属性
    set_tests_properties(iou3d_test PROPERTIES
        PASS_REGULAR_EXPRESSION "🎉 所有测试用例通过！"
    )
    set_tests_properties(rotation_test PROPERTIES
        PASS_REGULAR_EXPRESSION "✓ 旋转方向正确！"
    )
endif()

# 选项：是否构建示例
option(BUILD_EXAMPLES "Build example programs" OFF)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# 安装配置
include(GNUInstallDirs)

# 安装库文件
install(TARGETS iou3d
    EXPORT iou3dTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 安装导出文件
install(EXPORT iou3dTargets
    FILE iou3dTargets.cmake
    NAMESPACE iou3d::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iou3d
)

# 创建配置文件
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/iou3dConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/iou3dConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iou3d
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/iou3dConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# 安装配置文件
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/iou3dConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/iou3dConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iou3d
)

# 打印配置信息
message(STATUS "")
message(STATUS "=================== IoU3D Configuration ===================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build tests: ${BUILD_TESTS}")
message(STATUS "Build examples: ${BUILD_EXAMPLES}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "==========================================================")
message(STATUS "")
