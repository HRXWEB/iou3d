# 相机坐标系3D IoU计算库

这是一个专门为相机坐标系设计的3D包围盒IoU计算库，实现了基于Sutherland-Hodgman算法的精确多边形裁剪。

## 坐标系定义

- **x轴**：水平右方向（宽度）
- **y轴**：垂直向下方向（高度）  
- **z轴**：深度方向（长度）
- **BEV视角**：xoz平面（从上往下看）
- **yaw角**：从z轴绕向x轴为正向（绕y轴顺时针旋转）

## 核心功能

- `calculateIoU3D()` - 计算两个3D包围盒的3D IoU
- `calculateBEVIoU()` - 计算两个3D包围盒在BEV平面的IoU
- 支持任意角度的yaw旋转
- 使用Sutherland-Hodgman多边形裁剪算法处理复杂重叠情况

## 文件结构

```
iou3d/
├── iou3d.h                    # 头文件
├── iou3d.cpp                  # 实现文件
├── test_iou3d.cpp             # 主测试套件
├── rotation_test.cpp          # 旋转验证测试
├── CMakeLists.txt             # CMake主配置文件
├── cmake/                     # CMake配置文件
│   └── iou3dConfig.cmake.in
├── examples/                  # 示例代码
│   ├── CMakeLists.txt
│   ├── simple_example.cpp     # 简单使用示例
│   └── advanced_example.cpp   # 高级功能示例
└── README.md                  # 说明文档
```

## 编译和使用

### CMake方式（推荐）

```bash
# 创建构建目录
mkdir build && cd build

# 配置项目
cmake ..

# 编译
make

# 运行测试
make test
# 或者
ctest

# 单独运行特定测试
./test_iou3d           # 主测试套件
./rotation_test        # 旋转验证测试

# 运行示例（如果启用）
cmake .. -DBUILD_EXAMPLES=ON
make
./examples/simple_example
./examples/advanced_example

# 安装库（可选）
sudo make install
```

### 高级CMake选项

```bash
# Debug模式构建
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Release模式构建
cmake .. -DCMAKE_BUILD_TYPE=Release

# 禁用测试构建
cmake .. -DBUILD_TESTS=OFF

# 启用示例构建
cmake .. -DBUILD_EXAMPLES=ON

# 指定安装前缀
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
```


### 在其他项目中使用

如果已安装库：

```cmake
# 在你的CMakeLists.txt中
find_package(iou3d REQUIRED)
target_link_libraries(your_target iou3d::iou3d)
```

如果作为子项目：

```cmake
# 添加子目录
add_subdirectory(path/to/iou3d)
target_link_libraries(your_target iou3d)
```

## 使用示例

```cpp
#include "iou3d.h"
using namespace nms;

// 创建两个包围盒
Box box1;
box1.center_x = 0; box1.center_y = 0; box1.center_z = 0;
box1.width = 2; box1.length = 4; box1.height = 3;
box1.yaw = 0;

Box box2;  
box2.center_x = 1; box2.center_y = 0; box2.center_z = 0;
box2.width = 2; box2.length = 4; box2.height = 3;
box2.yaw = 0;

// 计算IoU
float iou_3d = calculateIoU3D(box1, box2);
float iou_bev = calculateBEVIoU(box1, box2);
```

## 测试验证

### 主测试套件 (`test_iou3d`)

包含以下几种典型场景：

1. **完全重叠** - IoU = 1.0
2. **完全不重叠** - IoU = 0.0  
3. **部分重叠** - 验证精确的数值计算
4. **BEV重叠但高度分离** - 3D IoU = 0, BEV IoU > 0
5. **旋转包围盒** - 验证Sutherland-Hodgman算法
6. **不同大小包围盒** - 验证包含关系

### 旋转验证测试 (`rotation_test`)

专门验证相机坐标系下的旋转计算：

- **旋转方向验证**：测试90度旋转后的几何变换
- **多角度测试**：验证0°、45°、90°、135°、180°的旋转结果
- **坐标系确认**：确保yaw角"从z轴绕向x轴为正向"的定义正确

```bash
# 运行旋转测试
cd build
./rotation_test
```

所有测试用例都通过了数学验证，确保实现的正确性。
