#include "iou3d.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace nms;

void testRotationDirection() {
    std::cout << "=== 测试旋转方向 ===" << std::endl;
    
    // 创建一个矩形框，长度大于宽度，便于观察旋转
    Box box;
    box.center_x = 0.0f;
    box.center_y = 0.0f;
    box.center_z = 0.0f;
    box.width = 1.0f;   // x方向
    box.length = 3.0f;  // z方向
    box.height = 1.0f;
    box.yaw = 0.0f;
    
    std::cout << "原始框 (width=1, length=3):" << std::endl;
    Polygon2D poly0 = boxToBEVPolygon(box);
    for (size_t i = 0; i < poly0.size(); ++i) {
        std::cout << "  顶点" << i << ": (" << poly0[i].x << ", " << poly0[i].z << ")" << std::endl;
    }
    
    // 测试90度旋转（π/2）- 从z轴绕向x轴
    box.yaw = M_PI / 2;
    std::cout << "\n旋转90度后 (yaw = π/2):" << std::endl;
    Polygon2D poly90 = boxToBEVPolygon(box);
    for (size_t i = 0; i < poly90.size(); ++i) {
        std::cout << "  顶点" << i << ": (" << std::fixed << std::setprecision(3) 
                  << poly90[i].x << ", " << poly90[i].z << ")" << std::endl;
    }
    
    std::cout << "\n期望结果：原来沿z轴的长边(3)现在应该沿x轴方向" << std::endl;
    std::cout << "验证：检查x方向的范围应该约为3，z方向的范围应该约为1" << std::endl;
    
    float x_min = poly90[0].x, x_max = poly90[0].x;
    float z_min = poly90[0].z, z_max = poly90[0].z;
    
    for (size_t i = 1; i < poly90.size(); ++i) {
        x_min = std::min(x_min, poly90[i].x);
        x_max = std::max(x_max, poly90[i].x);
        z_min = std::min(z_min, poly90[i].z);
        z_max = std::max(z_max, poly90[i].z);
    }
    
    float x_range = x_max - x_min;
    float z_range = z_max - z_min;
    
    std::cout << "实际结果：x方向范围 = " << x_range << ", z方向范围 = " << z_range << std::endl;
    
    if (std::abs(x_range - 3.0f) < 0.1f && std::abs(z_range - 1.0f) < 0.1f) {
        std::cout << "✓ 旋转方向正确！" << std::endl;
    } else {
        std::cout << "✗ 旋转方向可能有问题！" << std::endl;
    }
}

void testSpecificAngles() {
    std::cout << "\n=== 测试特定角度的旋转 ===" << std::endl;
    
    Box box;
    box.center_x = 0.0f;
    box.center_y = 0.0f;
    box.center_z = 0.0f;
    box.width = 2.0f;
    box.length = 2.0f;
    box.height = 1.0f;
    
    // 测试几个关键角度
    float angles[] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI};
    const char* names[] = {"0°", "45°", "90°", "135°", "180°"};
    
    for (int i = 0; i < 5; ++i) {
        box.yaw = angles[i];
        Polygon2D poly = boxToBEVPolygon(box);
        
        std::cout << "\n角度 " << names[i] << " (yaw=" << angles[i] << "):" << std::endl;
        for (size_t j = 0; j < poly.size(); ++j) {
            std::cout << "  (" << std::fixed << std::setprecision(3) 
                      << poly[j].x << ", " << poly[j].z << ")";
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "相机坐标系旋转测试" << std::endl;
    std::cout << "坐标系定义：x-右，y-下，z-前" << std::endl;
    std::cout << "yaw角定义：从z轴绕向x轴为正向" << std::endl;
    
    testRotationDirection();
    testSpecificAngles();
    
    return 0;
}
