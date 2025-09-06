#include "iou3d.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>

using namespace nms;

// 辅助函数：比较浮点数相等
bool isEqual(float a, float b, float epsilon = 1e-6f) {
    return std::abs(a - b) < epsilon;
}

// 辅助函数：创建测试用的Box
Box createBox(float center_x, float center_y, float center_z,
              float width, float length, float height, 
              float yaw = 0.0f, std::string class_name = "test") {
    Box box;
    box.class_name = class_name;
    box.class_id = 0;
    box.center_x = center_x;
    box.center_y = center_y;
    box.center_z = center_z;
    box.width = width;
    box.length = length;
    box.height = height;
    box.yaw = yaw;
    box.confidence = 1.0f;
    return box;
}

void testCase1_IdenticalBoxes() {
    std::cout << "\n=== 测试用例1: 完全重叠的两个盒子 ===" << std::endl;
    
    Box box1 = createBox(0, 0, 0, 2, 4, 3);
    Box box2 = createBox(0, 0, 0, 2, 4, 3);
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    assert(isEqual(iou, 1.0f));
    assert(isEqual(bev_iou, 1.0f));
    std::cout << "✓ 测试通过：完全重叠的IoU应该为1.0" << std::endl;
}

void testCase2_NoOverlap() {
    std::cout << "\n=== 测试用例2: 完全不重叠的两个盒子 ===" << std::endl;
    
    Box box1 = createBox(0, 0, 0, 2, 2, 2);
    Box box2 = createBox(10, 0, 0, 2, 2, 2);  // x方向远离
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    assert(isEqual(iou, 0.0f));
    assert(isEqual(bev_iou, 0.0f));
    std::cout << "✓ 测试通过：不重叠的IoU应该为0.0" << std::endl;
}

void testCase3_PartialOverlap() {
    std::cout << "\n=== 测试用例3: 部分重叠的两个盒子 ===" << std::endl;
    
    // 两个相同大小的正方体，沿x轴移动一半距离
    Box box1 = createBox(0, 0, 0, 2, 2, 2);  // 中心在(0,0,0)，边长为2
    Box box2 = createBox(1, 0, 0, 2, 2, 2);  // 中心在(1,0,0)，边长为2
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    // 理论计算：
    // Box1: x[-1,1], y[-1,1], z[-1,1]
    // Box2: x[0,2], y[-1,1], z[-1,1]
    // 交集: x[0,1], y[-1,1], z[-1,1] = 1*2*2 = 4
    // 并集: 2*2*2 + 2*2*2 - 4 = 8 + 8 - 4 = 12
    // IoU = 4/12 = 1/3 ≈ 0.333333
    
    float expected_iou = 1.0f / 3.0f;
    assert(isEqual(iou, expected_iou, 1e-5f));
    assert(isEqual(bev_iou, expected_iou, 1e-5f));
    std::cout << "✓ 测试通过：预期IoU = " << expected_iou << std::endl;
}

void testCase4_HeightNoOverlap() {
    std::cout << "\n=== 测试用例4: BEV重叠但高度不重叠 ===" << std::endl;
    
    Box box1 = createBox(0, 0, 0, 2, 2, 2);  // y范围[-1, 1]
    Box box2 = createBox(0, 3, 0, 2, 2, 2);  // y范围[2, 4]
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    assert(isEqual(iou, 0.0f));  // 3D不重叠
    assert(isEqual(bev_iou, 1.0f));  // BEV完全重叠
    std::cout << "✓ 测试通过：高度不重叠时3D IoU=0，BEV IoU=1" << std::endl;
}

void testCase5_RotatedBoxes() {
    std::cout << "\n=== 测试用例5: 旋转的盒子 ===" << std::endl;
    
    // 一个正方形不旋转，另一个旋转45度
    Box box1 = createBox(0, 0, 0, 2, 2, 2, 0);  // 不旋转
    Box box2 = createBox(0, 0, 0, 2, 2, 2, M_PI/4);  // 旋转45度
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    // 对于2x2的正方形旋转45度，IoU应该小于1但大于0
    assert(iou > 0.0f && iou < 1.0f);
    assert(bev_iou > 0.0f && bev_iou < 1.0f);
    std::cout << "✓ 测试通过：旋转正方形的IoU在(0,1)范围内" << std::endl;
}

void testCase6_DifferentSizes() {
    std::cout << "\n=== 测试用例6: 不同大小的盒子 ===" << std::endl;
    
    Box box1 = createBox(0, 0, 0, 4, 4, 4);  // 大盒子
    Box box2 = createBox(0, 0, 0, 2, 2, 2);  // 小盒子，完全包含在大盒子内
    
    float iou = calculateIoU3D(box1, box2);
    float bev_iou = calculateBEVIoU(box1, box2);
    
    std::cout << "Box1: " << box1 << std::endl;
    std::cout << "Box2: " << box2 << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << bev_iou << std::endl;
    
    // 理论计算：
    // 3D IoU: 小盒子体积 = 2*2*2 = 8, 大盒子体积 = 4*4*4 = 64
    // 交集体积 = 8 (小盒子完全在大盒子内), 并集体积 = 64
    // 3D IoU = 8/64 = 1/8 = 0.125
    //
    // BEV IoU: 小盒子面积 = 2*2 = 4, 大盒子面积 = 4*4 = 16  
    // 交集面积 = 4, 并集面积 = 16
    // BEV IoU = 4/16 = 1/4 = 0.25
    
    float expected_3d_iou = 1.0f / 8.0f;   // 0.125
    float expected_bev_iou = 1.0f / 4.0f;  // 0.25
    assert(isEqual(iou, expected_3d_iou, 1e-5f));
    assert(isEqual(bev_iou, expected_bev_iou, 1e-5f));
    std::cout << "✓ 测试通过：预期3D IoU = " << expected_3d_iou << ", 预期BEV IoU = " << expected_bev_iou << std::endl;
}

void testPolygonArea() {
    std::cout << "\n=== 测试多边形面积计算 ===" << std::endl;
    
    // 测试简单矩形
    Polygon2D rect = {{0, 0}, {2, 0}, {2, 1}, {0, 1}};
    float area = calculatePolygonArea(rect);
    std::cout << "矩形面积 (2x1): " << area << std::endl;
    assert(isEqual(area, 2.0f));
    
    // 测试单位正方形
    Polygon2D square = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    area = calculatePolygonArea(square);
    std::cout << "正方形面积 (1x1): " << area << std::endl;
    assert(isEqual(area, 1.0f));
    
    std::cout << "✓ 多边形面积计算测试通过" << std::endl;
}

int main() {
    std::cout << "开始3D IoU测试..." << std::endl;
    
    try {
        testPolygonArea();
        testCase1_IdenticalBoxes();
        testCase2_NoOverlap();
        testCase3_PartialOverlap();
        testCase4_HeightNoOverlap();
        testCase5_RotatedBoxes();
        testCase6_DifferentSizes();
        
        std::cout << "\n🎉 所有测试用例通过！" << std::endl;
        std::cout << "3D IoU实现验证成功。" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ 测试失败: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
