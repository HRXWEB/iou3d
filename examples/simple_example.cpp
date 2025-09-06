#include "iou3d.h"
#include <iostream>
#include <iomanip>

using namespace nms;

int main() {
    std::cout << "=== 简单的IoU3D使用示例 ===" << std::endl;
    
    // 创建两个包围盒
    Box box1;
    box1.class_name = "car";
    box1.class_id = 1;
    box1.center_x = 0.0f;
    box1.center_y = 0.0f;
    box1.center_z = 0.0f;
    box1.width = 2.0f;
    box1.length = 4.0f;
    box1.height = 1.5f;
    box1.yaw = 0.0f;
    box1.confidence = 0.9f;
    
    Box box2;
    box2.class_name = "car";
    box2.class_id = 1;
    box2.center_x = 1.0f;  // 沿x轴偏移1米
    box2.center_y = 0.0f;
    box2.center_z = 0.0f;
    box2.width = 2.0f;
    box2.length = 4.0f;
    box2.height = 1.5f;
    box2.yaw = 0.0f;
    box2.confidence = 0.8f;
    
    std::cout << "\n包围盒1: " << box1 << std::endl;
    std::cout << "包围盒2: " << box2 << std::endl;
    
    // 计算IoU
    float iou_3d = calculateIoU3D(box1, box2);
    float iou_bev = calculateBEVIoU(box1, box2);
    
    std::cout << "\n计算结果:" << std::endl;
    std::cout << "3D IoU: " << std::fixed << std::setprecision(6) << iou_3d << std::endl;
    std::cout << "BEV IoU: " << std::fixed << std::setprecision(6) << iou_bev << std::endl;
    
    // 解释结果
    if (iou_3d > 0.5f) {
        std::cout << "\n✓ 高重叠度：两个包围盒有很大重叠" << std::endl;
    } else if (iou_3d > 0.1f) {
        std::cout << "\n⚠ 中等重叠度：两个包围盒有部分重叠" << std::endl;
    } else {
        std::cout << "\n✗ 低重叠度：两个包围盒几乎不重叠" << std::endl;
    }
    
    return 0;
}
