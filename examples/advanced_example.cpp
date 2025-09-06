#include "iou3d.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace nms;

// 辅助函数：创建包围盒
Box createBox(const std::string& name, float x, float y, float z, 
              float w, float l, float h, float yaw = 0.0f) {
    Box box;
    box.class_name = name;
    box.class_id = 1;
    box.center_x = x;
    box.center_y = y;
    box.center_z = z;
    box.width = w;
    box.length = l;
    box.height = h;
    box.yaw = yaw;
    box.confidence = 1.0f;
    return box;
}

// 计算IoU矩阵
void calculateIoUMatrix(const std::vector<Box>& boxes) {
    size_t n = boxes.size();
    std::cout << "\n=== IoU矩阵计算 ===" << std::endl;
    std::cout << "包围盒数量: " << n << std::endl;
    
    // 打印包围盒信息
    for (size_t i = 0; i < n; ++i) {
        std::cout << "Box" << i << ": 中心(" << boxes[i].center_x << "," 
                  << boxes[i].center_y << "," << boxes[i].center_z 
                  << ") 尺寸(" << boxes[i].width << "," << boxes[i].length 
                  << "," << boxes[i].height << ") yaw=" << boxes[i].yaw << std::endl;
    }
    
    // 计算IoU矩阵
    std::cout << "\n3D IoU矩阵:" << std::endl;
    std::cout << std::setw(8) << "";
    for (size_t i = 0; i < n; ++i) {
        std::cout << std::setw(8) << ("Box" + std::to_string(i));
    }
    std::cout << std::endl;
    
    for (size_t i = 0; i < n; ++i) {
        std::cout << std::setw(8) << ("Box" + std::to_string(i));
        for (size_t j = 0; j < n; ++j) {
            float iou = calculateIoU3D(boxes[i], boxes[j]);
            std::cout << std::setw(8) << std::fixed << std::setprecision(3) << iou;
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nBEV IoU矩阵:" << std::endl;
    std::cout << std::setw(8) << "";
    for (size_t i = 0; i < n; ++i) {
        std::cout << std::setw(8) << ("Box" + std::to_string(i));
    }
    std::cout << std::endl;
    
    for (size_t i = 0; i < n; ++i) {
        std::cout << std::setw(8) << ("Box" + std::to_string(i));
        for (size_t j = 0; j < n; ++j) {
            float iou = calculateBEVIoU(boxes[i], boxes[j]);
            std::cout << std::setw(8) << std::fixed << std::setprecision(3) << iou;
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "=== 高级IoU3D使用示例 ===" << std::endl;
    
    // 场景1：多个车辆的IoU矩阵
    std::cout << "\n【场景1：停车场中的多辆车】" << std::endl;
    std::vector<Box> cars = {
        createBox("car", 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, 0.0f),        // 正常停放
        createBox("car", 1.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, 0.0f),        // 部分重叠
        createBox("car", 5.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, M_PI/4),      // 远离+旋转45度
        createBox("car", 0.0f, 2.0f, 0.0f, 2.0f, 4.0f, 1.5f, 0.0f)         // 高度分离
    };
    
    calculateIoUMatrix(cars);
    
    // 场景2：不同尺寸物体
    std::cout << "\n【场景2：不同尺寸的物体】" << std::endl;
    std::vector<Box> objects = {
        createBox("truck", 0.0f, 0.0f, 0.0f, 3.0f, 8.0f, 2.5f, 0.0f),      // 大卡车
        createBox("car", 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, 0.0f),        // 小轿车（包含在卡车内）
        createBox("bike", 0.0f, 0.0f, 0.0f, 0.8f, 1.8f, 1.2f, 0.0f)        // 自行车（包含在轿车内）
    };
    
    calculateIoUMatrix(objects);
    
    // 场景3：旋转测试
    std::cout << "\n【场景3：旋转角度对IoU的影响】" << std::endl;
    Box ref_box = createBox("ref", 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, 0.0f);
    
    std::cout << "参考框与不同旋转角度框的IoU:" << std::endl;
    std::cout << std::setw(12) << "角度(度)" << std::setw(12) << "3D IoU" << std::setw(12) << "BEV IoU" << std::endl;
    std::cout << std::string(36, '-') << std::endl;
    
    for (float angle_deg = 0.0f; angle_deg <= 90.0f; angle_deg += 15.0f) {
        float angle_rad = angle_deg * M_PI / 180.0f;
        Box rotated_box = createBox("rotated", 0.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.5f, angle_rad);
        
        float iou_3d = calculateIoU3D(ref_box, rotated_box);
        float iou_bev = calculateBEVIoU(ref_box, rotated_box);
        
        std::cout << std::setw(12) << std::fixed << std::setprecision(1) << angle_deg
                  << std::setw(12) << std::fixed << std::setprecision(6) << iou_3d
                  << std::setw(12) << std::fixed << std::setprecision(6) << iou_bev
                  << std::endl;
    }
    
    std::cout << "\n=== 示例运行完成 ===" << std::endl;
    return 0;
}
