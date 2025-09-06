#include "iou3d.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace nms;

void testVertexOrder() {
    std::cout << "=== 测试顶点顺序对Sutherland-Hodgman算法的影响 ===" << std::endl;
    
    // 创建两个完全相同的正方形，应该完全重叠
    Box box1, box2;
    box1.center_x = 0; box1.center_y = 0; box1.center_z = 0;
    box1.length = 2; box1.width = 2; box1.height = 2;
    box1.yaw = 0;
    
    box2 = box1; // 完全相同
    
    std::cout << "\n1. 检查当前顶点顺序:" << std::endl;
    Polygon2D poly1 = boxToBEVPolygon(box1);
    std::cout << "Box1顶点序列:" << std::endl;
    for (size_t i = 0; i < poly1.size(); ++i) {
        std::cout << "  顶点" << i << ": (" << poly1[i].x << ", " << poly1[i].z << ")";
        
        // 判断相对位置
        if (poly1[i].x > 0 && poly1[i].z > 0) std::cout << " [右前]";
        else if (poly1[i].x < 0 && poly1[i].z > 0) std::cout << " [左前]";
        else if (poly1[i].x < 0 && poly1[i].z < 0) std::cout << " [左后]";
        else if (poly1[i].x > 0 && poly1[i].z < 0) std::cout << " [右后]";
        std::cout << std::endl;
    }
    
    // 计算顶点顺序（使用叉积判断）
    std::cout << "\n2. 判断顶点顺序（通过叉积）:" << std::endl;
    float total_cross_product = 0;
    for (size_t i = 0; i < poly1.size(); ++i) {
        size_t next = (i + 1) % poly1.size();
        float cross = poly1[i].x * poly1[next].z - poly1[i].z * poly1[next].x;
        total_cross_product += cross;
        std::cout << "  边" << i << "->边" << next << " 叉积: " << cross << std::endl;
    }
    
    std::cout << "总叉积: " << total_cross_product << std::endl;
    if (total_cross_product > 0) {
        std::cout << "结论: 顶点按逆时针排列" << std::endl;
    } else {
        std::cout << "结论: 顶点按顺时针排列" << std::endl;
    }
    
    // 测试裁剪算法
    std::cout << "\n3. 测试相同多边形的裁剪结果:" << std::endl;
    Polygon2D intersection = sutherlandHodgmanClip(poly1, poly1);
    std::cout << "自己裁剪自己的结果顶点数: " << intersection.size() << std::endl;
    
    if (intersection.size() == 4) {
        float area = calculatePolygonArea(intersection);
        float expected_area = 4.0f; // 2x2的正方形
        std::cout << "裁剪结果面积: " << area << ", 期望面积: " << expected_area << std::endl;
        
        if (std::abs(area - expected_area) < 1e-6f) {
            std::cout << "✓ 裁剪算法工作正常" << std::endl;
        } else {
            std::cout << "✗ 裁剪算法可能有问题！" << std::endl;
        }
    } else {
        std::cout << "✗ 裁剪算法严重错误：结果顶点数不正确！" << std::endl;
    }
}

void testDifferentOrders() {
    std::cout << "\n=== 测试不同顶点顺序的影响 ===" << std::endl;
    
    // 手动创建两个相同的正方形，但顺序不同
    Polygon2D poly_ccw; // 逆时针
    poly_ccw.push_back({1, 1});   // 右前
    poly_ccw.push_back({-1, 1});  // 左前
    poly_ccw.push_back({-1, -1}); // 左后
    poly_ccw.push_back({1, -1});  // 右后
    
    Polygon2D poly_cw; // 顺时针
    poly_cw.push_back({1, 1});   // 右前
    poly_cw.push_back({1, -1});  // 右后
    poly_cw.push_back({-1, -1}); // 左后
    poly_cw.push_back({-1, 1});  // 左前
    
    std::cout << "逆时针多边形面积: " << calculatePolygonArea(poly_ccw) << std::endl;
    std::cout << "顺时针多边形面积: " << calculatePolygonArea(poly_cw) << std::endl;
    
    // 测试裁剪
    Polygon2D result1 = sutherlandHodgmanClip(poly_ccw, poly_ccw);
    Polygon2D result2 = sutherlandHodgmanClip(poly_cw, poly_cw);
    
    std::cout << "逆时针自裁剪结果顶点数: " << result1.size() << std::endl;
    std::cout << "顺时针自裁剪结果顶点数: " << result2.size() << std::endl;
    
    if (result1.size() > 0) {
        std::cout << "逆时针自裁剪结果面积: " << calculatePolygonArea(result1) << std::endl;
    }
    if (result2.size() > 0) {
        std::cout << "顺时针自裁剪结果面积: " << calculatePolygonArea(result2) << std::endl;
    }
}

int main() {
    std::cout << "Sutherland-Hodgman算法顶点顺序测试" << std::endl;
    
    testVertexOrder();
    testDifferentOrders();
    
    return 0;
}
