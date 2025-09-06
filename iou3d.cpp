#include "iou3d.h"
#include <cmath>
#include <algorithm>
#include <cassert>

namespace nms {

Polygon2D boxToBEVPolygon(const Box& box) {
    // 在BEV视角（xoz平面）中，计算旋转后的4个顶点
    float half_width = box.width * 0.5f;   // x方向的一半
    float half_length = box.length * 0.5f; // z方向的一半
    
    float cos_yaw = std::cos(box.yaw);
    float sin_yaw = std::sin(box.yaw);
    
    Polygon2D polygon;
    polygon.reserve(4);
    
    // 定义相对于中心的4个顶点（未旋转）
    // 逆时针方向：右前、右后、左后、左前
    float vertices[4][2] = {
        {half_width, half_length},   // 右前
        {-half_width, half_length},  // 左前  
        {-half_width, -half_length}, // 左后
        {half_width, -half_length}   // 右后
    };
    
    // 应用旋转并平移到中心位置
    for (int i = 0; i < 4; ++i) {
        float local_x = vertices[i][0];
        float local_z = vertices[i][1];
        
        // 绕y轴旋转（从z轴绕向x轴为正向）
        float rotated_x = local_x * cos_yaw + local_z * sin_yaw;
        float rotated_z = -local_x * sin_yaw + local_z * cos_yaw;
        
        // 平移到世界坐标
        polygon.emplace_back(rotated_x + box.center_x, rotated_z + box.center_z);
    }
    
    return polygon;
}

float getLineIntersectionX(float x1, float z1, float x2, float z2,
                          float x3, float z3, float x4, float z4) {
    float numerator = (x1*z2 - z1*x2) * (x3-x4) - (x1-x2) * (x3*z4 - z3*x4);
    float denominator = (x1-x2) * (z3-z4) - (z1-z2) * (x3-x4);
    
    // 避免除零
    if (std::abs(denominator) < 1e-10f) {
        return 0.0f;
    }
    
    return numerator / denominator;
}

float getLineIntersectionZ(float x1, float z1, float x2, float z2,
                          float x3, float z3, float x4, float z4) {
    float numerator = (x1*z2 - z1*x2) * (z3-z4) - (z1-z2) * (x3*z4 - z3*x4);
    float denominator = (x1-x2) * (z3-z4) - (z1-z2) * (x3-x4);
    
    // 避免除零
    if (std::abs(denominator) < 1e-10f) {
        return 0.0f;
    }
    
    return numerator / denominator;
}

float calculatePolygonArea(const Polygon2D& polygon) {
    if (polygon.size() < 3) {
        return 0.0f;
    }
    
    float area = 0.0f;
    size_t n = polygon.size();
    
    // 使用鞋带公式
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += polygon[i].x * polygon[j].z - polygon[j].x * polygon[i].z;
    }
    
    return std::abs(area) * 0.5f;
}

Polygon2D clipPolygonByLine(const Polygon2D& polygon, 
                           float x1, float z1, float x2, float z2) {
    if (polygon.empty()) {
        return Polygon2D();
    }
    
    Polygon2D clipped;
    
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t curr_i = i;
        size_t next_i = (i + 1) % polygon.size();
        
        float curr_x = polygon[curr_i].x;
        float curr_z = polygon[curr_i].z;
        float next_x = polygon[next_i].x;
        float next_z = polygon[next_i].z;
        
        // 计算点相对于裁剪线的位置
        // 使用叉积判断点在线的哪一侧（左侧为内侧）
        float curr_side = (x2 - x1) * (curr_z - z1) - (z2 - z1) * (curr_x - x1);
        float next_side = (x2 - x1) * (next_z - z1) - (z2 - z1) * (next_x - x1);
        
        // 当前点是否在内侧
        bool curr_inside = curr_side >= 0;
        bool next_inside = next_side >= 0;
        
        if (curr_inside && next_inside) {
            // 案例1：两个点都在内侧，添加next点
            clipped.emplace_back(next_x, next_z);
        } else if (curr_inside && !next_inside) {
            // 案例2：当前点在内侧，下一个点在外侧，添加交点
            float intersect_x = getLineIntersectionX(curr_x, curr_z, next_x, next_z, x1, z1, x2, z2);
            float intersect_z = getLineIntersectionZ(curr_x, curr_z, next_x, next_z, x1, z1, x2, z2);
            clipped.emplace_back(intersect_x, intersect_z);
        } else if (!curr_inside && next_inside) {
            // 案例3：当前点在外侧，下一个点在内侧，添加交点和next点
            float intersect_x = getLineIntersectionX(curr_x, curr_z, next_x, next_z, x1, z1, x2, z2);
            float intersect_z = getLineIntersectionZ(curr_x, curr_z, next_x, next_z, x1, z1, x2, z2);
            clipped.emplace_back(intersect_x, intersect_z);
            clipped.emplace_back(next_x, next_z);
        }
        // 案例4：两个点都在外侧，不添加任何点
    }
    
    return clipped;
}

Polygon2D sutherlandHodgmanClip(const Polygon2D& subject, const Polygon2D& clipper) {
    if (subject.empty() || clipper.empty()) {
        return Polygon2D();
    }
    
    Polygon2D clipped = subject;
    
    // 对每条裁剪边进行裁剪
    for (size_t i = 0; i < clipper.size(); ++i) {
        size_t next_i = (i + 1) % clipper.size();
        
        clipped = clipPolygonByLine(clipped, 
                                   clipper[i].x, clipper[i].z,
                                   clipper[next_i].x, clipper[next_i].z);
        
        if (clipped.empty()) {
            break;
        }
    }
    
    return clipped;
}

float calculateBEVIoU(const Box& box1, const Box& box2) {
    // 将3D包围盒投影到BEV平面
    Polygon2D poly1 = boxToBEVPolygon(box1);
    Polygon2D poly2 = boxToBEVPolygon(box2);
    
    // 计算两个多边形的交集
    Polygon2D intersection = sutherlandHodgmanClip(poly1, poly2);
    
    // 计算面积
    float area1 = calculatePolygonArea(poly1);
    float area2 = calculatePolygonArea(poly2);
    float area_intersection = calculatePolygonArea(intersection);
    
    // 计算并集面积
    float area_union = area1 + area2 - area_intersection;
    
    // 避免除零
    if (area_union < 1e-10f) {
        return 0.0f;
    }
    
    return area_intersection / area_union;
}

float calculateIoU3D(const Box& box1, const Box& box2) {
    // 计算BEV平面的交集面积
    Polygon2D poly1 = boxToBEVPolygon(box1);
    Polygon2D poly2 = boxToBEVPolygon(box2);
    Polygon2D intersection_poly = sutherlandHodgmanClip(poly1, poly2);
    float intersection_area = calculatePolygonArea(intersection_poly);
    
    if (intersection_area < 1e-10f) {
        return 0.0f; // BEV平面没有交集，3D IoU为0
    }
    
    // 计算y轴方向的重叠
    float box1_y_min = box1.center_y - box1.height * 0.5f;
    float box1_y_max = box1.center_y + box1.height * 0.5f;
    float box2_y_min = box2.center_y - box2.height * 0.5f;
    float box2_y_max = box2.center_y + box2.height * 0.5f;
    
    // 计算y轴方向的交集范围
    float y_intersection_min = std::max(box1_y_min, box2_y_min);
    float y_intersection_max = std::min(box1_y_max, box2_y_max);
    
    if (y_intersection_min >= y_intersection_max) {
        return 0.0f; // y轴方向没有重叠
    }
    
    float y_intersection_height = y_intersection_max - y_intersection_min;
    
    // 计算3D交集体积
    float intersection_volume = intersection_area * y_intersection_height;
    
    // 计算两个包围盒的体积
    float volume1 = calculatePolygonArea(poly1) * box1.height;
    float volume2 = calculatePolygonArea(poly2) * box2.height;
    
    // 计算并集体积
    float union_volume = volume1 + volume2 - intersection_volume;
    
    // 避免除零
    if (union_volume < 1e-10f) {
        return 0.0f;
    }
    
    return intersection_volume / union_volume;
}

} // namespace nms
