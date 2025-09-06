#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace nms {

/**
 * @brief 相机坐标系下的3D包围盒
 * 坐标系定义：
 * - x轴：水平右方向
 * - y轴：垂直向下方向（高度）
 * - z轴：深度方向（向前）
 * - yaw角：从z轴绕向x轴为正向（绕y轴顺时针）
 * BEV视角：xoz平面（从上往下看）
 */
struct Box {
    // the class name of the box
    std::string class_name;
    // the class id of the box
    int class_id;
    // the center point of the box in camera coordinate
    float center_x;
    float center_y;
    float center_z;
    // the x dimension of the box in camera coordinate
    float length;
    // the z dimension of the box in camera coordinate
    float width;
    // the y dimension of the box in camera coordinate
    float height;
    // 绕y轴旋转的角度（从z轴绕向x轴为正向）
    float yaw;
    // the confidence of the box
    float confidence;

    friend std::ostream& operator<<(std::ostream& os, const Box& box) {
        os << "class_name: " << box.class_name << ", class_id: " << box.class_id
           << ", center_x: " << box.center_x << ", center_y: " << box.center_y << ", center_z: " << box.center_z
           << ", width: " << box.width << ", length: " << box.length << ", height: " << box.height
           << ", yaw: " << box.yaw << ", confidence: " << box.confidence;
        return os;
    }
};

/**
 * @brief 2D点结构体，用于BEV平面（xoz）
 */
struct Point2D {
    float x;
    float z;  // 注意：这里是z坐标，不是y
    
    Point2D(float x = 0.0f, float z = 0.0f) : x(x), z(z) {}
};

/**
 * @brief 2D多边形类型定义
 */
using Polygon2D = std::vector<Point2D>;

/**
 * @brief 将3D包围盒投影到BEV平面（xoz）得到2D矩形顶点
 * @param box 3D包围盒
 * @return 顺时针排列的4个顶点坐标
 */
Polygon2D boxToBEVPolygon(const Box& box);

/**
 * @brief 计算两条直线的交点x坐标
 */
float getLineIntersectionX(float x1, float z1, float x2, float z2,
                          float x3, float z3, float x4, float z4);

/**
 * @brief 计算两条直线的交点z坐标
 */
float getLineIntersectionZ(float x1, float z1, float x2, float z2,
                          float x3, float z3, float x4, float z4);

/**
 * @brief 使用鞋带公式计算多边形面积
 * @param polygon 多边形顶点
 * @return 多边形面积
 */
float calculatePolygonArea(const Polygon2D& polygon);

/**
 * @brief 对多边形执行单边裁剪
 * @param polygon 待裁剪的多边形
 * @param x1, z1, x2, z2 裁剪线的两个端点
 * @return 裁剪后的多边形
 */
Polygon2D clipPolygonByLine(const Polygon2D& polygon, 
                           float x1, float z1, float x2, float z2);

/**
 * @brief Sutherland-Hodgman多边形裁剪算法
 * @param subject 被裁剪多边形
 * @param clipper 裁剪多边形
 * @return 交集多边形
 */
Polygon2D sutherlandHodgmanClip(const Polygon2D& subject, const Polygon2D& clipper);

/**
 * @brief 计算两个3D包围盒的IoU
 * @param box1 第一个包围盒
 * @param box2 第二个包围盒
 * @return IoU值 [0, 1]
 */
float calculateIoU3D(const Box& box1, const Box& box2);

/**
 * @brief 计算两个3D包围盒在BEV平面的IoU
 * @param box1 第一个包围盒
 * @param box2 第二个包围盒
 * @return BEV IoU值 [0, 1]
 */
float calculateBEVIoU(const Box& box1, const Box& box2);

} // namespace nms
