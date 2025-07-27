#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <big_armor_scale.hpp>

int main() {
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("f_mat_and_c_mat.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "错误：无法打开yml文件！" << std::endl;
        return 1;
    }
    fs["F"] >> cameraMatrix;   // 读取相机内参矩阵F
    fs["C"] >> distCoeffs;     // 读取畸变系数矩阵C
    fs.release();              // 释放文件存储对象

    std::cout << "相机内参矩阵 (F):\n" << cameraMatrix << std::endl;
    std::cout << "畸变系数 (C):\n" << distCoeffs << std::endl;

    // 2. 定义3D对象点 (来自 big_armor_scale.hpp)
    // 这些是装甲板灯条角点在装甲板自身坐标系中的3D坐标（单位：米）。
    std::vector<cv::Point3d> objectPoints = PW_BIG; //

    std::cout << "\n对象点 (3D):\n";
    for (const auto& p : objectPoints) {
        std::cout << p << std::endl;
    }

    // 3. 获取2D图像点
    // 对于此示例，我们直接硬编码了 imu_and_armor.txt 中提供的参考点。
    // 在实际应用中，你需要实现图像处理算法（例如，颜色阈值化、轮廓检测）
    // 从 hero.jpg 中检测这些点。
    // 重要提示：请确保图像点的顺序与对象点 (PW_BIG) 的顺序一致！
    // 根据 PW_BIG 的定义和对 hero.jpg 的目视检查，顺序大致为：
    // 左上、左下、右下、右上
    std::vector<cv::Point2d> imagePoints;
    imagePoints.push_back(cv::Point2d(575.508, 282.175)); // 大致左上角点
    imagePoints.push_back(cv::Point2d(573.93, 331.819));  // 大致左下角点
    imagePoints.push_back(cv::Point2d(764.518, 337.652)); // 大致右下角点
    imagePoints.push_back(cv::Point2d(765.729, 286.741)); // 大致右上角点

    std::cout << "\n图像点 (2D):\n";
    for (const auto& p : imagePoints) {
        std::cout << p << std::endl;
    }

    // 可选：加载并显示图片，并在检测到的点上绘制标记以进行验证
    cv::Mat image = cv::imread("hero.jpg");
    if (!image.empty()) {
        for (size_t i = 0; i < imagePoints.size(); ++i) {
            cv::circle(image, imagePoints[i], 5, cv::Scalar(0, 255, 0), -1); // 绿色圆点
            cv::putText(image, std::to_string(i), imagePoints[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2); // 红色数字标签
        }
        cv::imshow("Detected Armor Points", image);
        cv::waitKey(0);
    } else {
        std::cerr << "无法加载图片" << std::endl;
    }

    // 4. 使用 solvePnP 函数计算姿态
    cv::Mat rvec, tvec; // 输出旋转向量 (rvec) 和平移向量 (tvec)

    // solvePnP 函数根据3D对象点、2D图像点、相机内参和畸变系数，
    // 计算对象在相机坐标系中的姿态（旋转和平移）。
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    std::cout << "\n计算得到的姿态:\n";
    std::cout << "旋转向量 (rvec):\n" << rvec << std::endl;
    std::cout << "平移向量 (tvec):\n" << tvec << std::endl;

    // 可选：将旋转向量转换为旋转矩阵
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);
    std::cout << "旋转矩阵:\n" << rotationMatrix << std::endl;

    // rvec 和 tvec 共同定义了装甲板相对于相机在3D空间中的姿态。
    return 0;
}