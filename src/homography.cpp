#include "homography.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <QMessageBox>

Homography::Homography(QWidget *parent) : QWidget(parent)
{
}

void Homography::setImage(const QImage &image_in)
{
    image = image_in.copy();
    image = image.convertToFormat(QImage::Format_RGB888);
}

QImage Homography::getTransformedImage()
{
    return image_transformed.copy();
}

std::vector<Eigen::Vector2d> Homography::getConditionedDestinationPoints()
{
    return conditioned_destination_points;
}

std::vector<Eigen::Vector2d> Homography::getConditionedSourcePoints()
{
    return conditioned_source_points;
}

void Homography::updateLog(std::stringstream &msg)
{
    emit updateLogToMainWindow(msg);
}

Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &R)
{
    double sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
 
    bool singular = sy < 1e-6;
 
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }

    return Eigen::Vector3d(x, y, z);
}

double Homography::computeRealAspectRatio(const Eigen::Vector2d &center,
                                          const std::vector<Eigen::Vector2d> &corners,
                                          Eigen::Vector3d &euler_angles)
{
    euler_angles.setZero();

    if(corners.size() != 4)
        return -1;

    std::vector<Eigen::Vector3d> corners_centered_homogeneous =
    {
        Eigen::Vector3d(corners[0].x() - center.x(), corners[0].y() - center.y(), 1.0),
        Eigen::Vector3d(corners[1].x() - center.x(), corners[1].y() - center.y(), 1.0),
        Eigen::Vector3d(corners[3].x() - center.x(), corners[3].y() - center.y(), 1.0),
        Eigen::Vector3d(corners[2].x() - center.x(), corners[2].y() - center.y(), 1.0)
    };

    const Eigen::Vector3d &p1 = corners_centered_homogeneous[0];
    const Eigen::Vector3d &p2 = corners_centered_homogeneous[1];
    const Eigen::Vector3d &p3 = corners_centered_homogeneous[2];
    const Eigen::Vector3d &p4 = corners_centered_homogeneous[3];

    const double k2_den = p2.cross(p4).dot(p3);
    const double k3_den = p3.cross(p4).dot(p2);
    if (k2_den == 0.0 || k3_den == 0.0)
        return -1;

    double k2 = p1.cross(p4).dot(p3) / k2_den;
    double k3 = p1.cross(p4).dot(p2) / k3_den;

    Eigen::Vector3d n2 = k2 * p2 - p1;
    Eigen::Vector3d n3 = k3 * p3 - p1;

    const auto metricAspectRatio = [&n2, &n3](double focal_squared) {
        const auto metricNormSquared = [focal_squared](const Eigen::Vector3d &v) {
            return (v.x() * v.x() + v.y() * v.y()) / focal_squared + v.z() * v.z();
        };

        const double width_metric = metricNormSquared(n2);
        const double height_metric = metricNormSquared(n3);
        if (width_metric < 0.0 || height_metric <= std::numeric_limits<double>::epsilon())
            return -1.0;

        return std::sqrt(width_metric / height_metric);
    };

    const auto fallbackMetricAspectRatio = [&n2, &n3]() 
    {
        const double width_squared = n2.x() * n2.x() + n2.y() * n2.y();
        const double height_squared = n3.x() * n3.x() + n3.y() * n3.y();
        if (height_squared == 0.0)
            return -1.0;

        return sqrt(width_squared / height_squared);
    };

    const double singular_epsilon = std::sqrt(std::numeric_limits<double>::epsilon());
    const bool k2_singular = std::abs(k2 - 1.0) <= singular_epsilon;
    const bool k3_singular = std::abs(k3 - 1.0) <= singular_epsilon;
    if (k2_singular || k3_singular)
    {
        if (k2_singular && k3_singular)
            return fallbackMetricAspectRatio();

        return -1.0;
    }

    const double focal_den = n2.z() * n3.z();
    const double focal_squared = -(n2.x() * n3.x() + n2.y() * n3.y()) / focal_den;
    if (focal_squared <= 0.0 || !std::isfinite(focal_squared))
    {
        return fallbackMetricAspectRatio();
    }

    const double focal = std::sqrt(focal_squared);
    const Eigen::Matrix3d inv_camera =
        (Eigen::Matrix3d() << 1.0 / focal, 0.0, 0.0,
                             0.0, 1.0 / focal, 0.0,
                             0.0, 0.0, 1.0)
            .finished();

    Eigen::Vector3d r1 = inv_camera * n2;
    Eigen::Vector3d r2 = inv_camera * n3;
    r1.normalize();
    r2.normalize();
    Eigen::Vector3d r3 = r1.cross(r2).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = r1;
    rotation.col(1) = r2;
    rotation.col(2) = r3;
    euler_angles = rotationMatrixToEulerAngles(rotation);

    return metricAspectRatio(focal_squared);
}

// 역변환 위해서 구조체 정의
struct NormResult
{
    std::vector<Eigen::Vector2d> points;
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    bool valid = false;
};

static NormResult buildNormTransform(const std::vector<Eigen::Vector2d> &pts)
{
    NormResult result;
    if (pts.empty())
        return result;

    // 중점 구하기
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const auto &p : pts)
        center += p;
    center /= static_cast<double>(pts.size());

    // 평균 거리 구함
    double mean_dist = 0.0;
    for (const auto &p : pts)
        mean_dist += (p - center).norm();
    mean_dist /= static_cast<double>(pts.size());

    if (mean_dist <= std::numeric_limits<double>::epsilon())
        return result;

    const double scale = std::sqrt(2.0) / mean_dist;
    result.transform << scale, 0.0, -scale * center.x(),
                        0.0, scale, -scale * center.y(),
                        0.0, 0.0,  1.0;

    result.points.reserve(pts.size());
    for (const auto &p : pts)
    {
        const Eigen::Vector3d np = result.transform * Eigen::Vector3d(p.x(), p.y(), 1.0);
        result.points.emplace_back(np.x() / np.z(), np.y() / np.z());
    }

    result.valid = true;
    return result;
}

static bool hasCollinearTriple(const std::vector<Eigen::Vector2d> &pts)
{
    const int n = static_cast<int>(pts.size());
    for (int i = 0; i < n - 2; ++i)
        for (int j = i + 1; j < n - 1; ++j)
            for (int k = j + 1; k < n; ++k)
            {
                // (B-A) × (C-A) = 0 이면 세 점이 일직선
                const Eigen::Vector2d ab = pts[j] - pts[i];
                const Eigen::Vector2d ac = pts[k] - pts[i];
                const double cross = ab.x() * ac.y() - ab.y() * ac.x();
                const double denom = ab.norm() * ac.norm();
                // 정규화된 |sin(θ)| 로 비교 (픽셀 스케일 무관)
                if (denom > std::numeric_limits<double>::epsilon() &&
                    std::abs(cross) / denom < 0.01)
                    return true;
            }
    return false;
}

static bool isFiniteMatrix(const Eigen::Matrix3d &matrix)
{
    for (int row = 0; row < matrix.rows(); ++row)
        for (int col = 0; col < matrix.cols(); ++col)
            if (!std::isfinite(matrix(row, col)))
                return false;

    return true;
}

Eigen::Matrix3d Homography::compute(const std::vector<Eigen::Vector2d> &source_points,
                                    const std::vector<Eigen::Vector2d> &destination_points,
                                    const bool conditioning)
{
    image_transformed = QImage();
    conditioned_source_points.clear();
    conditioned_destination_points.clear();

    if (source_points.size() != destination_points.size() || source_points.size() < 4)
        return Eigen::Matrix3d::Identity();

    if (image.isNull())
        return Eigen::Matrix3d::Identity();

    const int n = static_cast<int>(source_points.size());

    // collinear 검사: 진행은 하되 경고 출력
    if (hasCollinearTriple(source_points) || hasCollinearTriple(destination_points))
    {
        QMessageBox::warning(nullptr, "Warning",
            "source 또는 destination points에 3점 이상 collinear(일직선)인 조합이 존재합니다.\n호모그래피를 계산할 수 없습니다.");
    }

    // --- 정규화 ---
    NormResult src_norm, dst_norm;
    if (conditioning)
    {
        src_norm = buildNormTransform(source_points);
        dst_norm = buildNormTransform(destination_points);
        if (!src_norm.valid || !dst_norm.valid)
            return Eigen::Matrix3d::Identity();
    }
    else
    {
        src_norm.points = source_points;
        dst_norm.points = destination_points;
        src_norm.valid = dst_norm.valid = true;
    }

    conditioned_source_points      = src_norm.points;
    conditioned_destination_points = dst_norm.points;

    // --- DLT: 2n x 9 행렬 A 구성.
    // 외적값이0이 되야함
    Eigen::MatrixXd A(2 * n, 9);
    for (int i = 0; i < n; ++i)
    {
        const double x = src_norm.points[i].x();
        const double y = src_norm.points[i].y();
        const double u = dst_norm.points[i].x();
        const double v = dst_norm.points[i].y();

        A.row(2 * i)     << -x, -y, -1.0,  0.0,  0.0,  0.0,  u*x,  u*y,  u;
        A.row(2 * i + 1) <<  0.0,  0.0,  0.0, -x, -y, -1.0,  v*x,  v*y,  v;
    }

    // --- SVD → V의 마지막 열이 해 ---
    // jacobian은 정확도 bdc는 속도. 즉 bdc는 큰 행렬에서 사용.  ComputeFullV플레그는 V를 전부구하라는 뜻.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    // Ah = 0 으로부터 h = 0이면 자명한 해가 되므로 h가 최소가 되는 값을 사용. 즉 특이값 최소점을 사용.
    const Eigen::VectorXd h = svd.matrixV().col(8);

    Eigen::Matrix3d H_norm;
    H_norm << h(0), h(1), h(2),
               h(3), h(4), h(5),
               h(6), h(7), h(8);

    // --- 역정규화: H = T_dst^{-1} * H̃ * T_src ---
    Eigen::Matrix3d H = dst_norm.transform.inverse() * H_norm * src_norm.transform;
    if (!isFiniteMatrix(H) || H.norm() <= std::numeric_limits<double>::epsilon())
        return Eigen::Matrix3d::Identity();

    // 스케일 제거
    if (std::abs(H(2, 2)) > std::numeric_limits<double>::epsilon())
        H /= H(2, 2);
    else
        H /= H.norm();

    if (!isFiniteMatrix(H) || std::abs(H.determinant()) <= std::numeric_limits<double>::epsilon())
        return Eigen::Matrix3d::Identity();

    // --- 이미지 변환 (dst 점 기준 출력 크기) ---
    double max_x = 0.0, max_y = 0.0;
    for (const auto &p : destination_points)
    {
        max_x = std::max(max_x, p.x());
        max_y = std::max(max_y, p.y());
    }
    const int output_margin = static_cast<int>(std::ceil(image.width() * 0.1));
    const cv::Size output_size(
        std::max(image.width(),  static_cast<int>(std::ceil(max_x)) + 1),
        static_cast<int>(std::ceil(max_y)) + output_margin);

    cv::Mat src_mat(image.height(), image.width(), CV_8UC3,
                    const_cast<uchar *>(image.bits()),
                    static_cast<size_t>(image.bytesPerLine()));

    cv::Mat H_cv;
    cv::eigen2cv(H, H_cv);

    cv::Mat dst_mat = cv::Mat::zeros(output_size, CV_8UC3);
    cv::Mat H_inv = H_cv.inv();

    const int srcWidth  = src_mat.cols;
    const int srcHeight = src_mat.rows;
    const int channel   = src_mat.channels();

    #pragma omp parallel for schedule(static)
    for (int row = 0; row < output_size.height; row++)
    {
        unsigned char *dstRow = dst_mat.ptr<unsigned char>(row);
        for (int col = 0; col < output_size.width; col++)
        {
            const double hw = H_inv.at<double>(2,0) * col
                            + H_inv.at<double>(2,1) * row
                            + H_inv.at<double>(2,2);
            if (std::abs(hw) < std::numeric_limits<double>::epsilon())
                continue;

            const double x = (H_inv.at<double>(0,0) * col
                            + H_inv.at<double>(0,1) * row
                            + H_inv.at<double>(0,2)) / hw;
            const double y = (H_inv.at<double>(1,0) * col
                            + H_inv.at<double>(1,1) * row
                            + H_inv.at<double>(1,2)) / hw;

            if (x < 0 || y < 0 || x >= srcWidth || y >= srcHeight)
                continue;

            int r0 = int(std::floor(y));
            int c0 = int(std::floor(x));
            int r1 = r0 + 1;
            int c1 = c0 + 1;

            if (r1 == srcHeight) r1 = r0;
            if (c1 == srcWidth)  c1 = c0;

            double dr = y - r0;
            double dc = x - c0;
            const unsigned char *srcRow0 = src_mat.ptr<unsigned char>(r0);
            const unsigned char *srcRow1 = src_mat.ptr<unsigned char>(r1);

            const double w00 = (1.0 - dr) * (1.0 - dc);
            const double w01 = (1.0 - dr) * dc;
            const double w10 = dr * (1.0 - dc);
            const double w11 = dr * dc;

            int dstOffset = col * channel;
            int src00 = c0 * channel;
            int src01 = c1 * channel;

            for (int ch = 0; ch < channel; ch++)
            {
                double rgb = srcRow0[src00 + ch] * w00 +
                    srcRow0[src01 + ch] * w01 +
                    srcRow1[src00 + ch] * w10 +
                    srcRow1[src01 + ch] * w11;

                dstRow[dstOffset + ch] =
                    static_cast<unsigned char>(std::clamp(rgb, 0.0, 255.0));
            }
        }
    }

    image_transformed = QImage(dst_mat.data, dst_mat.cols, dst_mat.rows,
                                static_cast<int>(dst_mat.step),
                                QImage::Format_RGB888).copy();

    return H;
}
