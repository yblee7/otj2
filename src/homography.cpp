#include "homography.h"
#include <iostream>

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
    // Implement requested functionality based on below lines
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

    double k2 = p1.cross(p4).dot(p3) / p2.cross(p4).dot(p3);
    double k3 = p1.cross(p4).dot(p2) / p3.cross(p4).dot(p2);

    Eigen::Vector3d n2 = k2 * p2 - p1;
    Eigen::Vector3d n3 = k3 * p3 - p1;

    return sqrt((n2.x() * n2.x() + n2.y() * n2.y()) / (n3.x() * n3.x() + n3.y() * n3.y()));
}

Eigen::Matrix3d Homography::compute(const std::vector<Eigen::Vector2d> &source_points,
                                    const std::vector<Eigen::Vector2d> &destination_points,
                                    const bool conditioning)
{
    // Write your own code below

    if(source_points.size() != destination_points.size())
        return Eigen::Matrix3d::Identity();

    if(image.isNull())
        return Eigen::Matrix3d::Identity();
    
    return Eigen::Matrix3d::Identity();
}