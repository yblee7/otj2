#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include <QWidget>
#include <QImage>

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{
	// initialize original index locations
	std::vector<size_t> idx(v.size());
	std::iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	// using std::stable_sort instead of std::sort
	// to avoid unnecessary index re-orderings
	// when v contains elements of equal values
	std::stable_sort(idx.begin(), idx.end(),
					 [&v](size_t i1, size_t i2)
					 { return v[i1] < v[i2]; });

	return idx;
}

class Homography : public QWidget
{
    Q_OBJECT
public:
    explicit Homography(QWidget *parent = nullptr);

    void setImage(const QImage& image_in);

    static double computeRealAspectRatio(const Eigen::Vector2d &center,
                                         const std::vector<Eigen::Vector2d> &corners,
                                         Eigen::Vector3d &euler_angles);

    Eigen::Matrix3d compute(const std::vector<Eigen::Vector2d> &source_points,
                            const std::vector<Eigen::Vector2d> &destination_points,
                            const bool conditioning = true);

    QImage getTransformedImage();

    std::vector<Eigen::Vector2d> getConditionedSourcePoints();

    std::vector<Eigen::Vector2d> getConditionedDestinationPoints();

public slots:

protected:

private:
    void updateLog(std::stringstream &msg);

    QImage image;
    QImage image_transformed;
    std::vector<Eigen::Vector2d> conditioned_source_points;
    std::vector<Eigen::Vector2d> conditioned_destination_points;

signals:
    void updateLogToMainWindow(std::stringstream &msg);
};

#endif
