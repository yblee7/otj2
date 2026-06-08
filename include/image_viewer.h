#ifndef IMAGE_VIEWER_H
#define IMAGE_VIEWER_H

#include <QWidget>
#include <QPaintEvent>
#include <QPainter>
#include <QKeyEvent>

#include <vector>

#include <opencv2/core.hpp>

enum class MagnifierLocation
{
    TopLeft,
    Cursor,
    Adaptive
};

class ImageViewer : public QWidget
{
    Q_OBJECT
public:
    explicit ImageViewer(QWidget *parent = nullptr);
    bool setImage(const QImage &image_in);
    bool setImage(const cv::Mat &image_in);
    bool setCorners(const QList<QPointF> &corners_in);
    QList<QPointF> getCorners();
    QImage getImage();

    void setShowMagnifier(const bool flag);
    void setMagnifierLocation(const MagnifierLocation &magnifier_location_in);
    void setViewCorners(const bool flag);
    void setDetectCorners(const bool flag);

public slots:
    void clearImage();
    void print();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    QPointF computeOffset(const QSize &widget_size);
    QPointF imagePoseToWidgetPose(const QPointF &pose_widget);
    QPointF widgetPoseToImagePose(const QPointF &image_widget);
    void updateStatusBarMessage();
    void resetCorners();

    bool is_translating; // Boolean flag indicating whether current event is of translating or not
    bool is_zooming;     // Boolean flag indicating whether current event is of zooming or not
    bool is_resetting;   // Boolean flag indicating whether current event is of resetting or not

    bool show_magnifier;
    bool view_corners;
    bool detect_corners;

    QRectF magnifier_rect;
    MagnifierLocation magnifier_location;

    QImage image; // Image to render

    QPointF cursor_pose_image;  // Current cursor's coordinate with respect to image's coordinate system [pixel]
    QPointF cursor_pose_widget; // Current cursor's coordinate with respect to widget's coordinate system [pixel]

    double image_zoom; // Image zoom factor

    // A rectangular portion of the image is rendered on the widget in which the rectangle is defined as
    // (offset.x(), offset.y(), widget.width(), widget.height())
    QPointF offset;

    QList<QPointF> corners;

    int selected_point;

signals:
    void keyPressed(QKeyEvent *event);
    void mousePressed(QMouseEvent *event);
    void signalCursorPoseImage(QMouseEvent *event, QPointF *point);
    void infoMessageToMainWindow(QString *msg);
    void infoCornersUpdated();
};

class CornerImageViewer : ImageViewer
{

};

#endif // IMAGE_VIEWER_H
