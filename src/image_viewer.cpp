#include "image_viewer.h"
#include "document_corner_detector.h"
#include "homography.h"

#include <QDebug>

#include <opencv2/core.hpp>

ImageViewer::ImageViewer(QWidget *parent) :
    QWidget(parent),
    is_translating(false),
    is_zooming(false),
    is_resetting(false),
    image_zoom(1),
    show_magnifier(false),
    view_corners(true),
    detect_corners(false),
    magnifier_rect(QRectF(0, 0, 150, 150)),
    magnifier_location(MagnifierLocation::TopLeft),
    selected_point(-1)
{
    setMouseTracking(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setStyleSheet("color: green; font: 50px;");
}

void ImageViewer::setShowMagnifier(const bool flag)
{
    show_magnifier = flag;
}

void ImageViewer::setViewCorners(const bool flag)
{
    view_corners = flag;
}

void ImageViewer::setDetectCorners(const bool flag)
{
    detect_corners = flag;
}

void ImageViewer::setMagnifierLocation(const MagnifierLocation &magnifier_location_in)
{
    magnifier_location = magnifier_location_in;

    update();
}

QPointF ImageViewer::computeOffset(const QSize &widget_size)
{
    qreal pix_width = image.width();
    qreal pix_height = image.height();
    qreal scale_horizontal = (qreal)widget_size.width() / pix_width;
    qreal scale_vertical = (qreal)widget_size.height() / pix_height;
    image_zoom = std::min(scale_horizontal, scale_vertical);

    qreal xoffset = 0.0, yoffset = 0.0;
    if ((pix_width * image_zoom) < widget_size.width())
    {
        xoffset = (widget_size.width() - (pix_width * image_zoom)) * 0.5;
    }
    if ((pix_height * image_zoom) < widget_size.height())
    {
        yoffset = (widget_size.height() - (pix_height * image_zoom)) * 0.5;
    }

    return -1 * QPointF(xoffset, yoffset) / image_zoom;
}

QList<QPointF> ImageViewer::getCorners()
{
    return corners;
}

QImage ImageViewer::getImage()
{
    return image;
}

QPointF ImageViewer::widgetPoseToImagePose(const QPointF& widget_pose)
{
    return offset + widget_pose / image_zoom;
}

QPointF ImageViewer::imagePoseToWidgetPose(const QPointF& image_pose)
{
    return image_zoom * (image_pose - offset);
}

void ImageViewer::resetCorners()
{
    corners.clear();

    if (!image.isNull())
    {
        QList<QPointF> detected = DocumentCornerDetector::detect(image);
        if (detected.size() == 4)
        {
            corners = detected;
            emit infoCornersUpdated();
            return;
        }
    }

    // 검출 실패 시 기본 위치 (10%~90%)
    int width  = this->image.width();
    int height = this->image.height();
    corners.push_back(QPointF(width * 0.1, height * 0.1));
    corners.push_back(QPointF(width * 0.9, height * 0.1));
    corners.push_back(QPointF(width * 0.9, height * 0.9));
    corners.push_back(QPointF(width * 0.1, height * 0.9));

    emit infoCornersUpdated();
}

bool ImageViewer::setImage(const QImage &image_in)
{
    if (image_in.isNull())
        return false;

    image = image_in.copy();
    image_zoom = 1;

    QSize sz = this->geometry().size();
    offset = computeOffset(sz);

    cursor_pose_image = QPointF(0, 0);
    cursor_pose_widget = QPointF(0, 0);

    if(detect_corners)
    {
        resetCorners();
    }

    update();

    return true;
}

bool ImageViewer::setImage(const cv::Mat &image_in)
{
    if(image_in.empty())
        return false;
    
    cv::Mat image_converted;
    if(image_in.channels() == 3)
    {
        cv::cvtColor(image_in, image_converted, cv::COLOR_BGR2RGB);
    }
    else if(image_in.channels() == 1)
    {
        cv::cvtColor(image_in, image_converted, cv::COLOR_GRAY2RGB);
    }

    return setImage(QImage((uchar*)image_converted.data, image_converted.cols, image_converted.rows, image_converted.step, QImage::Format_RGB888));
}

bool ImageViewer::setCorners(const QList<QPointF> &corners_in)
{
    if(corners_in.size() < 1)
        return false;

    corners = corners_in;
    emit infoCornersUpdated();

    return true;
}

void ImageViewer::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(this->rect(), QBrush(Qt::gray));

    QPointF new_top_left;

    if(is_resetting)
    {
        new_top_left = computeOffset(this->rect().size());
        is_resetting = false;
    }
    else if(is_zooming || is_translating)
    {
        qreal ui = cursor_pose_image.x();
        qreal vi = cursor_pose_image.y();
        qreal uw = cursor_pose_widget.x();
        qreal vw = cursor_pose_widget.y();
        new_top_left = QPointF(ui - uw / image_zoom,
                               vi - vw / image_zoom);

        if(is_zooming)
            is_zooming = false;
    }
    else
    {
        new_top_left = offset;
    }

    QRectF newSourceRect(new_top_left.x(),
                         new_top_left.y(),
                         this->rect().width() / image_zoom,
                         this->rect().height() / image_zoom);

    painter.scale(image_zoom, image_zoom);
    painter.drawImage(QPointF(0, 0), image, newSourceRect);
    painter.scale(1.0 / image_zoom, 1.0 / image_zoom);

    offset = new_top_left;

    if(show_magnifier && !image.isNull())
    {
        const double magnifier_zoom = 4.0;

        QRectF src_rect(
            cursor_pose_image.x() - magnifier_rect.width()  / (2.0 * magnifier_zoom * image_zoom),
            cursor_pose_image.y() - magnifier_rect.height() / (2.0 * magnifier_zoom * image_zoom),
            magnifier_rect.width()  / (magnifier_zoom * image_zoom),
            magnifier_rect.height() / (magnifier_zoom * image_zoom)
        );

        QRectF dst_rect;
        if(magnifier_location == MagnifierLocation::TopLeft)
        {
            dst_rect = QRectF(1, 1, magnifier_rect.width(), magnifier_rect.height());
        }
        else if(magnifier_location == MagnifierLocation::Cursor)
        {
            dst_rect = QRectF(
                cursor_pose_widget.x() - magnifier_rect.width()  / 2.0,
                cursor_pose_widget.y() - magnifier_rect.height() / 2.0,
                magnifier_rect.width(),
                magnifier_rect.height()
            );
        }
        else
        {
            qreal x = cursor_pose_widget.x() > this->rect().center().x() ? 0 : this->rect().width()  - magnifier_rect.width();
            qreal y = cursor_pose_widget.y() > this->rect().center().y() ? 0 : this->rect().height() - magnifier_rect.height();
            dst_rect = QRectF(x, y, magnifier_rect.width(), magnifier_rect.height());
        }

        painter.drawImage(dst_rect, image, src_rect);

        // image coord -> magnifier widget coord
        auto toMagnifier = [&](const QPointF &p) {
            return QPointF(dst_rect.left() + (p.x() - src_rect.left()) * magnifier_zoom * image_zoom,
                           dst_rect.top()  + (p.y() - src_rect.top())  * magnifier_zoom * image_zoom);
        };

        if(corners.size() > 0)
        {
            painter.setClipRect(dst_rect);
            painter.setPen(QPen(selected_point > -1 ? Qt::red : Qt::green, 3));
            painter.setBrush(Qt::NoBrush);
            for(int i = 0; i < corners.size(); ++i)
                painter.drawLine(toMagnifier(corners[i]),
                                 toMagnifier(corners[(i + 1) % corners.size()]));
            painter.setClipping(false);
        }
        
        // 테두리랑 중앙 선 그리기
        painter.setPen(QPen(Qt::yellow, 1));
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(dst_rect.left(),       dst_rect.center().y()),
                         QPointF(dst_rect.right(),      dst_rect.center().y()));
        painter.drawLine(QPointF(dst_rect.center().x(), dst_rect.top()),
                         QPointF(dst_rect.center().x(), dst_rect.bottom()));

        painter.setPen(QPen(QColor(0, 255, 255), 2));
        painter.drawRect(dst_rect);
    }

    int pen_size_min = 4 * image_zoom < 4 ? 4 : 4 * image_zoom;
    if(corners.size() > 0)
    {
        if(selected_point > -1)
        {
            painter.setPen(QPen(Qt::red , pen_size_min * 0.5));
            painter.setBrush(Qt::red);
        }
        else
        {
            painter.setPen(QPen(Qt::green, pen_size_min * 0.5));
            painter.setBrush(Qt::green);
        }
        for (int i = 0; i < corners.size(); ++i)
        {
            const QPointF &p1 = imagePoseToWidgetPose(corners[i]);
            int j = (i + 1) % corners.size();
            const QPointF &p2 = imagePoseToWidgetPose(corners[j]);
            painter.drawLine(p1, p2);
        }
    }

    painter.end();

    QWidget::paintEvent(event);
}

void ImageViewer::updateStatusBarMessage()
{
    if (cursor_pose_image.x() < 0 || cursor_pose_image.x() > image.width() - 1 ||
        cursor_pose_image.y() < 0 || cursor_pose_image.y() > image.height() - 1)
        return;

    QRgb pix_val = image.pixel(cursor_pose_image.x(), cursor_pose_image.y());

    QString message = QString("offset: (%1, %2), "
                              "cursor_pose_widget: (%3, %4), "
                              "cursor_pose_image: (%5, %6), "
                              "Red: %7, Green: %8, Blue: %9, "
                              "zoom: %10")
                          .arg(offset.x(), 4, 'f', 0)
                          .arg(offset.y(), 4, 'f', 0)
                          .arg(cursor_pose_widget.x(), 4, 'f', 0)
                          .arg(cursor_pose_widget.y(), 4, 'f', 0)
                          .arg(cursor_pose_image.x(), 8, 'f', 3)
                          .arg(cursor_pose_image.y(), 8, 'f', 3)
                          .arg(qRed(pix_val), 3)
                          .arg(qGreen(pix_val), 3)
                          .arg(qBlue(pix_val), 3)
                          .arg(image_zoom, 6, 'f', 3);

    infoMessageToMainWindow(&message);
}

void ImageViewer::resizeEvent(QResizeEvent *event)
{
    offset = computeOffset(event->size());

    int magnifier_size = std::min(event->size().width(), event->size().height()) * 0.2;
    magnifier_rect.setWidth(magnifier_size);
    magnifier_rect.setHeight(magnifier_size);

    update();

	QWidget::resizeEvent(event);
}

void ImageViewer::mousePressEvent(QMouseEvent *event)
{
    cursor_pose_widget = event->pos();
    cursor_pose_image = QPointF(cursor_pose_widget / image_zoom) + offset;

    if (event->button() == Qt::RightButton)
    {
        is_translating = true;
    }
    else if (event->button() == Qt::LeftButton && view_corners)
    {
        Eigen::Vector2d pose(cursor_pose_image.x(), cursor_pose_image.y());
        double nearest_distance = 1e6;
        int nearest_index = 0;
        for(int i = 0; i < corners.size(); ++i)
        {
            const QPointF &p = corners[i];
            double distance = (pose - Eigen::Vector2d(p.x(), p.y())).norm();
            if(distance < nearest_distance)
            {
                nearest_distance = distance;
                nearest_index = i;
            }
        }
        corners[nearest_index] = QPointF(pose.x(), pose.y());
        selected_point = nearest_index;
        emit infoCornersUpdated();
        update();
    }

    emit mousePressed(event);
    emit signalCursorPoseImage(event, &cursor_pose_image);
}

void ImageViewer::mouseMoveEvent(QMouseEvent *event)
{
    cursor_pose_widget = event->pos();

    updateStatusBarMessage();

    if(!is_translating)
        cursor_pose_image = QPointF(cursor_pose_widget / image_zoom) + offset;
    
    if(selected_point > -1)
    {
        corners[selected_point] = cursor_pose_image;
        emit infoCornersUpdated();
    }

    update();
}

void ImageViewer::mouseReleaseEvent(QMouseEvent *event)
{
    cursor_pose_widget = event->pos();
    cursor_pose_image = QPointF(cursor_pose_widget / image_zoom) + offset;

    if (event->button() == Qt::RightButton && is_translating)
	{
		is_translating = false;
	}
    if (event->button() == Qt::LeftButton)
    {
        selected_point = -1;
    }

    update();
}

void ImageViewer::wheelEvent(QWheelEvent *event)
{
    qreal degrees = event->angleDelta().y() / 8.0;

    cursor_pose_widget = event->position();
    cursor_pose_image = QPointF(cursor_pose_widget / image_zoom) + offset;

    qreal steps = degrees / 60.0;
    qreal zoom_inc = std::pow(1.125, steps);
    qreal factor = image_zoom * zoom_inc;

    image_zoom = std::clamp(factor, 0.1, 20.0);

    is_zooming = true;

    updateStatusBarMessage();

    update();

    QWidget::wheelEvent(event);
}

void ImageViewer::keyPressEvent(QKeyEvent* event)
{
    qDebug() << event->key();
    emit keyPressed(event);
    if(event->key() == Qt::Key_R)
    {
        is_resetting = true;
        if(detect_corners)
        {
            resetCorners();
        }
        update();
    }
}

void ImageViewer::clearImage()
{
    image = QImage();
    update();
}

void ImageViewer::print()
{
#if defined(QT_PRINTSUPPORT_LIB) && QT_CONFIG(printdialog)
    QPrinter printer(QPrinter::HighResolution);

    QPrintDialog printDialog(&printer, this);

    if (printDialog.exec() == QDialog::Accepted)
    {
        QPainter painter(&printer);
        QRect rect = painter.viewport();
        QSize size = image.size();
        size.scale(rect.size(), Qt::KeepAspectRatio);
        painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
        painter.setWindow(image.rect());
        painter.drawImage(0, 0, image);
    }
#endif // QT_CONFIG(printdialog)
}
