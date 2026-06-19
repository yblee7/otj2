#ifndef DOCUMENT_CORNER_DETECTOR_H
#define DOCUMENT_CORNER_DETECTOR_H

#include <QImage>
#include <QList>
#include <QPointF>

class DocumentCornerDetector
{
public:
    static QList<QPointF> detect(const QImage &image);
};

#endif // DOCUMENT_CORNER_DETECTOR_H
