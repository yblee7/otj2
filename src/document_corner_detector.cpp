#include "document_corner_detector.h"

#include <QDebug>

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

static double computeMedian(const cv::Mat &gray)
{
    cv::Mat flat = gray.reshape(1, (int)gray.total());
    cv::Mat sorted;
    cv::sort(flat, sorted, cv::SORT_ASCENDING);
    return sorted.at<uchar>((int)(sorted.total() / 2));
}

static cv::Mat preprocessForEdges(const cv::Mat &rgb_in)
{
    cv::Mat gray;
    cv::cvtColor(rgb_in, gray, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    int kernelSize = std::max(rgb_in.cols, rgb_in.rows) / 30;
    kernelSize = (kernelSize % 2 == 0) ? kernelSize + 1 : kernelSize;
    kernelSize = std::max(kernelSize, 15);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, kernel);

    double med = computeMedian(gray);
    cv::Mat edges;
    cv::Canny(gray, edges, std::max(0.0, 0.66 * med), std::min(255.0, 1.33 * med));
    cv::dilate(edges, edges,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    return edges;
}

static std::vector<cv::Point2f> contourQuad(const cv::Mat &edges, int w, int h)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges.clone(), contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    std::sort(contours.begin(), contours.end(),
              [](const auto &a, const auto &b)
              { return cv::contourArea(a) > cv::contourArea(b); });

    const double min_area = w * h * 0.10;
    for (const auto &c : contours)
    {
        double peri = cv::arcLength(c, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, 0.02 * peri, true);

        if (approx.size() == 4 && cv::isContourConvex(approx) &&
            cv::contourArea(approx) > min_area)
        {
            std::vector<cv::Point2f> points;
            for (auto &p : approx)
                points.emplace_back((float)p.x, (float)p.y);

            return points;
        }
    }
    return {};
}

static cv::Point2f lineIntersect(cv::Vec2f l1, cv::Vec2f l2, bool &ok)
{
    float r1 = l1[0], t1 = l1[1];
    float r2 = l2[0], t2 = l2[1];
    float det = std::cos(t1) * std::sin(t2) - std::cos(t2) * std::sin(t1);
    ok = std::abs(det) > 1e-6f;
    if (!ok)
        return {-1.f, -1.f};

    return {(r1 * std::sin(t2) - r2 * std::sin(t1)) / det,
            (r2 * std::cos(t1) - r1 * std::cos(t2)) / det};
}

static bool hasEdgeNear(const cv::Mat &edges, cv::Point2f p, int radius)
{
    const int x = cvRound(p.x);
    const int y = cvRound(p.y);
    for (int dy = -radius; dy <= radius; ++dy)
    {
        const int yy = y + dy;
        if (yy < 0 || yy >= edges.rows)
            continue;

        for (int dx = -radius; dx <= radius; ++dx)
        {
            const int xx = x + dx;
            if (xx >= 0 && xx < edges.cols && edges.at<uchar>(yy, xx) > 0)
                return true;
        }
    }
    return false;
}

static float edgeSupportRatio(const cv::Mat &edges, cv::Point2f a, cv::Point2f b)
{
    const float len = (float)cv::norm(b - a);
    if (len < 1.0f)
        return 0.0f;

    const int samples = std::max(12, (int)(len / 4.0f));
    int hits = 0;

    for (int i = 0; i <= samples; ++i)
    {
        const float t = (float)i / (float)samples;
        const cv::Point2f p(a.x + (b.x - a.x) * t,
                            a.y + (b.y - a.y) * t);
        if (hasEdgeNear(edges, p, 2))
            ++hits;
    }

    return (float)hits / (float)(samples + 1);
}

static float extendAlongEdge(const cv::Mat &edges,
                             cv::Point2f top,
                             cv::Point2f &bottom,
                             float max_extend)
{
    cv::Point2f dir = bottom - top;
    const float len = (float)cv::norm(dir);
    if (len < 1.0f)
        return 0.0f;

    dir *= 1.0f / len;
    const float step = 3.0f;
    const int max_steps = std::max(1, (int)(max_extend / step));
    const cv::Point2f original = bottom;
    cv::Point2f last_hit = bottom;
    int misses = 0;
    bool extended = false;

    for (int i = 1; i <= max_steps; ++i)
    {
        const cv::Point2f p = original + dir * (step * (float)i);
        if (p.x < 0 || p.x >= edges.cols || p.y < 0 || p.y >= edges.rows)
            break;

        if (hasEdgeNear(edges, p, 3))
        {
            last_hit = p;
            misses = 0;
            extended = true;
        }
        else if (extended && ++misses >= 6)
        {
            break;
        }
    }

    bottom = last_hit;
    return (float)cv::norm(bottom - original);
}

static void extendBottomAlongSideEdges(const cv::Mat &edges, std::vector<cv::Point2f> &q)
{
    if (q.size() != 4)
        return;

    const float left_len = (float)cv::norm(q[3] - q[0]);
    const float right_len = (float)cv::norm(q[2] - q[1]);
    const float max_extend = std::min((float)edges.rows * 0.25f,
                                      std::max(left_len, right_len) * 0.65f);

    cv::Point2f bl = q[3];
    cv::Point2f br = q[2];
    const float left_ext = extendAlongEdge(edges, q[0], bl, max_extend);
    const float right_ext = extendAlongEdge(edges, q[1], br, max_extend);
    const float max_ext = std::max(left_ext, right_ext);
    const float min_ext = std::min(left_ext, right_ext);

    if (max_ext < std::max(12.0f, std::max(left_len, right_len) * 0.08f))
        return;
    if (min_ext / max_ext < 0.35f)
        return;

    q[2] = br;
    q[3] = bl;
    qDebug() << "[houghQuad] extended bottom left/right=" << left_ext << right_ext;
}

struct EdgeSupport
{
    float minimum = 0.0f;
    float average = 0.0f;
};

static EdgeSupport computeQuadEdgeSupport(const cv::Mat &edges,
                                          cv::Point2f tl,
                                          cv::Point2f tr,
                                          cv::Point2f br,
                                          cv::Point2f bl)
{
    const float top = edgeSupportRatio(edges, tl, tr);
    const float bottom = edgeSupportRatio(edges, bl, br);
    const float left = edgeSupportRatio(edges, tl, bl);
    const float right = edgeSupportRatio(edges, tr, br);

    EdgeSupport support;
    support.minimum = std::min(std::min(top, bottom), std::min(left, right));
    support.average = (top + bottom + left + right) * 0.25f;
    return support;
}

static std::vector<cv::Point2f> houghQuad(const cv::Mat &edges, int w, int h)
{
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI / 180.0, std::min(w, h) / 4);
    if (lines.size() < 4)
        return {};

    const float PI = (float)CV_PI;
    float cx = w / 2.0f, cy = h / 2.0f;

    const float ANGLE_TOL = 15.0f * PI / 180.0f;
    std::vector<std::vector<cv::Vec2f>> clusters;
    for (auto &l : lines)
    {
        bool merged = false;
        for (auto &cluster : clusters)
        {
            float theta = cluster[0][1];
            float diff = std::abs(l[1] - theta);
            diff = std::min(diff, PI - diff);
            if (diff < ANGLE_TOL)
            {
                cluster.push_back(l);
                merged = true;
                break;
            }
        }
        if (!merged)
            clusters.push_back({l});
    }

    auto ypos = [&](const cv::Vec2f &l) {
        return (l[0] - cx * std::cos(l[1])) / std::sin(l[1]);
    };
    auto xpos = [&](const cv::Vec2f &l) {
        return (l[0] - cy * std::sin(l[1])) / std::cos(l[1]);
    };

    const size_t MAX_PER_CLUSTER = 8;
    const float RHO_TOL = (float)std::min(w, h) * 0.04f;

    std::vector<cv::Vec2f> h_lines, v_lines;
    for (auto &cluster : clusters)
    {
        float theta = cluster[0][1];
        bool isHorizon = std::abs(theta - PI / 2.f) < PI / 4.f;
        auto posOf = [&](const cv::Vec2f &l) { return isHorizon ? ypos(l) : xpos(l); };

        std::vector<cv::Vec2f> dedup;
        for (auto &l : cluster)
        {
            bool dup = false;
            for (auto &r : dedup)
            {
                if (std::abs(posOf(l) - posOf(r)) < RHO_TOL)
                {
                    dup = true;
                    break;
                }
            }
            if (!dup)
                dedup.push_back(l);
        }

        if (dedup.size() > MAX_PER_CLUSTER)
            dedup.resize(MAX_PER_CLUSTER);

        auto &dst = isHorizon ? h_lines : v_lines;
        for (auto &l : dedup)
            dst.push_back(l);
    }

    if (h_lines.size() < 2 || v_lines.size() < 2)
        return {};

    const float img_area = (float)w * h;
    const float margin = (float)std::max(w, h) * 0.1f;
    auto inBounds = [&](cv::Point2f p) {
        return p.x > -margin && p.x < w + margin &&
               p.y > -margin && p.y < h + margin;
    };
    auto cross = [](cv::Point2f o, cv::Point2f a, cv::Point2f b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    float best_score = -1.f;
    std::vector<cv::Point2f> best_pts;
    float best_area_ratio = 0.0f;
    float best_min_support = 0.0f;
    float best_avg_support = 0.0f;
    const size_t nh = h_lines.size(), nv = v_lines.size();
    int candidate_count = 0;
    int rejected_area = 0;
    int rejected_shape = 0;
    int rejected_support = 0;

    for (size_t i = 0; i < nh; ++i)
    for (size_t j = i + 1; j < nh; ++j)
    for (size_t k = 0; k < nv; ++k)
    for (size_t m = k + 1; m < nv; ++m)
    {
        cv::Vec2f ht = h_lines[i], hb = h_lines[j];
        if (ypos(ht) > ypos(hb))
            std::swap(ht, hb);
        cv::Vec2f vl = v_lines[k], vr = v_lines[m];
        if (xpos(vl) > xpos(vr))
            std::swap(vl, vr);

        bool ok1, ok2, ok3, ok4;
        auto tl = lineIntersect(ht, vl, ok1);
        auto tr = lineIntersect(ht, vr, ok2);
        auto br = lineIntersect(hb, vr, ok3);
        auto bl = lineIntersect(hb, vl, ok4);
        if (!ok1 || !ok2 || !ok3 || !ok4)
            continue;
        if (!inBounds(tl) || !inBounds(tr) || !inBounds(br) || !inBounds(bl))
            continue;

        float c1 = cross(tl, tr, br);
        float c2 = cross(tr, br, bl);
        float c3 = cross(br, bl, tl);
        float c4 = cross(bl, tl, tr);
        bool convex = (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) ||
                      (c1 < 0 && c2 < 0 && c3 < 0 && c4 < 0);
        if (!convex)
            continue;

        std::vector<cv::Point2f> q = {tl, tr, br, bl};
        float area = std::abs((float)cv::contourArea(q));
        float area_ratio = area / img_area;
        if (area_ratio < 0.06f || area_ratio > 0.70f)
        {
            ++rejected_area;
            continue;
        }

        float top_len = (float)cv::norm(tr - tl);
        float bot_len = (float)cv::norm(br - bl);
        float lft_len = (float)cv::norm(bl - tl);
        float rgt_len = (float)cv::norm(br - tr);
        float hr = std::min(top_len, bot_len) / std::max(top_len, bot_len);
        float vr_ratio = std::min(lft_len, rgt_len) / std::max(lft_len, rgt_len);
        if (hr < 0.35f || vr_ratio < 0.35f)
        {
            ++rejected_shape;
            continue;
        }
        float aspect = (top_len + bot_len) / (lft_len + rgt_len);
        if (aspect < 0.25f || aspect > 4.0f)
        {
            ++rejected_shape;
            continue;
        }

        const EdgeSupport support = computeQuadEdgeSupport(edges, tl, tr, br, bl);
        if (support.minimum < 0.18f || support.average < 0.32f)
        {
            ++rejected_support;
            continue;
        }

        auto strength = [](size_t idx, size_t total) {
            return 1.f - (float)idx / (float)total;
        };
        float s_lines = (strength(i, nh) + strength(j, nh) +
                         strength(k, nv) + strength(m, nv)) * 0.25f;

        const float support_score = support.minimum * 0.65f + support.average * 0.35f;
        const float area_score = std::sqrt(area_ratio);
        float score = support_score * support_score *
                      hr * vr_ratio *
                      area_score *
                      (0.5f + 0.5f * s_lines);
        ++candidate_count;
        if (score > best_score)
        {
            best_score = score;
            best_pts = q;
            best_area_ratio = area_ratio;
            best_min_support = support.minimum;
            best_avg_support = support.average;
        }
    }

    qDebug() << "[houghQuad] lines=" << (int)lines.size()
             << " h=" << (int)h_lines.size()
             << " v=" << (int)v_lines.size()
             << " candidates=" << candidate_count
             << " reject(area/shape/support)="
             << rejected_area << rejected_shape << rejected_support
             << " best_score=" << best_score
             << " area=" << best_area_ratio
             << " support(min/avg)=" << best_min_support << best_avg_support;

    if (best_score < 0.f)
        return {};

    extendBottomAlongSideEdges(edges, best_pts);

    qDebug() << "[houghQuad] chosen="
             << QPointF(best_pts[0].x, best_pts[0].y)
             << QPointF(best_pts[1].x, best_pts[1].y)
             << QPointF(best_pts[2].x, best_pts[2].y)
             << QPointF(best_pts[3].x, best_pts[3].y);
    return best_pts;
}

static QList<QPointF> orderAndScale(std::vector<cv::Point2f> pts, double inv_scale)
{
    QList<QPointF> result;
    if (pts.size() != 4)
        return result;

    cv::Point2f center(0.f, 0.f);
    for (const auto &p : pts)
        center += p;
    center *= 0.25f;

    std::sort(pts.begin(), pts.end(),
              [&](const cv::Point2f &a, const cv::Point2f &b)
              {
                  return std::atan2(a.y - center.y, a.x - center.x) <
                         std::atan2(b.y - center.y, b.x - center.x);
              });

    auto top_left = std::min_element(pts.begin(), pts.end(),
                                     [](const cv::Point2f &a, const cv::Point2f &b)
                                     {
                                         return a.x + a.y < b.x + b.y;
                                     });
    std::rotate(pts.begin(), top_left, pts.end());

    for (const auto &p : pts)
        result.push_back(QPointF(p.x * inv_scale, p.y * inv_scale));

    return result;
}

static void refineCornersSubPixel(const cv::Mat &rgb_in, std::vector<cv::Point2f> &points)
{
    if (points.size() != 4)
        return;

    cv::Mat gray;
    cv::cvtColor(rgb_in, gray, cv::COLOR_RGB2GRAY);

    cv::cornerSubPix(gray, points, cv::Size(7, 7), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                      20, 0.03));
}

QList<QPointF> DocumentCornerDetector::detect(const QImage &qimage)
{
    QImage img = qimage.convertToFormat(QImage::Format_RGB888);
    cv::Mat src(img.height(), img.width(), CV_8UC3,
                const_cast<uchar *>(img.bits()), (size_t)img.bytesPerLine());

    const int MAX_DIM = 800;
    double scale = 1.0;
    cv::Mat detectionImg;
    int maxdim = std::max(src.cols, src.rows);
    if (maxdim > MAX_DIM)
    {
        scale = (double)MAX_DIM / maxdim;
        cv::resize(src, detectionImg, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    else
    {
        detectionImg = src;
    }

    auto tryPipeline = [&](const cv::Mat &rgb_in, const char *channel) -> std::vector<cv::Point2f>
    {
        cv::Mat edges = preprocessForEdges(rgb_in);
        auto points = contourQuad(edges, rgb_in.cols, rgb_in.rows);
        if (!points.empty())
        {
            qDebug() << "[detectDocumentCorners] mode=CONTOUR channel=" << channel;
            return points;
        }

        points = houghQuad(edges, rgb_in.cols, rgb_in.rows);
        if (!points.empty())
            qDebug() << "[detectDocumentCorners] mode=HOUGH channel=" << channel;
        return points;
    };

    auto points = tryPipeline(detectionImg, "gray");

    if (points.empty())
    {
        cv::Mat hsv;
        cv::cvtColor(detectionImg, hsv, cv::COLOR_RGB2HSV);
        std::vector<cv::Mat> ch;
        cv::split(hsv, ch);
        cv::Mat sat_rgb;
        cv::cvtColor(ch[1], sat_rgb, cv::COLOR_GRAY2RGB);
        points = tryPipeline(sat_rgb, "saturation");
    }

    if (points.empty())
    {
        qDebug() << "[detectDocumentCorners] mode=FAILED";
        return {};
    }

    refineCornersSubPixel(detectionImg, points);
    return orderAndScale(points, 1.0 / scale);
}
