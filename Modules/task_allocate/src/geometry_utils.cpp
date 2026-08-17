#include "task_allocate/geometry_utils.h"

#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace task_allocate
{

double pathLength(const std::vector<Eigen::Vector3d>& pts)
{
    if(pts.size() < 2)
    {
        return 0.0;
    }
    double len = 0.0;
    for(size_t i = 1; i < pts.size(); ++i)
    {
        len += (pts[i] - pts[i - 1]).norm();
    }
    return len;
}

double distancePointToSegment2D(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    const Eigen::Vector2d ab = b - a;
    const double denom = ab.squaredNorm();
    if(denom < 1e-9)
    {
        return (p - a).norm();
    }
    double t = (p - a).dot(ab) / denom;
    t = std::max(0.0, std::min(1.0, t));
    const Eigen::Vector2d proj = a + t * ab;
    return (p - proj).norm();
}

double signedDistanceToObstacle2D(double x, double y, const StaticObstacle& obs)
{
    const double dx = x - obs.position.x();
    const double dy = y - obs.position.y();

    if(obs.type == 0)
    {
        const double hx = std::max(0.01, obs.size.x() * 0.5);
        const double hy = std::max(0.01, obs.size.y() * 0.5);
        const double qx = std::abs(dx) - hx;
        const double qy = std::abs(dy) - hy;
        const double outside_x = std::max(qx, 0.0);
        const double outside_y = std::max(qy, 0.0);
        const double outside_dist = std::sqrt(outside_x * outside_x + outside_y * outside_y);
        const double inside_dist = std::min(std::max(qx, qy), 0.0);
        return outside_dist + inside_dist;
    }

    const double radius = std::max(0.01, obs.size.x());
    return std::sqrt(dx * dx + dy * dy) - radius;
}

bool isPointInsideObstacle2D(double x, double y, const StaticObstacle& obs, double inflation, bool avoid_danger_zone)
{
    if(obs.isDangerZone() && !avoid_danger_zone)
    {
        return false;
    }
    return signedDistanceToObstacle2D(x, y, obs) <= inflation;
}

bool isLineSegmentCollisionFree2D(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b,
                                  const std::vector<StaticObstacle>& obstacles,
                                  double inflation,
                                  bool avoid_danger_zone,
                                  double sample_step)
{
    const Eigen::Vector2d a2(a.x(), a.y());
    const Eigen::Vector2d b2(b.x(), b.y());
    const double dist = (b2 - a2).norm();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(dist / std::max(sample_step, 0.05))));

    for(int i = 0; i <= sample_num; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(sample_num);
        const double x = a.x() + t * (b.x() - a.x());
        const double y = a.y() + t * (b.y() - a.y());
        for(size_t k = 0; k < obstacles.size(); ++k)
        {
            if(isPointInsideObstacle2D(x, y, obstacles[k], inflation, avoid_danger_zone))
            {
                return false;
            }
        }
    }
    return true;
}

double segmentObstacleRisk(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& goal,
                           const std::vector<StaticObstacle>& obstacles,
                           double safe_margin,
                           bool include_danger_zone,
                           int sample_num)
{
    if(obstacles.empty())
    {
        return 0.0;
    }

    const int n = std::max(2, sample_num);
    const double margin = std::max(0.1, safe_margin);
    double total_risk = 0.0;

    for(size_t k = 0; k < obstacles.size(); ++k)
    {
        const StaticObstacle& obs = obstacles[k];
        if(obs.isDangerZone() && !include_danger_zone)
        {
            continue;
        }

        double min_signed_dist = 1e9;
        double inside_count = 0.0;
        for(int i = 0; i <= n; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(n);
            const double x = start.x() + t * (goal.x() - start.x());
            const double y = start.y() + t * (goal.y() - start.y());
            const double d = signedDistanceToObstacle2D(x, y, obs);
            min_signed_dist = std::min(min_signed_dist, d);
            if(d <= 0.0)
            {
                inside_count += 1.0;
            }
        }

        double risk = 0.0;
        if(min_signed_dist <= 0.0)
        {
            risk = 1.0 + inside_count / static_cast<double>(n + 1);
        }
        else if(min_signed_dist < margin)
        {
            risk = (margin - min_signed_dist) / margin;
        }

        if(obs.isDangerZone())
        {
            risk *= 0.75;
        }
        total_risk += risk;
    }

    return total_risk;
}

std::vector<Eigen::Vector3d> makeStraightPath(const Eigen::Vector3d& start,
                                              const Eigen::Vector3d& goal,
                                              int sample_num,
                                              double z_offset)
{
    std::vector<Eigen::Vector3d> path;
    const int n = std::max(2, sample_num);
    path.reserve(n + 1);
    for(int i = 0; i <= n; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        Eigen::Vector3d p = start + t * (goal - start);
        p.z() += z_offset;
        path.push_back(p);
    }
    return path;
}

std::vector<Eigen::Vector3d> makeCurvedDemoPath(const Eigen::Vector3d& start,
                                                const Eigen::Vector3d& goal,
                                                int sample_num,
                                                double side_offset,
                                                double mid_height)
{
    std::vector<Eigen::Vector3d> path;
    const int n = std::max(2, sample_num);
    path.reserve(n + 1);

    Eigen::Vector3d dir = goal - start;
    Eigen::Vector3d side(-dir.y(), dir.x(), 0.0);
    if(side.norm() > 1e-6)
    {
        side.normalize();
    }
    else
    {
        side = Eigen::Vector3d(0.0, 1.0, 0.0);
    }

    for(int i = 0; i <= n; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        Eigen::Vector3d p = start + t * (goal - start);
        p += std::sin(M_PI * t) * side_offset * side;
        p.z() += std::sin(M_PI * t) * mid_height;
        path.push_back(p);
    }
    return path;
}

} // namespace task_allocate
