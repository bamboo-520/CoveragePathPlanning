#include "task_allocate/visualization_utils.h"

#include <ros/ros.h>

#include <sstream>

namespace task_allocate
{

geometry_msgs::Point toPointMsg(const Eigen::Vector3d& p)
{
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    return pt;
}

geometry_msgs::Pose toPoseMsg(const Eigen::Vector3d& p)
{
    geometry_msgs::Pose pose;
    pose.position = toPointMsg(p);
    pose.orientation.w = 1.0;
    return pose;
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

void setMarkerColor(visualization_msgs::Marker& marker, double r, double g, double b, double a)
{
    marker.color = makeColor(r, g, b, a);
}

visualization_msgs::Marker makeBasicMarker(const std::string& frame_id,
                                           const std::string& ns,
                                           int id,
                                           int type,
                                           const Eigen::Vector3d& position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = toPoseMsg(position);
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

std::string gazeboColorScript(double r, double g, double b, double a)
{
    std::ostringstream ss;
    ss << "<ambient>" << r << " " << g << " " << b << " " << a << "</ambient>"
       << "<diffuse>" << r << " " << g << " " << b << " " << a << "</diffuse>";
    return ss.str();
}

std::string makeGazeboSphereSdf(double radius,
                                double r,
                                double g,
                                double b,
                                double a,
                                bool with_collision)
{
    std::ostringstream ss;
    ss << "<sdf version='1.6'><model name='visual_sphere'>"
       << "<static>true</static><link name='link'>"
       << "<visual name='visual'><geometry><sphere><radius>" << radius << "</radius></sphere></geometry>"
       << "<material>" << gazeboColorScript(r, g, b, a) << "</material></visual>";
    if(with_collision)
    {
        ss << "<collision name='collision'><geometry><sphere><radius>" << radius << "</radius></sphere></geometry></collision>";
    }
    ss << "</link></model></sdf>";
    return ss.str();
}

std::string makeGazeboBoxSdf(const Eigen::Vector3d& size,
                             double r,
                             double g,
                             double b,
                             double a,
                             bool with_collision)
{
    std::ostringstream ss;
    ss << "<sdf version='1.6'><model name='visual_box'>"
       << "<static>true</static><link name='link'>"
       << "<visual name='visual'><geometry><box><size>" << size.x() << " " << size.y() << " " << size.z()
       << "</size></box></geometry><material>" << gazeboColorScript(r, g, b, a) << "</material></visual>";
    if(with_collision)
    {
        ss << "<collision name='collision'><geometry><box><size>" << size.x() << " " << size.y() << " " << size.z()
           << "</size></box></geometry></collision>";
    }
    ss << "</link></model></sdf>";
    return ss.str();
}

std::string makeGazeboCylinderSdf(double radius,
                                  double height,
                                  double r,
                                  double g,
                                  double b,
                                  double a,
                                  bool with_collision)
{
    std::ostringstream ss;
    ss << "<sdf version='1.6'><model name='visual_cylinder'>"
       << "<static>true</static><link name='link'>"
       << "<visual name='visual'><geometry><cylinder><radius>" << radius << "</radius><length>" << height
       << "</length></cylinder></geometry><material>" << gazeboColorScript(r, g, b, a) << "</material></visual>";
    if(with_collision)
    {
        ss << "<collision name='collision'><geometry><cylinder><radius>" << radius << "</radius><length>" << height
           << "</length></cylinder></geometry></collision>";
    }
    ss << "</link></model></sdf>";
    return ss.str();
}

} // namespace task_allocate
