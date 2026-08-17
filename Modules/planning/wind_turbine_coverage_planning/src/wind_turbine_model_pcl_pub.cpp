#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigen>

#include <random>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

class WindTurbineModelPclPub
{
public:
    WindTurbineModelPclPub(ros::NodeHandle& nh)
        : rng_(std::random_device{}())
    {
        nh.param("frame_id", frame_id_, std::string("world"));
        nh.param("publish_rate", publish_rate_, 1.0);

        // 这 5 个 STL 以 wind_turbine_gazebo.world 的最终显示模型为准
        nh.param("support_mesh", support_mesh_,
                 std::string("/home/amov/Prometheus/Simulator/gazebo_simulator/models/wind_turbine_gazebo/meshes/Support.stl"));
        nh.param("center_mesh", center_mesh_,
                 std::string("/home/amov/Prometheus/Simulator/gazebo_simulator/models/wind_turbine_gazebo/meshes/Center.stl"));
        nh.param("blade1_mesh", blade1_mesh_,
                 std::string("/home/amov/Prometheus/Simulator/gazebo_simulator/models/wind_turbine_gazebo/meshes/Blade1.stl"));
        nh.param("blade2_mesh", blade2_mesh_,
                 std::string("/home/amov/Prometheus/Simulator/gazebo_simulator/models/wind_turbine_gazebo/meshes/Blade2.stl"));
        nh.param("blade3_mesh", blade3_mesh_,
                 std::string("/home/amov/Prometheus/Simulator/gazebo_simulator/models/wind_turbine_gazebo/meshes/Blade3.stl"));

        // 采样密度控制
        nh.param("sample_density", sample_density_, 1800.0);   // 每平方米采样点数
        nh.param("min_points_per_mesh", min_points_per_mesh_, 800);
        nh.param("leaf_size", leaf_size_, 0.05);               // 体素滤波下采样

        // 如果将来 world 里整体又平移/旋转了，可通过这些参数做整体位姿补偿
        nh.param("model_tx", model_tx_, 0.0);
        nh.param("model_ty", model_ty_, 0.0);
        nh.param("model_tz", model_tz_, 0.0);
        nh.param("model_roll",  model_roll_,  0.0);
        nh.param("model_pitch", model_pitch_, 0.0);
        nh.param("model_yaw",   model_yaw_,   0.0);

        pub_ = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/global_planning/global_pcl", 1, true);

        buildCloud();

        timer_ = nh.createTimer(ros::Duration(1.0 / std::max(0.1, publish_rate_)),
                                &WindTurbineModelPclPub::timerCb, this);
    }

private:
    ros::Publisher pub_;
    ros::Timer timer_;

    std::string frame_id_;
    double publish_rate_;

    std::string support_mesh_, center_mesh_, blade1_mesh_, blade2_mesh_, blade3_mesh_;

    double sample_density_;
    int min_points_per_mesh_;
    double leaf_size_;

    double model_tx_, model_ty_, model_tz_;
    double model_roll_, model_pitch_, model_yaw_;

    std::mt19937 rng_;
    sensor_msgs::PointCloud2 cloud_msg_;

private:
    struct Triangle
    {
        Eigen::Vector3d a, b, c;
        double area;
    };

    Eigen::Matrix3d eulerToRot(double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisd rx(roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(yaw,   Eigen::Vector3d::UnitZ());
        return (rz * ry * rx).toRotationMatrix();
    }

    Eigen::Vector3d samplePointOnTriangle(const Triangle& tri)
    {
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        double r1 = uni(rng_);
        double r2 = uni(rng_);

        double sr1 = std::sqrt(r1);
        double u = 1.0 - sr1;
        double v = sr1 * (1.0 - r2);
        double w = sr1 * r2;

        return u * tri.a + v * tri.b + w * tri.c;
    }

    bool loadMeshTriangles(const std::string& mesh_path,
                           std::vector<Triangle>& triangles,
                           double& total_area)
    {
        pcl::PolygonMesh mesh;
        if (pcl::io::loadPolygonFileSTL(mesh_path, mesh) == 0)
        {
            ROS_ERROR("[wind_turbine_model_pcl_pub] failed to load STL: %s", mesh_path.c_str());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ> vertices;
        pcl::fromPCLPointCloud2(mesh.cloud, vertices);

        triangles.clear();
        total_area = 0.0;

        for (const auto& poly : mesh.polygons)
        {
            if (poly.vertices.size() < 3) continue;

            // STL 一般是三角形；即便不是，这里也只取前三个点
            const auto& p0 = vertices.points[poly.vertices[0]];
            const auto& p1 = vertices.points[poly.vertices[1]];
            const auto& p2 = vertices.points[poly.vertices[2]];

            Triangle tri;
            tri.a = Eigen::Vector3d(p0.x, p0.y, p0.z);
            tri.b = Eigen::Vector3d(p1.x, p1.y, p1.z);
            tri.c = Eigen::Vector3d(p2.x, p2.y, p2.z);
            tri.area = 0.5 * ((tri.b - tri.a).cross(tri.c - tri.a)).norm();

            if (tri.area < 1e-12) continue;

            triangles.push_back(tri);
            total_area += tri.area;
        }

        if (triangles.empty() || total_area <= 0.0)
        {
            ROS_ERROR("[wind_turbine_model_pcl_pub] mesh has no valid triangles: %s", mesh_path.c_str());
            return false;
        }

        ROS_INFO("[wind_turbine_model_pcl_pub] loaded %s, triangles=%zu, area=%.3f",
                 mesh_path.c_str(), triangles.size(), total_area);
        return true;
    }

    void sampleMeshToCloud(const std::string& mesh_path,
                           pcl::PointCloud<pcl::PointXYZ>& cloud,
                           const Eigen::Matrix3d& R_model,
                           const Eigen::Vector3d& t_model)
    {
        std::vector<Triangle> triangles;
        double total_area = 0.0;
        if (!loadMeshTriangles(mesh_path, triangles, total_area))
        {
            return;
        }

        int n_samples = std::max(min_points_per_mesh_, static_cast<int>(std::ceil(total_area * sample_density_)));

        std::vector<double> cdf(triangles.size(), 0.0);
        double accum = 0.0;
        for (size_t i = 0; i < triangles.size(); ++i)
        {
            accum += triangles[i].area;
            cdf[i] = accum;
        }

        std::uniform_real_distribution<double> area_rand(0.0, total_area);

        for (int i = 0; i < n_samples; ++i)
        {
            double pick = area_rand(rng_);
            auto it = std::lower_bound(cdf.begin(), cdf.end(), pick);
            size_t idx = std::distance(cdf.begin(), it);
            if (idx >= triangles.size()) idx = triangles.size() - 1;

            Eigen::Vector3d p = samplePointOnTriangle(triangles[idx]);

            // 当前 wind_turbine_gazebo.world 下，这 5 个 STL 已经与最终显示模型对齐
            // 这里只保留一个“整体位姿补偿”接口，方便你将来 world 里再整体平移/旋转模型
            Eigen::Vector3d pw = R_model * p + t_model;

            pcl::PointXYZ pt;
            pt.x = pw.x();
            pt.y = pw.y();
            pt.z = pw.z();
            cloud.points.push_back(pt);
        }
    }

    void buildCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        raw->clear();

        Eigen::Matrix3d R_model = eulerToRot(model_roll_, model_pitch_, model_yaw_);
        Eigen::Vector3d t_model(model_tx_, model_ty_, model_tz_);

        sampleMeshToCloud(support_mesh_, *raw, R_model, t_model);
        sampleMeshToCloud(center_mesh_,  *raw, R_model, t_model);
        sampleMeshToCloud(blade1_mesh_,  *raw, R_model, t_model);
        sampleMeshToCloud(blade2_mesh_,  *raw, R_model, t_model);
        sampleMeshToCloud(blade3_mesh_,  *raw, R_model, t_model);

        raw->width = raw->points.size();
        raw->height = 1;
        raw->is_dense = true;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (leaf_size_ > 1e-6)
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(raw);
            vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            vg.filter(*filtered);
        }
        else
        {
            *filtered = *raw;
        }

        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;

        pcl::toROSMsg(*filtered, cloud_msg_);
        cloud_msg_.header.frame_id = frame_id_;

        ROS_INFO("[wind_turbine_model_pcl_pub] raw points=%zu, filtered points=%zu",
                 raw->points.size(), filtered->points.size());
    }

    void timerCb(const ros::TimerEvent&)
    {
        cloud_msg_.header.stamp = ros::Time::now();
        pub_.publish(cloud_msg_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_turbine_model_pcl_pub");
    ros::NodeHandle nh("~");

    WindTurbineModelPclPub node(nh);
    ros::spin();
    return 0;
}