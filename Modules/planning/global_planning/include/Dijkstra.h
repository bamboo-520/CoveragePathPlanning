#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

// 直接复用 A* 中已经定义好的 Node / NodePtr / NodeComparator0 / NodeHashTable0
#include "A_star.h"
#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

#define NODE_NAME "Global_Planner [Dijkstra]"

namespace Global_Planning
{

extern ros::Publisher message_pub;

class Dijkstra
{
    private:
        // 备选路径点指针容器
        std::vector<NodePtr> path_node_pool_;
        // 使用节点计数器、迭代次数计数器
        int use_node_num_, iter_num_;
        // 扩展的节点
        NodeHashTable0 expanded_nodes_;
        // open set （根据规则已排序好）
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
        // 最终路径点容器
        std::vector<NodePtr> path_nodes_;

        // 参数
        double lambda_heu_;
        int max_search_num;
        double tie_breaker_;
        int is_2D;
        double fly_height;

        /* ---------- record data ---------- */
        Eigen::Vector3d goal_pos;

        // 地图相关
        std::vector<int> occupancy_buffer_;
        double resolution_, inv_resolution_;
        Eigen::Vector3d origin_, map_size_3d_;
        bool has_global_point;

        // 辅助函数
        Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
        void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);
        void retrievePath(NodePtr end_node);

        // 与 A* 保持同一套接口，Dijkstra 实际上不使用启发项
        double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

    public:
        Dijkstra() {}
        ~Dijkstra();

        enum
        {
          REACH_END = 1,
          NO_PATH = 2
        };

        Occupy_map::Ptr Occupy_map_ptr;

        void reset();
        void init(ros::NodeHandle& nh);
        bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
        std::vector<Eigen::Vector3d> getPath();
        nav_msgs::Path get_ros_path();
        std::vector<NodePtr> getVisitedNodes();

        typedef std::shared_ptr<Dijkstra> Ptr;
};

}

#endif
