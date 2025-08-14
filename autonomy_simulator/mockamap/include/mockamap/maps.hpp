/*
 * Copyright 2024 The Openbot Beginner Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace openbot_ros {

class Maps 
{
public:
    typedef struct BasicInfo 
    {
        ros::NodeHandle *nh_private;
        int sizeX;
        int sizeY;
        int sizeZ;
        int seed;
        double scale;
        sensor_msgs::PointCloud2 *output;
        pcl::PointCloud<pcl::PointXYZ> *cloud;
    } BasicInfo;

    BasicInfo getInfo() const;
    void setInfo(const BasicInfo &value);

    Maps();

    void generate(int type);

private:
 
    void pcl2ros();

    void perlin3D();
    void maze2D();
    void randomMapGenerate();
    void Maze3DGen();
    void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
    void recursizeDivisionMaze(Eigen::MatrixXi &maze);
    void optimizeMap();

    BasicInfo info;
};

class MazePoint 
{
public:
    pcl::PointXYZ getPoint();
    int getPoint1();
    int getPoint2();
    double getDist1();
    double getDist2();
    void setPoint(pcl::PointXYZ p);
    void setPoint1(int p);
    void setPoint2(int p);
    void setDist1(double set);
    void setDist2(double set);

private:
    pcl::PointXYZ point;
    double dist1;
    double dist2;
    int point1;
    int point2;
    bool isdoor;
};

} // namespace openbot_ros

