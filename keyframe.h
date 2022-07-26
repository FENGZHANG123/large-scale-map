/** \brief:  关键帧定义
 *  \author: 张锋
 *  \date: 2022.7.5
*/

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dat_io.h"

typedef pcl::PointXYZI PointType;

class Keyframe
{
public:
    Keyframe(int64_t id, const double &stamp_, const Eigen::Matrix4f &pose_,
             const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    virtual ~Keyframe();

    int64_t GetKeyframeID();

    void ClearPointCloud();

    void SavePointCloud(const std::string &path);

    void LoadPointCloud(const std::string &path);

//public:
//    pcl::PointCloud<PointType>::Ptr DownSampling(const pcl::PointCloud<PointType>::ConstPtr &cloud);

//    void SetKeyframePose(const Eigen::Matrix4f &pose);

//    void SetKeyframePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

//    void SearchNearestCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud);

private:
    int64_t id_;

    double stamp_;

    float voxel_size_;

public:
    Eigen::Matrix4f pose_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
