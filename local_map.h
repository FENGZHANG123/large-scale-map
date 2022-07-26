/** \brief:
 * 1. 包含一个稀疏点云用于显示
 * 2. 用于搜索附近的block及其关键帧,创建局部点云用于建图和定位的匹配
 * 3.其他的点云处于非激活状态
 * 4. 算出点云的空间距离，补充关键帧的更多选择
 * \date: 20220710
 * \author: 张锋 */

#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include "block.h"
#include "keyframe.h"
#include <vector>
#include <queue>

//typedef pcl::PointXYZI PointType;

class LocalMap
{
public:
    LocalMap(const std::string &path);

    virtual ~LocalMap();

public:
    /** \in: 当前位置和点云
     *  \ou: 周边的关键帧点云以及位姿
    */
    void SearchNearestKeyframe(const double &stamp, const Eigen::Matrix4f & Twl,const pcl::PointCloud<PointType>::Ptr &cloud);

    //当前点云的处理
    float GetCloudBoundingBoxSize(const pcl::PointCloud<PointType>::Ptr &cloud);

    //区分一下建图和定位：建图时block存图后清理缓存，定位时只清理block缓存
    void SetLocalMapMode(const int &mode);

    //block相关
private:
    //搜索block
    void SearchNearestBlock(const float &x, const float &y);

    void SetAdaptiveParams();

    //keyframe相关
private:
    void ComputeConvexHull();

    void ComputeConcaveHull();

    void LoadNearestBlock();

    //todo: 新建包括block的左上，右下角点的位置
    void CreateNewBlock(const float &x, const float &y);

    void CreateNewKeyframe(const double &stamp, const Eigen::Matrix4f &pose, const pcl::PointCloud<PointType>::Ptr &cloud);

    void FindNewKeyFrameCondition(const Eigen::Matrix4f &currPose, const Eigen::Matrix4f &pose);

    bool IsKeyframe(const Eigen::Matrix4f &pose);

    template<typename T>
    std::vector<T> FindBlockKMinDistance(const std::vector<float> &dist, const T &k, const std::vector<T> &order);

    pcl::PointCloud<PointType>::Ptr DownSampling(const float & voxel, const pcl::PointCloud<PointType>::Ptr &cloud);

private:
     //global map params
    int64_t keyframe_size_;

    std::vector<Block*> map_block_list_;

    int block_size_;

    float block_scale_;

    pcl::PointCloud<PointType>::Ptr map_for_view_;

    int mode_; //区分建图和定位,建图为0，定位为1

    //local map params
    int local_block_num_;

    int local_keyframe_num_;

    std::vector<int64_t> old_keyframe_list_;

    std::vector<int> current_keyframe_list_;

    std::atomic<bool> keyframes_changed_flag_;

    std::vector<int> old_block_list_;

    std::vector<int> current_block_list_;

    std::atomic<bool> blocks_changed_flag_;

    Block *ptrBlock_;

    std::atomic<bool> need_block_flag_;

    pcl::PointCloud<PointType>::Ptr keyframes_map_;

    pcl::PointCloud<PointType>::Ptr keyframe_for_view_;

    float voxel_size_;

    float voxel_view_size_;

    //关键帧参数
    std::atomic<bool> is_keyframe_flag_;

    std::atomic<bool> inside_block_flag_;

    Keyframe* ptrKeyFrame_;

    pcl::PointCloud<PointType>::Ptr keyframes_pose_;

    pcl::ConvexHull<PointType> convex_hull_;

    pcl::ConcaveHull<PointType> concave_hull_;

    std::vector<int> keyframe_convex_;

    std::vector<int> keyframe_concave_;
    //todo: 点云初始化和析构的检查

    std::string local_map_path_;

    //关键帧的条件
    float bounding_box_size_;

    float keyframe_dist_thres_;

    float keyframe_angle_thres_;

    int near_keyframe_num_;

    float keyframe_nearest_dist_;

    Eigen::Matrix4f nearest_keyframe_pose_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
