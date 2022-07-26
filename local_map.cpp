#include "local_map.h"

using namespace std;

LocalMap::LocalMap(const std::string &path)
    :keyframe_size_(0)
    ,block_size_(0)
    ,block_scale_(50.f)
    ,mode_(0) //默认是建图
    ,local_block_num_(4)
    ,local_keyframe_num_(10)
    ,keyframes_changed_flag_(false)
    ,blocks_changed_flag_(false)
    ,ptrBlock_(nullptr)
    ,need_block_flag_(true)
    ,voxel_size_(0.05)
    ,voxel_view_size_(0.5)
    ,is_keyframe_flag_(false)
    ,inside_block_flag_(false)
    ,ptrKeyFrame_(nullptr)
    ,local_map_path_(path)
    ,keyframe_dist_thres_(1.0)
    ,keyframe_angle_thres_(3)
    ,near_keyframe_num_(0)
    ,nearest_keyframe_pose_(Eigen::Matrix4f::Identity())
    ,keyframe_nearest_dist_(std::numeric_limits<float>::max())
{
}

LocalMap::~LocalMap()
{
    old_keyframe_list_.clear();
    current_keyframe_list_.clear();
    old_block_list_.clear();
    current_block_list_.clear();
    map_block_list_.clear();
    delete ptrBlock_;
    delete ptrKeyFrame_;
    keyframe_convex_.clear();
    keyframe_concave_.clear();
}

//todo: 点云有可能还包含ID号: surf，corner等
void LocalMap::SearchNearestKeyframe(const double &stamp, const Eigen::Matrix4f &Twl, const pcl::PointCloud<PointType>::Ptr &cloud)
{
    bounding_box_size_ = GetCloudBoundingBoxSize(cloud);

    SearchNearestBlock(Twl(0,3), Twl(1,3));

    //把关键帧找出来
    if(!current_block_list_.empty())
    {
        vector<float> dist;
        vector<int> dist_order;
        int iter = 0;
        for(int i = 0; i < current_block_list_.size(); ++i)
        {
            ptrBlock_ = map_block_list_[current_block_list_[i]];
            for(int j = 0; j < ptrBlock_->keyframe_list_.size(); ++j)
            {
                PointType point;
                point.x = ptrBlock_->keyframe_list_[i]->pose_(0,3);
                point.y = ptrBlock_->keyframe_list_[i]->pose_(1,3);
                point.z = ptrBlock_->keyframe_list_[i]->pose_(2,3);
                keyframes_pose_->emplace_back(point);

                float d = sqrt(pow(Twl(0,3) - point.x, 2) +
                               pow(Twl(1,3) - point.y, 2) +
                               pow(Twl(2,3) - point.z, 2));

                dist.emplace_back(d);
                dist_order.emplace_back(iter);
                iter++;
            }
        }

        //把关键帧挑出来
        current_keyframe_list_ = FindBlockKMinDistance<int>(dist, local_keyframe_num_, dist_order);

        std::vector<float> convex_ds;
        for(int i = 0; i < keyframe_convex_.size(); ++i)
        {
            convex_ds.emplace_back(dist[keyframe_convex_[i]]);
        }

        std::vector<int> current_convex = FindBlockKMinDistance<int>(convex_ds, local_keyframe_num_, keyframe_convex_);

        std::vector<float> concave_ds;
        for(int i = 0; i < keyframe_concave_.size(); ++i)
        {
            concave_ds.emplace_back(dist[keyframe_concave_[i]]);
        }
        std::vector<int> current_concave = FindBlockKMinDistance<int>(concave_ds, local_keyframe_num_, keyframe_concave_);

        current_keyframe_list_.insert(current_keyframe_list_.end(), current_convex.begin(), current_convex.end());
        current_keyframe_list_.insert(current_keyframe_list_.end(), current_concave.begin(), current_concave.end());

        sort(current_keyframe_list_.begin(), current_keyframe_list_.end());
        current_keyframe_list_.erase(unique(current_keyframe_list_.begin(), current_keyframe_list_.end()), current_keyframe_list_.end());
    }

    //判断要不要成为关键帧,第一帧要成为关键帧,第一帧是自动创建block
    CreateNewKeyframe(stamp, Twl, cloud);

    //逻辑关系：如果block没变，keyframes有可能没变，但block变了，keyframes一定变了, 目的是使用该条件降低局部keyframes列表的变更

    //根据block再去检索到附近的关键帧
    int counter = 0;
    for(int i = 0; i < current_block_list_.size(); ++i)
    {
        ptrBlock_ = map_block_list_[current_block_list_[i]];

        //获取到周边空间点云
        for(int j = 0; j < ptrBlock_->keyframe_list_.size(); ++j)
        {
            for(int k = 0; k < current_keyframe_list_.size(); ++k)
            {
                if(current_keyframe_list_[k] == counter)
                {
                    counter++;
                    *keyframes_map_ += *(ptrBlock_->keyframe_list_[j]->cloud_);

                    //顺便算下最近的关键帧用来判断当前帧要不要成为关键帧
                    FindNewKeyFrameCondition(Twl, ptrBlock_->keyframe_list_[j]->pose_);
                    break;
                }
            }
        }
    }

    for(int i = 0; i < current_block_list_.size(); ++i) //不能将该式和上面的循环合并，否则会错位，这个机制待优化
    {
        ptrBlock_ = map_block_list_[current_block_list_[i]];
        //判断关键帧是否在block内: 关键帧后面有可能是重复过来的
        if(is_keyframe_flag_)
        {
            if(ptrBlock_->UnderBlock(Twl(0, 3), Twl(1, 3)))
            {
                inside_block_flag_ = true;
                //把关键帧插入
                ptrBlock_->AddNewKeyFrameToBlock(ptrKeyFrame_);
            }
        }
    }
    //如果不在block内，重新再创建block；
    if(is_keyframe_flag_)
    {
        if(!inside_block_flag_)
        {
            CreateNewBlock(Twl(0,3), Twl(1,3));
        }
    }

    //最后： 置标志位
    inside_block_flag_ = false;
    is_keyframe_flag_ = false;
    keyframes_pose_->clear();
}

float LocalMap::GetCloudBoundingBoxSize(const pcl::PointCloud<PointType>::Ptr &cloud)
{
    std::vector<float> dist;

    for(int i = 0; i <= cloud->points.size(); i++)
    {
        float d = std::sqrt(pow(cloud->points[i].x, 2) +
                            pow(cloud->points[i].y, 2) +
                            pow(cloud->points[i].z, 2));
        dist.push_back(d);
    }

    // median
    std::nth_element(dist.begin(), dist.begin() + dist.size()/2, dist.end());
    return dist[dist.size()/2];
}

//todo: 实现根据ID查找相应的block和关键帧
void LocalMap::SetLocalMapMode(const int &mode)
{
    if(mode != 0 && mode != 1)
    {
        cout << "set local map mode wrong!" << endl;
        return;
    }
    mode_ = mode;
}

void LocalMap::SearchNearestBlock(const float &x, const float &y)
{
    //block：搜索四个
    if(block_size_ >= local_block_num_)
    {
        vector<float> dist;
        vector<int> order;
        float d = .0f;
        for(int i = 0; i < map_block_list_.size(); ++i)
        {
            // d = std::sqrt(pow(Twl(0,3) - , 2) + pow(Twl(1,3) - , 2) );
            ptrBlock_ = map_block_list_[i];
            d = pow(x - ptrBlock_->center_corner_.x, 2) +
                    pow(y - ptrBlock_->center_corner_.y, 2);

            dist.emplace_back(d);
            order.emplace_back(i);
        }

        //找出最近的四个
        current_block_list_ = FindBlockKMinDistance<int>(dist, local_block_num_, order);
        sort(current_block_list_.begin(), current_block_list_.end());
        //        current_block_list_.erase(unique(current_block_list_.begin(), current_block_list_.end()), current_block_list_.end());
        if(!old_block_list_.empty())
        {
            if(old_block_list_ == current_block_list_)
            {
                blocks_changed_flag_ = false;
            }
            else
            {
                blocks_changed_flag_ = true;
                //todo: 分改变和没改变的:新增的加个线程去读,老的不需要了加个线程去存;  对没改变的开始处理
                //todo: 分建图和定位
                std::string block_path = " ";
                for(int i = 0; i < current_block_list_.size(); ++i) // 正向获取
                {
                    bool new_block_change = false;
                    for(int j = 0; j < old_block_list_.size();++j)
                    {
                        if(current_block_list_[i] != old_block_list_[j])
                        {
                            new_block_change = true;
                            break;
                        }
                    }
                    if(new_block_change)
                    {
                        //去加载block
                        ptrBlock_ = map_block_list_[current_block_list_[i]];
                        ptrBlock_->SetBlockStatus(BlockStatus::Active);
                        block_path = local_map_path_+ "/" + boost::lexical_cast<string>(ptrBlock_->GetBlockID()) + "/";
                        ptrBlock_->LoadBlock(block_path);
                    }
                }

                for(int i = 0; i < old_block_list_.size(); ++i) //反向获取
                {
                    bool old_block_change = false;
                    for(int j = 0; j < current_block_list_.size();++j)
                    {
                        if(current_block_list_[j] != old_block_list_[i])
                        {
                            old_block_change = true;
                            break;
                        }
                    }
                    if(old_block_change)
                    {
                        //处理block
                        if(mode_ = 0) //建图是0
                        {
                            //todo把block保存下来
                            ptrBlock_ = map_block_list_[old_block_list_[i]];
                            ptrBlock_->SetBlockStatus(BlockStatus::Inactive);
                            block_path = local_map_path_+ "/" + boost::lexical_cast<string>(ptrBlock_->GetBlockID()) + "/";
                            ptrBlock_->SaveBlock(block_path);

                        }
                        else if(mode_ = 1) //定位是1
                        {
                            //todo把block从内存里清掉
                            ptrBlock_ = map_block_list_[old_block_list_[i]];
                            ptrBlock_->SetBlockStatus(BlockStatus::Inactive);
                            ptrBlock_->ClearBlock();
                        }
                    }
                }
            }
            old_block_list_ = current_block_list_;
        }
        else
        {
            current_block_list_.clear();
            for(int i = 0; i < map_block_list_.size(); ++i)
            {
                current_block_list_.emplace_back(i);
            }
            blocks_changed_flag_ = false;
        }
    }
}

void LocalMap::SetAdaptiveParams()
{
    // Set Keyframe Thresh from Spaciousness Metric
    if (bounding_box_size_ > 20.0){
      keyframe_dist_thres_ = 10.0;
    } else if (bounding_box_size_ > 10.0 && bounding_box_size_ <= 20.0) {
      keyframe_dist_thres_ = 5.0;
    } else if (bounding_box_size_ > 5.0 && bounding_box_size_ <= 10.0) {
      keyframe_dist_thres_ = 1.0;
    } else if (bounding_box_size_ <= 5.0) {
      keyframe_dist_thres_ = 0.5;
    }
  
    // set concave hull alpha
    concave_hull_.setAlpha(keyframe_dist_thres_);
}

void LocalMap::ComputeConvexHull()
{
    if(keyframes_pose_->size() <4)
        return;

    convex_hull_.setInputCloud(keyframes_pose_);

    // get the indices of the keyframes on the convex hull
    pcl::PointCloud<PointType>::Ptr convex_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
    convex_hull_.reconstruct(*convex_points);

    pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
    convex_hull_.getHullPointIndices(*convex_hull_point_idx);

    keyframe_convex_.clear();
    for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
        keyframe_convex_.push_back(convex_hull_point_idx->indices[i]);
    }
}

void LocalMap::ComputeConcaveHull()
{
    if(keyframes_pose_->size() > 5)
        return;

    // calculate the concave hull of the point cloud
    concave_hull_.setInputCloud(keyframes_pose_);

    // get the indices of the keyframes on the concave hull
    pcl::PointCloud<PointType>::Ptr concave_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
    concave_hull_.reconstruct(*concave_points);

    pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
    concave_hull_.getHullPointIndices(*concave_hull_point_idx);

    keyframe_concave_.clear();
    for (int i=0; i< concave_hull_point_idx->indices.size(); ++i) {
        keyframe_concave_.push_back(concave_hull_point_idx->indices[i]);
    }
}

void LocalMap::CreateNewBlock(const float &x, const float &y)
{
    block_size_++;
    int xth = x / block_scale_;
    int yth = y / block_scale_;

    float left = xth * block_scale_;
    float right = (xth + 1) * block_scale_;
    float top = yth * block_size_;
    float bottom = (yth + 1) * block_size_;

    if(x < left)
    {
        right = left;
        left = left - block_scale_;
    }
    else if(x >= right)
    {
        left = right;
        right = right + block_scale_;
    }

    if(y >= top)
    {
        bottom = top;
        top = top + block_scale_;
    }
    else if(y < bottom)
    {
        top = bottom;
        bottom = bottom - block_scale_;
    }

    ptrBlock_ = new Block(block_size_, left, right, top, bottom);
    ptrBlock_->SetBlockStatus(BlockStatus::Active);
    ptrBlock_->AddNewKeyFrameToBlock(ptrKeyFrame_);
}

void LocalMap::CreateNewKeyframe(const double &stamp, const Eigen::Matrix4f &pose, const pcl::PointCloud<PointType>::Ptr &cloud)
{
    //关键帧判断条件
    if(IsKeyframe(pose))
    {
        is_keyframe_flag_ = true;
        keyframe_size_++;
        //对点云下采样
        keyframes_map_ = DownSampling(voxel_size_, cloud);
        //对点云再次下采样，用于显示和重定位
        keyframe_for_view_ = DownSampling(voxel_view_size_, keyframes_map_);
        *map_for_view_ += *keyframe_for_view_;
        ptrKeyFrame_ = new Keyframe(keyframe_size_, stamp, pose, keyframes_map_);
    }
    else
    {
        is_keyframe_flag_ = false;
    }
}

void LocalMap::FindNewKeyFrameCondition(const Eigen::Matrix4f &currPose, const Eigen::Matrix4f &pose)
{
    float dist = sqrt(pow(currPose(0,3) - pose(0,3), 2) +
                      pow(currPose(1, 3) - pose(1,3) ,2) +
                      pow(currPose(2, 3) - pose(2,3) ,2));

    if(dist < 1.5 * keyframe_dist_thres_)
    {
        near_keyframe_num_++;
    }

    if(dist < keyframe_nearest_dist_)
    {
        keyframe_nearest_dist_ = dist;
        nearest_keyframe_pose_ = pose;
    }
}

bool LocalMap::IsKeyframe(const Eigen::Matrix4f &pose)
{
    if(!keyframe_size_)
    {
        //        inside_block_flag_ = false;
        return true;
    }
    else
    {
        //todo: 条件: 根据block算出来的最近的关键帧之间有个距离和角度之间的关系
        Eigen::Matrix4f deltaMat = pose * nearest_keyframe_pose_.inverse();
//        float angle =  * 180./M_PI;

//        Eigen::Quaternionf dq = this->rotq * (closest_pose_r.inverse());

//        float theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
//        float theta_deg = theta_rad * (180.0/M_PI);

//        // update keyframe
//        bool newKeyframe = false;

//        if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
//          newKeyframe = true;
//        }
//        if (abs(dd) <= this->keyframe_thresh_dist_) {
//          newKeyframe = false;
//        }
//        if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_ && num_nearby <= 1) {
//          newKeyframe = true;
//        }

        near_keyframe_num_ = 0;
        keyframe_nearest_dist_ = std::numeric_limits<float>::max();
        nearest_keyframe_pose_ = Eigen::Matrix4f::Identity();
    }
}

pcl::PointCloud<PointType>::Ptr LocalMap::DownSampling(const float &voxel, const pcl::PointCloud<PointType>::Ptr &cloud)
{
    pcl::PointCloud<PointType>::Ptr filterCloud(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> filter_;
    filter_.setLeafSize(voxel, voxel, voxel);
    filter_.setInputCloud(cloud);
    filter_.filter(*filterCloud);
    return filterCloud;
}


template<typename T>
std::vector<T> LocalMap::FindBlockKMinDistance(const std::vector<float> &dist, const T &k, const std::vector<T> &order)
{
    std::vector<T> result;

    if(dist.size() < k)
    {
        for(size_t i = 0; i < dist.size(); ++i)
        {
            result.emplace_back(order[i]);
        }
        return result;
    }

    std::priority_queue<float> topK;
    for(auto d: dist)
    {
        if(topK.size() >= k && topK.top() > d)
        {
            topK.push(d);
            topK.pop();
        }
        else {topK.push(d);}
    }
    float kthValue = topK.top();


    for(size_t i = 0; i < dist.size(); ++i)
    {
        if(dist[i] <= kthValue)
            result.emplace_back(order[i]);
    }
    return result;
}
