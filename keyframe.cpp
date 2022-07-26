#include "keyframe.h"

using namespace std;

Keyframe::Keyframe(int64_t id, const double &stamp, const Eigen::Matrix4f &pose,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    :id_(id)
    ,stamp_(stamp)
    ,pose_(pose)
    ,cloud_(cloud)
{
}

Keyframe::~Keyframe()
{
    cloud_->clear();
}

int64_t Keyframe::GetKeyframeID()
{
    return id_;
}

void Keyframe::ClearPointCloud()
{
    cloud_->clear();
}

void Keyframe::SavePointCloud(const std::string &path)
{
    DatIO datIO;
    if(datIO.SaveBinaryDatFile(path, cloud_))
    {
        cout<< "save " << path << " succeed." << endl;
    }
    else{
        cout << "save "<< path << "failed!" <<endl;
    }
}

void Keyframe::LoadPointCloud(const std::string &path)
{
    //有无文件夹的判断
    //有无点云的判断
    DatIO datIO;
    if(datIO.ReadBinaryDatFile(path, cloud_))
    {
        cout << "load " << path << " succeed." << endl;
    }
    else {
        cout << "load "<< path << " failed!" << endl;
    }
}

//pcl::PointCloud<PointType>::Ptr Keyframe::DownSampling(const pcl::PointCloud<PointType>::ConstPtr &cloud)
//{

//}
