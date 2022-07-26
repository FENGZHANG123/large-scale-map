/** \brief: 数据保存读取都通过DAT的格式，包含pcd的格式转换
 * \author: 张锋
 * \date: 2022.7.18
*/

#ifndef DAT_IO_H_
#define DAT_IO_H_

#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

class DatIO
{
public:
    DatIO();

    ~DatIO();

    bool SaveBinaryDatFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    bool ReadBinaryDatFile(const std::string &path, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    void SavePclPcd(const std::string &path, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        pcl::io::savePCDFileBinary(path, *cloud);
    }

    void ReadPclPcd(const std::string &path, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud) == -1)
        {
          std::cerr << "can't load point cloud file " << path << std::endl;
        }
    }

private:
    bool ValidateFileFormat(const std::string &path);
};

#endif
