#include "dat_io.h"

using namespace std;

DatIO::DatIO()
{

}

DatIO::~DatIO()
{

}

bool DatIO::SaveBinaryDatFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    if(!ValidateFileFormat(path))
    {
        std::cout << path << " not found!" << std::endl;
        return false;
    }

    fstream cloudfile(path, fstream::out | fstream::binary);

    if(!cloudfile.is_open())
    {
        std::cout << "open " << path << " failed!" << endl;
        cloudfile.close();
        return false;
    }

    //两种方式
    uint32_t cloudSize = 4*cloud->size();

#if 0
    std::vector<float> cloudV;
    cloudV.reserve(cloudSize);

    for(size_t i = 0; i < cloud->size(); ++i)
    {
        cloudV.emplace_back(cloud->points[i].x);
        cloudV.emplace_back(cloud->points[i].y);
        cloudV.emplace_back(cloud->points[i].z);
        cloudV.emplace_back(cloud->points[i].intensity);
    }

    cloudfile.write(reinterpret_cast<char *>(&cloudV[0]), cloudSize*sizeof(float));


//#else if 0
//    float buf[cloudSize];
//    memcpy(&buf[0], &(cloud->points[0].x), sizeof(cloud->points));
//    cloudfile.write(reinterpret_cast<char *>(&buf[0]), cloudSize*sizeof(float));
#else
    size_t val = sizeof (float) * 3;
//    char out[val];
//    char *buf = &out + val * cloud->size();
    for(size_t i = 0; i < cloud->size(); ++i)
    {
//        memcpy(&out, reinterpret_cast<const char *>(&cloud->points[i]), val);
        cloudfile.write(reinterpret_cast<const char *>(&cloud->points[i]), val);
        cloudfile.write(reinterpret_cast<const char *>(&cloud->points[i].intensity), sizeof (float));
    }

#endif    //两种方式
    cloudfile.close();
    return true;
}

bool DatIO::ReadBinaryDatFile(const std::string &path, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    if(!ValidateFileFormat(path))
    {
        cout << path << " not found!" << endl;
        return false;
    }

    std::ifstream cloud_file(path, std::ios::binary);
    if(!cloud_file)
    {
        cout << path << " open failed!" << endl;
        return false;
    }

    cloud_file.seekg(0, cloud_file.end);
    uint32_t len = (uint32_t)cloud_file.tellg();
    cloud_file.seekg(0, cloud_file.beg);

    uint32_t num = len / sizeof (float);
    std::vector<float> buffer(num);
    cloud_file.read(reinterpret_cast<char *>(&buffer[0]), num*sizeof(float));
    cloud_file.close();

    pcl::PointXYZI point;
    cloud->points.reserve(num/4);

    for(uint32_t i = 0; i < num; i += 4)
    {
        point.x = buffer[i];
        point.y = buffer[i+1];
        point.z = buffer[i+2];
        point.intensity = buffer[i+3];
        cloud->points.emplace_back(point);
    }
    return true;
}

bool DatIO::ValidateFileFormat(const std::string &path)
{
    std::size_t found = path.find(".dat");
    if(found != path.npos)
    {
        return true;
    }

    return false;
}

