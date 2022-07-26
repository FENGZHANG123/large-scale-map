#include "dat_io.h"
#include "tic_toc.h"

using namespace std;

int main(int argc, char** argv)
{
    //读写是否有误的测试；
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;
    cloud->reserve(100000000);
    for(size_t i = 0; i < 100000000; ++i)
    {
        float value = (float)i;
        point.x = value;
        point.y = value;
        point.z = value;
        point.intensity= value;
        cloud->points.emplace_back(point);
    }

    size_t len = sizeof(pcl::PointXYZI);
    cout << "len: " << len << endl;

    //dat写
    DatIO datIO;
    TicToc timer;
    datIO.SaveBinaryDatFile("/home/zf/Project/dat_test/test/test.dat", cloud);
    cout << "dat save time spent: " << timer.toc() << endl;



    //和binary ascii的pcd测试；
    timer.tic();
    datIO.SavePclPcd("/home/zf/Project/dat_test/test/test.pcd", cloud);
    cout << "pcd save time spent: " <<  timer.toc() << endl;

    //dat读
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudR(new pcl::PointCloud<pcl::PointXYZI>);
    timer.tic();
    datIO.ReadBinaryDatFile("/home/zf/Project/dat_test/test/test.dat", cloudR);
    cout << "dat read time spent: " << timer.toc() << endl<< endl;
    cout  << "dat test: " << endl;
//    for(size_t i = 0; i < cloudR->size(); ++i)
//    {
//        cout << i << " point: " << cloudR->points[i].x << " " << cloudR->points[i].y << " "<<
//                cloudR->points[i].z << " " << cloudR->points[i].intensity << endl;
//    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZI>);
    timer.tic();
    datIO.ReadPclPcd("/home/zf/Project/dat_test/test/test.pcd", cloudS);
    cout << "pcd read time spent: " << timer.toc() <<endl << endl;

//    cout << "pcd test: " << endl;
//    for(size_t i = 0; i < cloudS->size(); ++i)
//    {
//        cout << i << " point: " << cloudS->points[i].x << " " << cloudS->points[i].y << " "<<
//                cloudS->points[i].z << " " << cloudS->points[i].intensity << endl;
//    }

    return 0;
}
