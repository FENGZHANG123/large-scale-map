#ifndef BLOCK_H_
#define BLOCK_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include "keyframe.h"

#include <boost/lexical_cast.hpp>

//typedef pcl::PointXYZI PointType;

enum class BlockStatus
{
    Empty,
    Active,
    Inactive,
};

struct Corner
{
    float x;
    float y;
};

class Block
{
public:
    Block(int block_id, const float &left, const float &right,
          const float top, const float bottom);

    virtual ~Block();

public:
    /** \brief: 把关键帧放到block内*/
    bool AddNewKeyFrameToBlock(Keyframe* keyframe);

    bool LoadBlock(const std::string &path);

    /** \brief: 判断是否在blcok范围内*/
    bool UnderBlock(const float &xpose, const float &ypose);

    void SetBlockStatus(BlockStatus status);

    void GetBlockStatus(BlockStatus status);

//    bool SearchBlocksKeyframe();

    bool SaveBlock(const std::string &path);

    bool ClearBlock();

    int GetBlockID();

private:
    //block的状态，default是Empty, active是正在使用，inactive是保存或者不用的状态
    BlockStatus status_;

    int id_;

    /** 形状如下，左下都是<=, 右上都是<
       ————————
      |        |
      |        |
      |        |
       --------   */
    Corner left_bottom_corner_; //左小右大； 上大下小

    Corner right_top_corner_;

public:
    std::vector<Keyframe*> keyframe_list_;

    Corner center_corner_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
