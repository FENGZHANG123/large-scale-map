#include "block.h"

using namespace std;

Block::Block(int block_id, const float &left, const float &right, const float top, const float bottom)
    :status_(BlockStatus::Empty)
    ,id_(block_id)
{
    left_bottom_corner_.x = left;
    left_bottom_corner_.y = bottom;
    right_top_corner_.x = right;
    right_top_corner_.y = top;

    center_corner_.x = (left + right) * 0.5;
    center_corner_.y = (top + bottom) * 0.5;
}

Block::~Block()
{
    keyframe_list_.clear();
}

bool Block::AddNewKeyFrameToBlock(Keyframe* keyframe)
{
    keyframe_list_.emplace_back(keyframe);
}

bool Block::LoadBlock(const std::string &path)
{
    string blocksKeyFrames = " ";
    for(auto keyframeIter : keyframe_list_)
    {
        blocksKeyFrames = path + "/" + boost::lexical_cast<string>(keyframeIter->GetKeyframeID()) + ".dat";
        keyframeIter->LoadPointCloud(blocksKeyFrames);
    }
}

bool Block::UnderBlock(const float &xpose, const float &ypose)
{
    if(left_bottom_corner_.x <= xpose && xpose < right_top_corner_.x &&
       left_bottom_corner_.y <= ypose && ypose < right_top_corner_.y )
    {
        return true;
    }

    return false;
}

void Block::SetBlockStatus(BlockStatus status)
{
    status_ = status;
}

void Block::GetBlockStatus(BlockStatus status)
{
    status = status_;
}

bool Block::SaveBlock(const std::string &path)
{
    string blocksKeyFrames = " ";
    for(auto keyframeIter : keyframe_list_)
    {
        blocksKeyFrames = path + "/" + boost::lexical_cast<string>(keyframeIter->GetKeyframeID()) + ".dat";
        keyframeIter->SavePointCloud(blocksKeyFrames);
    }
}

bool Block::ClearBlock()
{
    for(auto keyframeIter : keyframe_list_)
    {
        keyframeIter->ClearPointCloud();
    }
}

int Block::GetBlockID()
{
    return id_;
}
