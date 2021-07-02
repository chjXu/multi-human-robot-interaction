#include <human_pose/track.h>
#include <ros/ros.h>

using namespace HumanPose;

vector<vector<int>> Track::tracking(vector<HumanPose::Pose_Info>& framepose, int operator_index)
{
    //ROS_INFO("Tracking...\n");
    if(framepose.empty()){
        return {};
    }

    Skeletons availableSkeletons;
    availableSkeletons.frame_index = frame_index;
    availableSkeletons.skeletons_3d.resize(framepose.size());
    // 对传入的姿态进行编号和复制
    for(int i=0; i<availableSkeletons.skeletons_3d.size(); ++i){
        availableSkeletons.skeletons_3d[i].ID = -(i+1);
        availableSkeletons.skeletons_3d[i].Pose3D = framepose[i].pose_3d;
    }

    // 第一帧数据处理, 直接跳过，不做任何处理
    if(queue_pose.size() == 0){
        queue_pose.push_back(availableSkeletons);
        ++frame_index;
        // 对第一帧的姿态进行状态更新
        for(int i=0; i<queue_pose.size(); ++i){
            assert(queue_pose[i].frame_index == 1);
            for(int j=0; j<queue_pose[i].skeletons_3d.size(); ++j){
                queue_pose[i].skeletons_3d[j].paired = true;  
                if(j == operator_index)
                    queue_pose[i].skeletons_3d[j].ID = operator_index;
                else
                    queue_pose[i].skeletons_3d[j].ID = j;
            }
        }
    }
    else if(queue_pose.size() <= frameNum){
        if(queue_pose.size() == frameNum){
            queue_pose.erase(queue_pose.begin());
            //更新帧数
            for(int i=0; i<queue_pose.size(); ++i){
                queue_pose[i].frame_index -= 1;
            }
        }else if(queue_pose.size() == frameNum - 1){
            frame_index = frameNum;
        }
        else{
            ++frame_index;
        }
        
        queue_pose.push_back(availableSkeletons);
        //ROS_INFO("queue size is: %d", queue_pose.size());
        //ROS_INFO("Current queue has: %d frame, frame index is: %d", (int)queue_pose.size(), queue_pose[queue_pose.size()-1].frame_index);

        /**
         * 分为3种情况来讨论
         * 1. 每一帧之间的姿态数量是一样的， 这样匹配的效率最高，也是最容易的。
         * 2. 后一帧比前一帧多，那么后一帧的部分姿态与前一帧的姿态匹配完成后，剩余姿态要继续向前一帧进行搜索，
         * 直到配对成功，若无法配对，那么就要赋予一个独一无二的ID。
         * 3. 后一帧比前一帧少，这种情况也相对简单，直到匹配成功。
         * 
        */

        assert(queue_pose.size() >= 2);
        paired_frame = queue_pose.size() - 1;   //待匹配帧
        search_frame = queue_pose.size() - 2;   //搜索帧
        // 开始匹配
        //ROS_INFO("frame: %d, %d start pairing.", queue_pose[paired_frame].frame_index, queue_pose[search_frame].frame_index);

        Skeletons skele_1 = queue_pose[paired_frame]; //待匹配骨架
        Skeletons skele_2 = queue_pose[search_frame]; //搜索骨架

        // for(int i=0; i<skele_1.skeletons_3d.size(); ++i){
        //     ROS_INFO("pair frame %d pose's id is: %d", i, skele_1.skeletons_3d[i].ID);
        // }

        // for(int i=0; i<skele_2.skeletons_3d.size(); ++i){
        //     ROS_INFO("search frame %d pose's id is: %d", i, skele_2.skeletons_3d[i].ID);
        // }

        generateSkeletonsPair(skele_1, skele_2);
        
        vector<vector<int>>skele_as = skeletonsAssociate();   // 这里应该将错误的匹配关系删除， 并返回正确的配对关系,并更新姿态ID
        
        // for(int i=0; i<skele_as.size(); ++i){
        //     for(int j=0; j<skele_as[i].size(); ++j){
        //         cout << skele_as[i][j] << endl;
        //     }
        // }
   
        if(!skele_as.empty()){
            for(int i=0; i<skele_as.size(); ++i){
                queue_pose[paired_frame].skeletons_3d[skele_as[i][0]].paired = true;
            }
            setTrackingID(skele_as);
            // setTrackingID(skele_1, skele_2, skele_as);
        }else{
            // queue_pose.erase(queue_pose.end());
        }

        // 检查当前帧是否匹配完成， 如果没有，则search_frame-1且不能小于0；
        // 如果完成了， 则继续；
        // 套娃模式， 开始使用递归的算法。
        // return 未匹配的姿态的索引
        for(int i=0; i<queue_pose[paired_frame].skeletons_3d.size(); ++i){
            //停止条件
            while(!queue_pose[paired_frame].skeletons_3d[i].paired && search_frame > 0){
                //ROS_INFO("Recursion start.");
                search_frame--;
                Skeletons skele_3 = queue_pose[search_frame];
                generateSkeletonsPair(skele_1, skele_3);
                vector<vector<int>>skele_as_temp = skeletonsAssociate();
                if(!skele_as_temp.empty())
                    //setTrackingID(skele_1, skele_3, skele_as_temp);
                    setTrackingID(skele_as_temp);
                //else
                 //   continue;
            }
            //ROS_INFO("Recursion end.");
        }

        // 如果search_frame小于0，且当前匹配帧仍存在未配对的姿态，则将该姿态赋予一个独有的ID
        for(int i=0; i<queue_pose[paired_frame].skeletons_3d.size(); ++i){
            if(queue_pose[paired_frame].skeletons_3d[i].paired == false || queue_pose[paired_frame].skeletons_3d[i].ID < 0){
                //ROS_INFO("exist unpaired skeleton");
                queue_pose[paired_frame].skeletons_3d[i].ID = i;
                queue_pose[paired_frame].skeletons_3d[i].paired = true;
            }else{
                continue;
            }
            //ROS_INFO("update finish.");
        }


        // 统计id信息，防止因部分跟踪错误而出现的误分配
        map<int, int> index_with_id;
        int max_id;
        for(int i=0; i<queue_pose[paired_frame].skeletons_3d.size(); ++i){
            int id = queue_pose[paired_frame].skeletons_3d[i].ID;
            if(index_with_id.end() != index_with_id.find(id)){
                index_with_id[id]++;
                id++;
            }else{
                index_with_id.insert(pair<int, int>(id, 1));
            }

            // while(id >= queue_pose[paired_frame].skeletons_3d.size()){
            //     id--;
            // }
            queue_pose[paired_frame].skeletons_3d[i].ID = id; 
        }

        // for(auto it : index_with_id){
        //     cout << it.first << endl << it.second << endl;
        // }
    }

    // for(int i=0; i<queue_pose[paired_frame].skeletons_3d.size(); ++i){
    //     //ROS_INFO("cur_frame pose %d id is %d and state is: %d.", i, queue_pose[paired_frame].skeletons_3d[i].ID, queue_pose[paired_frame].skeletons_3d[i].paired);
    // }

    bool check = false;
    vector<vector<int>> as_results;
    vector<HumanPose::Pose3DWithId> skele_end = queue_pose[paired_frame].skeletons_3d;
    for(int i=0; i<skele_end.size(); ++i){
        if(skele_end[i].paired == true){
            vector<int> as_index_label;
            as_index_label.push_back(i);
            as_index_label.push_back(skele_end[i].ID);

            as_results.push_back(as_index_label);
            check = true;
        }else{
            check = false;
            continue;
        }
    }

    if(check){
        //ROS_INFO("Tracking over.");
        return as_results;
    }else{
        return {};
    }
}

void Track::setTrackingID(Skeletons &cur_frame, Skeletons& search_frame, vector<vector<int>>& skele_as){
    ROS_INFO("skele_as: %d", (int)skele_as.size());
    map<int, int> pose_id;

    for(int i=0; i<skele_as.size(); ++i){
        ROS_INFO("search pose %d id is %d.", skele_as[i][1], search_frame.skeletons_3d[skele_as[i][1]].ID);
        cur_frame.skeletons_3d[skele_as[i][0]].ID = search_frame.skeletons_3d[skele_as[i][1]].ID;
        cur_frame.skeletons_3d[skele_as[i][0]].paired = true;
    }

    for(int i=0; i<cur_frame.skeletons_3d.size(); ++i){
        ROS_INFO("cur_frame pose %d id is %d and state is: %d.", i, cur_frame.skeletons_3d[i].ID, cur_frame.skeletons_3d[i].paired);
    }
}

void Track::setTrackingID(vector<vector<int>>& skele_as){
    //ROS_INFO("skele_as: %d", (int)skele_as.size());

    for(int i=0; i<skele_as.size(); ++i){
        //ROS_INFO("search pose %d id is %d.", skele_as[i][1], queue_pose[search_frame].skeletons_3d[skele_as[i][1]].ID);
        queue_pose[paired_frame].skeletons_3d[skele_as[i][0]].ID = queue_pose[search_frame].skeletons_3d[skele_as[i][1]].ID;
        queue_pose[paired_frame].skeletons_3d[skele_as[i][0]].paired = true;
    }
}

void Track::generateSkeletonsPair(Skeletons &cur_frame, Skeletons& search_frame){
    //ROS_INFO("generating pair, cur has %d pose, and search has %d pose", (int)cur_frame.skeletons_3d.size(), (int)search_frame.skeletons_3d.size());
    skeleton_pair.clear();

    // for(int i=0; i<search_frame.skeletons_3d.size(); ++i){
    //     ROS_INFO("pose state: %d", search_frame.skeletons_3d[i].paired);
    // }

    if(cur_frame.frame_index - search_frame.frame_index < frameNum){
        for(int i=0; i<cur_frame.skeletons_3d.size(); ++i){
            if(cur_frame.skeletons_3d[i].paired == false){
                for(int j=0; j<search_frame.skeletons_3d.size();++j){
                    assert(search_frame.skeletons_3d[j].paired == true);
                    //ROS_INFO("cur: %d, search: %d will generate pair", i, j);
                    SkeletonPair s_pair;
                    s_pair.cur_frame_index = cur_frame.frame_index;
                    s_pair.sea_frame_index = search_frame.frame_index;
                    s_pair.index_1 = i;
                    s_pair.index_2 = j;
                    skeleton_pair.push_back(s_pair);
                }
            }            
        }
    }
    //ROS_INFO("Generate pair: %d", (int)skeleton_pair.size());
}

void* skeletons_thread(void* args){
    track_args* m_args = (track_args*)args;
    m_args->obj->getSkeletonsCorrespondence(*(m_args->pair));
}

vector<vector<int>> Track::skeletonsAssociate(){
    // ROS_INFO("Associating");
    // pthread_t track_thread[skeleton_pair.size()];
    // track_args t_args[skeleton_pair.size()];
    // int i=0;

    // for(auto it = skeleton_pair.begin(); it != skeleton_pair.end(); ++it){
    //     t_args[i] = {this, it};
    //     int ret = pthread_create(&track_thread[i], NULL, skeletons_thread, &(t_args[i]));
    //     if(ret == 0){
    //         continue;
    //     }
    //     ++i;
    // }

    // for(int i=0; i<skeleton_pair.size(); ++i){
    //     pthread_join(track_thread[i], NULL);
    // }

    for(auto it = skeleton_pair.begin(); it != skeleton_pair.end(); ++it){
        getSkeletonsCorrespondence(*it);
    }

    double threshold = 0.5;
    vector<vector<int>> skele_as;
    auto rank_min = min_element(skeleton_pair.begin(), skeleton_pair.end(), SkeletonPair::comp);
    // vector<int> as_min;
    // as_min.push_back(rank_min->index_1);
    // as_min.push_back(rank_min->index_2);
    // skele_as.push_back(as_min);

    
    while(!skeleton_pair.empty() && rank_min->delta <= threshold){
        // auto rank_min = min_element(skeleton_pair.begin(), skeleton_pair.end(), SkeletonPair::comp);

        // int pair_id = 1;
        // for(auto it=skeleton_pair.begin(); it!=skeleton_pair.end(); ++it){
        //     ROS_INFO("pair %d: frame %d:%d , index %d:%d, score:%.2f", pair_id, it->cur_frame_index, it->sea_frame_index, it->index_1, it->index_2, it->delta);
        //     ++pair_id;
        // }
        // ROS_INFO("pair: %d, %d is min.", rank_min->index_1, rank_min->index_2);

        int id_1 = rank_min->index_1;
        int id_2 = rank_min->index_2;
        vector<int> as_new_min;
        as_new_min.push_back(id_1);
        as_new_min.push_back(id_2);
        skele_as.push_back(as_new_min);

        skeleton_pair.erase(rank_min);
        for(auto it=skeleton_pair.begin(); it!=skeleton_pair.end(); ){
            if(it->index_1 == id_1 || it->index_2 == id_2){
                //ROS_INFO("pair: %d:%d erased", it->index_1, it->index_2);
                it = skeleton_pair.erase(it);
            }else{
                ++it;
            }


            // if((it->index_1 == id_1 && it->index_2 != id_2) || (it->index_1 != id_1 && it->index_2 == id_2)){
            //     ROS_INFO("pair: %d:%d erased", it->index_1, it->index_2);
            //     it = skeleton_pair.erase(it);
            // }
        }
        if(!skeleton_pair.empty())
            rank_min = min_element(skeleton_pair.begin(), skeleton_pair.end(), SkeletonPair::comp);
        else 
            break;
    }

    // 在实际中，每2帧之间的匹配度应该为0，但是考虑前帧也将进行匹配。所以，我们还要设置一个阈值。
    // for(auto it = skeleton_pair.begin(); it != skeleton_pair.end();){
    //     if(it->delta >= threshold){
    //         ROS_INFO("pair: %d:%d erased", it->index_1, it->index_2);
    //         it = skeleton_pair.erase(it);
    //     }
    //     else
    //     {
    //         ++it;
    //     }
    // }
    
    // 返回值
    if(!skele_as.empty()){
        // vector<vector<int>> skele_as;
        // for(auto skele_pair = skeleton_pair.begin(); skele_pair != skeleton_pair.end(); ++skele_pair){
        //     vector<int> as_id;
        //     as_id.push_back(skele_pair->index_1);
        //     as_id.push_back(skele_pair->index_2);
        //     skele_as.push_back(as_id);
        // }
        return skele_as;

    }else{
        return {};
    }

}

bool Track::getSkeletonsCorrespondence(SkeletonPair& pair, double threshold){
    pthread_mutex_trylock(&track_mutex);
    HumanPose::Pose3DWithId skeletons_1 = queue_pose[pair.cur_frame_index - 1].skeletons_3d[pair.index_1];
    HumanPose::Pose3DWithId skeletons_2 = queue_pose[pair.sea_frame_index - 1].skeletons_3d[pair.index_2];
    pthread_mutex_unlock(&track_mutex);

    double delta = 0.0;
    assert(skeletons_1.Pose3D.size() == skeletons_2.Pose3D.size());
    
    //vector<int> iou_1 = getIOU(skeletons_1.Pose3D);
    //ROS_INFO("x_min: %d, y_min: %d, z_min: %d, x_max: %d, y_max: %d, z_max: %d", iou_1[0], iou_1[1], iou_1[2], iou_1[3], iou_1[4], iou_1[5]);
    
    double total_score = 0.0;
    for(int i=0; i<skeletons_1.Pose3D.size(); ++i){
        total_score += skeletons_1.Pose3D[i].p * skeletons_2.Pose3D[i].p;
        delta += getDistance(skeletons_1.Pose3D[i], skeletons_2.Pose3D[i]) * skeletons_1.Pose3D[i].p * skeletons_2.Pose3D[i].p;
    }

    if(total_score == 0){
        return false;
    }else{
        pair.delta = delta/total_score;
    }

    if(pair.delta <= threshold){
        return true;
    }else
    {
        return false;
    }
    
}

double Track::getDistance(Pose_3d& joint_1, Pose_3d& joint_2){
    return std::sqrt((joint_1.x - joint_2.x) * (joint_1.x - joint_2.x) + 
                     (joint_1.y - joint_2.y) * (joint_1.y - joint_2.y) +
                     (joint_1.z - joint_2.z) * (joint_1.z - joint_2.z));
}

vector<int> Track::getIOU(vector<HumanPose::Pose_3d>& pose_3d){
    if(pose_3d.empty()){
        return {};
    }

    vector<int> bbox(6);
    int x_min=pose_3d[0].x, y_min=pose_3d[0].y, z_min=pose_3d[0].z;
    int x_max=pose_3d[0].x, y_max=pose_3d[0].y, z_max=pose_3d[0].z;

    for(int i=1; i<pose_3d.size(); ++i){
        if(x_min > pose_3d[i].x){
            x_min = pose_3d[i].x;
        }
        if(x_max < pose_3d[i].x){
            x_max < pose_3d[i].x;
        }
        if(y_min > pose_3d[i].y){
            y_min = pose_3d[i].y;
        }
        if(y_max < pose_3d[i].y){
            y_max < pose_3d[i].y;
        }
        if(z_min > pose_3d[i].z){
            z_min = pose_3d[i].z;
        }
        if(z_max < pose_3d[i].z){
            z_max < pose_3d[i].z;
        }
    }

    bbox[0] = x_min;
    bbox[1] = y_min;
    bbox[2] = z_min;
    bbox[3] = x_max;
    bbox[4] = y_max;
    bbox[5] = z_max;

    return bbox;
}

// void Track::updateSkeletons(){
//     // 从最后的3帧中，使得结果更加柔滑
//     if(queue_pose.size() < 3){
//         return
//     }

//     for(int i=queue_pose.size(); i > queue_pose.size()-3; --i){
//         Skeletons poses = queue_pose[i].skeletons_3d;
//         for(int j=0; j<poses.skeletons_3d.size(); ++j){

//         }
//     }
// }
