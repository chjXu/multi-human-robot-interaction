#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include <utility>
#include <iostream>

#include "human_pose.hpp"
using namespace std;

namespace human_pose_estimation{
    const std::vector<std::pair<int, int> > limbIdsHeatmap = {
        {2, 3}, {2, 6}, {3, 4}, {4, 5}, {6, 7}, {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12}, {12, 13}, {13, 14},
        {2, 1}, {1, 15}, {15, 17}, {1, 16}, {16, 18}, {3, 17}, {6, 18}
    };
    const std::vector<std::pair<int, int> > limbIdsPaf = {
        {31, 32}, {39, 40}, {33, 34}, {35, 36}, {41, 42}, {43, 44}, {19, 20}, {21, 22}, {23, 24}, {25, 26},
        {27, 28}, {29, 30}, {47, 48}, {49, 50}, {53, 54}, {51, 52}, {55, 56}, {37, 38}, {45, 46}
    };

    // statistic bone length
    const std::vector<float> bone_length = {26.42178982, 48.36980909,
                                            14.88291009, 31.28002332, 23.915707,
                                            14.97674918, 31.28002549, 23.91570732,
                                            12.4644364,  48.26604433, 39.03553194,
                                            12.4644364, 48.19076948, 39.03553252};


    class HumanPoseByPeaksIndices
    {
    public:
        HumanPoseByPeaksIndices(const int keypointsNumber):
                                peaksIndices(std::vector<int>(keypointsNumber, -1)), nJoints(0), score(0.0f){}

        std::vector<int> peaksIndices;
        int nJoints;
        float score;

    private:

    };

    class TwoJointConnection
    {
     public:
        TwoJointConnection(const int firstJointIdx, const int secondJointIdx, const float score):
                            firstJointIdx(firstJointIdx), secondJointIdx(secondJointIdx), score(score){}
        int firstJointIdx;
        int secondJointIdx;
        float score;
     private:

    };

    class Peak
    {
    public:
        Peak(const int id = 0, const cv::Point2f& pos = cv::Point2f(), const float& score = 0.0f);

        int id;
        cv::Point2f pos;
        float score;

        friend ostream& operator<<(ostream& os, const Peak& p);
    private:

    };

    Peak::Peak(const int id, const cv::Point2f& pos, const float& score):
        id(id), pos(pos), score(score){}

    ostream& operator << (ostream& os, const Peak& p){
            os << "id: " << p.id << "  " << "pos: " << p.pos << "  " << "score: " << p.score;
            return os;
        }

    void findPeaks(const std::vector<cv::Mat>& heatmaps, std::vector<std::vector<Peak>>& peakFromHeatmap,
                   const float& minPeaksDistance, int heatmapIndex)
    {
        const float threshold = 0.1f;
        std::vector<cv::Point> peaks;
        const cv::Mat& heatMap = heatmaps[heatmapIndex];
        const float* heatMapData = heatMap.ptr<float>();
        size_t heatMapStep = heatMap.step1();   //获取每一行的元素总个数   128

        for(int y = -1; y < heatMap.rows+1; ++y){
            for(int x = -1; x < heatMap.cols+1; ++x){
                float val = 0;
                if(x >= 0 && y >= 0 && x < heatMap.cols && y < heatMap.rows){
                    val = heatMapData[y * heatMapStep + x];
                    val = val > threshold? val : 0;
                }

                float left_val = 0;
                if(y >= 0 && x < heatMap.cols - 1 && y < heatMap.rows){
                    left_val = heatMapData[y * heatMapStep + x + 1];
                    left_val = left_val > threshold? left_val : 0;
                }

                float right_val = 0;
                if(x > 0 && y >= 0 && y < heatMap.rows){
                    right_val = heatMapData[y * heatMapStep + x - 1];
                    right_val = right_val > threshold? right_val : 0;
                }

                float top_val = 0;
                if(x >= 0 && x < heatMap.cols && y < heatMap.rows - 1){
                    top_val = heatMapData[(y+1) * heatMapStep + x];
                    top_val = top_val > threshold? top_val : 0;
                }

                float bottom_val = 0;
                if(x >= 0 && y > 0 && x < heatMap.cols){
                    bottom_val = heatMapData[(y-1) * heatMapStep + x];
                    bottom_val = bottom_val > threshold? bottom_val : 0;
                }

                if((val > left_val) && (val > right_val) && (val > top_val) && (val > bottom_val)){
                    peaks.push_back(cv::Point(x, y));
                }
            }
        }

        std::sort(peaks.begin(), peaks.end(), [](const cv::Point& a, const cv::Point& b){
            return a.x < b.x;
        });

        std::vector<bool> isActualPeaks(peaks.size(), true);
        int peakCounter = 0;
        std::vector<Peak>& peakWithScoreAndId = peakFromHeatmap[heatmapIndex];
        // std::cout << "peak size: " << peaks.size() << std::endl;
        for(size_t i=0; i<peaks.size(); ++i){
            if(isActualPeaks[i]){
                for(size_t j = i+1; j<peaks.size(); ++j){
                    if(std::sqrt((peaks[i].x - peaks[j].x) * (peaks[i].x - peaks[j].x) +
                                (peaks[i].y - peaks[j].y) * (peaks[i].y - peaks[j].y)) < minPeaksDistance){
                                    isActualPeaks[j] = false;
                                }
                }
                peakWithScoreAndId.push_back(Peak(peakCounter++, peaks[i], heatMap.at<float>(peaks[i])));
            }
        }
    }

    std::vector<HumanPose> groupPeaksToPoses(const std::vector<std::vector<Peak> >& allPeaks,
                                             const std::vector<cv::Mat>& pafs,
                                             const size_t keypointsNumber,
                                             const float midPointsScoreThreshold,
                                             const float foundMidPointsRatioThreshold,
                                             const int minJointsNumber,
                                             const float minSubsetScore)
    {
        std::vector<Peak> candidates;
        for(const auto& peaks : allPeaks){
            candidates.insert(candidates.end(), peaks.begin(), peaks.end());
        }

        std::vector<HumanPoseByPeaksIndices> subset(0, HumanPoseByPeaksIndices(static_cast<int>(keypointsNumber)));
        for(size_t k = 0; k < limbIdsPaf.size(); k++){
            std::vector<TwoJointConnection> connections;
            const int mapIdxOffset = static_cast<int>(keypointsNumber) + 1;
            std::pair<cv::Mat, cv::Mat> scoreMid = {pafs[limbIdsPaf[k].first - mapIdxOffset], pafs[limbIdsPaf[k].second - mapIdxOffset]};
            const int idxJointA = limbIdsHeatmap[k].first - 1;
            const int idxJointB = limbIdsHeatmap[k].second - 1;
            const std::vector<Peak>& candA = allPeaks[idxJointA];
            const std::vector<Peak>& candB = allPeaks[idxJointB];
            const size_t nJointsA = candA.size();
            const size_t nJointsB = candB.size();
            if(nJointsA == 0 && nJointsB == 0){
                continue;
            }
            else if(nJointsA == 0){
                for(size_t i=0; i < nJointsB; ++i){
                    int num = 0;
                    for(size_t j=0; j < subset.size(); j++){
                        if(subset[j].peaksIndices[idxJointB] == candB[i].id){
                            num++;
                            continue;
                        }
                    }

                    if(num == 0){
                        HumanPoseByPeaksIndices personKeypoints(static_cast<int>(keypointsNumber));
                        personKeypoints.peaksIndices[idxJointB] = candB[i].id;
                        personKeypoints.nJoints = 1;
                        personKeypoints.score = candB[i].score;
                        subset.push_back(personKeypoints);
                    }
                }
                continue;
            }
            else if(nJointsB == 0){
                for (size_t i = 0; i < nJointsA; i++) {
                    int num = 0;
                    for (size_t j = 0; j < subset.size(); j++) {
                        if (subset[j].peaksIndices[idxJointA] == candA[i].id) {
                            num++;
                            continue;
                        }
                    }
                    if (num == 0) {
                        HumanPoseByPeaksIndices personKeypoints(static_cast<int>(keypointsNumber));
                        personKeypoints.peaksIndices[idxJointA] = candA[i].id;
                        personKeypoints.nJoints = 1;
                        personKeypoints.score = candA[i].score;
                        subset.push_back(personKeypoints);
                    }
                }
                continue;
            }


            std::vector<TwoJointConnection> tempJointConnections;
            for (size_t i = 0; i < nJointsA; i++) {
                for (size_t j = 0; j < nJointsB; j++) {
                    cv::Point2f pt = candA[i].pos * 0.5 + candB[j].pos * 0.5;
                    cv::Point mid = cv::Point(cvRound(pt.x), cvRound(pt.y));
                    cv::Point2f vec = candB[j].pos - candA[i].pos; //vector
                    double norm_vec = cv::norm(vec);    //vector normalize
                    if (norm_vec == 0) {
                        continue;
                    }
                    vec /= norm_vec;    //unit vector
                    float score = vec.x * scoreMid.first.at<float>(mid) + vec.y * scoreMid.second.at<float>(mid); //the socre of keypoints
                    //std::cout << "socre: " << score << std::endl;
                    int height_n  = pafs[0].rows / 2; //64
                    //std::cout << "height: " << height_n << std::endl;
                    float suc_ratio = 0.0f;
                    float mid_score = 0.0f;
                    const int mid_num = 10;
                    const float scoreThreshold = -100.0f;
                    if (score > scoreThreshold) {
                        float p_sum = 0;
                        int p_count = 0;
                        cv::Size2f step((candB[j].pos.x - candA[i].pos.x)/(mid_num - 1),
                                        (candB[j].pos.y - candA[i].pos.y)/(mid_num - 1));
                        for (int n = 0; n < mid_num; n++) {
                            cv::Point midPoint(cvRound(candA[i].pos.x + n * step.width),
                                               cvRound(candA[i].pos.y + n * step.height));
                            cv::Point2f pred(scoreMid.first.at<float>(midPoint),
                                             scoreMid.second.at<float>(midPoint));
                            score = vec.x * pred.x + vec.y * pred.y;
                            if (score > midPointsScoreThreshold) {
                                p_sum += score;
                                p_count++;
                            }
                        }
                        suc_ratio = static_cast<float>(p_count / mid_num);
                        float ratio = p_count > 0 ? p_sum / p_count : 0.0f; //average pose score
                        mid_score = ratio + static_cast<float>(std::min(height_n / norm_vec - 1, 0.0));
                    }
                    if (mid_score > 0 && suc_ratio > foundMidPointsRatioThreshold) {
                        //std::cout << i << " " << j << " " << mid_score << std::endl;
                        tempJointConnections.push_back(TwoJointConnection(static_cast<int>(i), static_cast<int>(j), mid_score));
                    }
                }
            }

            if (!tempJointConnections.empty()) {
                std::sort(tempJointConnections.begin(), tempJointConnections.end(), [](const TwoJointConnection& a, const TwoJointConnection& b) {
                    return a.score > b.score;
                });
            }
            size_t num_limbs = std::min(nJointsA, nJointsB);
            size_t cnt = 0;
            std::vector<int> occurA(nJointsA, 0);
            std::vector<int> occurB(nJointsB, 0);
            for (size_t row = 0; row < tempJointConnections.size(); row++) {
                if (cnt == num_limbs) {
                    break;
                }
                const int& indexA = tempJointConnections[row].firstJointIdx;
                const int& indexB = tempJointConnections[row].secondJointIdx;
                const float& score = tempJointConnections[row].score;   //paf score
                if (occurA[indexA] == 0
                        && occurB[indexB] == 0) {
                    connections.push_back(TwoJointConnection(candA[indexA].id, candB[indexB].id, score));
                    cnt++;
                    occurA[indexA] = 1;
                    occurB[indexB] = 1;
                }
            }
            if (connections.empty()) {
                continue;
            }

            bool extraJointConnections = (k == 17 || k == 18);
            if (k == 0) {
                subset = std::vector<HumanPoseByPeaksIndices>(
                            connections.size(), HumanPoseByPeaksIndices(static_cast<int>(keypointsNumber)));
                for (size_t i = 0; i < connections.size(); i++) {
                    const int& indexA = connections[i].firstJointIdx;
                    const int& indexB = connections[i].secondJointIdx;
                    subset[i].peaksIndices[idxJointA] = indexA;
                    subset[i].peaksIndices[idxJointB] = indexB;
                    subset[i].nJoints = 2;
                    subset[i].score = candidates[indexA].score + candidates[indexB].score + connections[i].score;
                }
            } else if (extraJointConnections) {
                for (size_t i = 0; i < connections.size(); i++) {
                    const int& indexA = connections[i].firstJointIdx;
                    const int& indexB = connections[i].secondJointIdx;
                    for (size_t j = 0; j < subset.size(); j++) {
                        if (subset[j].peaksIndices[idxJointA] == indexA
                                && subset[j].peaksIndices[idxJointB] == -1) {
                            subset[j].peaksIndices[idxJointB] = indexB;
                        } else if (subset[j].peaksIndices[idxJointB] == indexB
                                    && subset[j].peaksIndices[idxJointA] == -1) {
                            subset[j].peaksIndices[idxJointA] = indexA;
                        }
                    }
                }
                continue;
            } else {
                for (size_t i = 0; i < connections.size(); i++) {
                    const int& indexA = connections[i].firstJointIdx;
                    const int& indexB = connections[i].secondJointIdx;
                    bool num = false;
                    for (size_t j = 0; j < subset.size(); j++) {
                        if (subset[j].peaksIndices[idxJointA] == indexA) {
                            subset[j].peaksIndices[idxJointB] = indexB;
                            subset[j].nJoints++;
                            subset[j].score += candidates[indexB].score + connections[i].score;
                            num = true;
                        }
                    }
                    if (!num) {
                        HumanPoseByPeaksIndices hpWithScore(static_cast<int>(keypointsNumber));
                        hpWithScore.peaksIndices[idxJointA] = indexA;
                        hpWithScore.peaksIndices[idxJointB] = indexB;
                        hpWithScore.nJoints = 2;
                        hpWithScore.score = candidates[indexA].score + candidates[indexB].score + connections[i].score;
                        subset.push_back(hpWithScore);
                    }
                }
            }
        }

        std::vector<HumanPose> poses;
        for (const auto& subsetI : subset) {
            if (subsetI.nJoints < minJointsNumber
                    || subsetI.score / subsetI.nJoints < minSubsetScore) {
                continue;
            }
            int position = -1;
            HumanPose pose(std::vector<cv::Point3f>(keypointsNumber, cv::Point3f(-1.0f, -1.0f, -1.0f)),
                            subsetI.score * std::max(0, subsetI.nJoints - 1));
            for (const auto& peakIdx : subsetI.peaksIndices) {
                position++;
                if (peakIdx >= 0) {
                    pose.keypoints[position].x = candidates[peakIdx].pos.x + 0.5f;
                    pose.keypoints[position].y = candidates[peakIdx].pos.y + 0.5f;
                    pose.keypoints[position].z = candidates[peakIdx].score;
                }
            }
            poses.push_back(pose);
        }

//        for(auto& human_id:poses){
//            for(int i=0; i<human_id.keypoints.size(); ++i){
//                std::cout << "Point id: " << i << "  Point: " << human_id.keypoints[i] << "  score: " << human_id.scores << std::endl;
//            }
//            //std::cout << human << std::endl;
//        }
        return poses;
    }
}