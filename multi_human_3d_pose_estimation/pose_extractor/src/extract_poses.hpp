#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "human_pose.hpp"
#include "peak.hpp"

using namespace std;

namespace human_pose_estimation{

    void showHeatMaps(std::vector<cv::Mat>& heatmap, std::vector<cv::Mat>& pafs){
        if (heatmap.empty() || pafs.empty()){
            return ;
        }

        //for(auto &heat : heatmap){
            cv::resize(heatmap[1], heatmap[1], cv::Size(1920, 1080));
            cv::cvtColor(heatmap[1], heatmap[1], cv::COLOR_GRAY2RGB);
            //cout << heatmap[0].channels() << endl;
            cv::imwrite("heatmap.jpg", heatmap[1]*255);
            cv::imshow("heatmap", heatmap[1]);
            cv::waitKey(3);
        //}

        //for(auto &paf : pafs){
            cv::resize(pafs[1], pafs[1], cv::Size(1920, 1080));
            cv::cvtColor(pafs[1], pafs[1], cv::COLOR_GRAY2RGB);
            //cout << heatmap[0].channels() << endl;
            cv::imwrite("pafs.jpg", pafs[1]*255);
            cv::imshow("paf", pafs[1]);
            cv::waitKey(0);
        //}

    }


    /**
    * function: resize heatmap
    * featuresMap: input heatmap
    * up_ratio: extension ratio
    */
    static void resizeFeaturesMap(std::vector<cv::Mat>& featuresMaps, int up_ratio){
        for(auto &featuresMap : featuresMaps){
            cv::resize(featuresMap, featuresMap, cv::Size(),
                       up_ratio, up_ratio, cv::INTER_CUBIC);
        }
    }

    /**
    * function: find every joint's peak from heatmap, and save in vector peaksFromHeatmap.
    * heatmap: input image
    * peaksFromHeatmap: save vector
    * minPeakDistance: correspondence threshold
    */
    class FindPeakBody : public cv::ParallelLoopBody
    {
    public:
        FindPeakBody(const std::vector<cv::Mat>& heatmap, std::vector<vector<Peak>>& peaksFromHeatmap, float& minPeakDistance):
                    heatmap(heatmap), peaksFromHeatmap(peaksFromHeatmap), minPeakDistance(minPeakDistance){}

        virtual void operator()(const cv::Range& range) const{
            for(int i=range.start; i<range.end; ++i){
                findPeaks(heatmap, peaksFromHeatmap, minPeakDistance, i);
            }
        }
    private:
        const std::vector<cv::Mat>& heatmap;
        std::vector<vector<Peak>>& peaksFromHeatmap;
        float minPeakDistance;
    };

    /**
    * function: extract pose from heatmap image
    * heatmap: source input heatmap from python inference.
    * pafs:
    * upSample_ratio: extent ratio for find joint point accuracy.
    */
    std::vector<HumanPose> extract_poses(std::vector<cv::Mat>& heatmap,
                                         std::vector<cv::Mat>& pafs,
                                         int upSample_ratio)
    {
//        show heatmap and pafs image
//        for (int i=0; i<heatmap.size(); ++i){
//            cv::resize(heatmap[heatmap.size()-1], heatmap[heatmap.size()-1], cv::Size(851, 580));
//            cv::imshow("heatmap", heatmap[heatmap.size()-1]);
//            cv::waitKey(0);
//        }
//          showHeatMaps(heatmap, pafs);

//        for (int i=0; i<pafs.size(); ++i){
//            cv::resize(pafs[i], pafs[i], cv::Size(851, 580));
//            cv::imshow("pafs", pafs[i]);
//            cv::waitKey(0);
//        }

        resizeFeaturesMap(heatmap, upSample_ratio);
        resizeFeaturesMap(pafs, upSample_ratio);

        std::vector<vector<Peak>> peaksFromHeatmap(heatmap.size());
        float minPeakDistance = 3.0f;
        FindPeakBody findpeakbody(heatmap, peaksFromHeatmap, minPeakDistance);
        cv::parallel_for_(cv::Range(0, static_cast<int>(heatmap.size())), findpeakbody);

//        for(int i=0; i<peaksFromHeatmap.size(); ++i){
//            for(int j=0; j<peaksFromHeatmap[i].size(); ++j){
//                std::cout << peaksFromHeatmap[i][j] << std::endl;
//            }
//        }
        int peakBefore = 0;
        for(size_t heatmapId = 1; heatmapId < heatmap.size(); heatmapId++){
            peakBefore += static_cast<int>(peaksFromHeatmap[heatmapId - 1].size());
            for(auto &peak : peaksFromHeatmap[heatmapId]){
                peak.id += peakBefore;
            }
        }

        int keypointsNumber = 18;
        float midPointsScoreThreshold = 0.05f;
        float foundMidPointsRatioThreshold = 0.8f;
        int minJointsNumber = 3;
        float minSubsetScore = 0.2f;
        std::vector<HumanPose> poses = groupPeaksToPoses(
                    peaksFromHeatmap, pafs, keypointsNumber, midPointsScoreThreshold,
                    foundMidPointsRatioThreshold, minJointsNumber, minSubsetScore);
        return poses;
    }
}