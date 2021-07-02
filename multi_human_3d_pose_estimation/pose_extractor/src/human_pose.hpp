#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>

namespace human_pose_estimation{
   class HumanPose{
   public:
        HumanPose(const std::vector<cv::Point3f>& keypoints = std::vector<cv::Point3f>(),
                 const float& scores = 0):
                 keypoints(keypoints), scores(scores){}

        //friend ostream& operator<<(ostream& os, const HumanPose& human);

        std::vector<cv::Point3f> keypoints;
        float scores;
   private:

   };

//   ostream& operator<<(ostream& os, const HumanPose& human){
//       os << "keyPoint: " << human.keypoints << "  " << "score: " << human.scores;
//       return os;
//   }

}