#ifndef REALTIMETRAJECTORY_H
#define REALTIMETRAJECTORY_H

#include "MapDrawer.h"
#include "System.h"
#include <mutex>
namespace ORB_SLAM3 {
class MapDrawer;
class RealTimeTrajectory {
public:
    RealTimeTrajectory(MapDrawer *pMapDrawer, const string &strSettingPath, Settings *settings);
    void Run();
    void RequestFinish();

private:
    // bool ParseRealTimeTrajcetoryParamFile(cv::FileStorage &fSettings);
    MapDrawer *mpMapDrawer;
    float mT;    // 1/fps in ms
    bool mbFinishRequested = false;
    std::mutex mMutexFinish;
    std::vector< Eigen::Matrix4f > historyTwc;

    // bool ParseRealTimeTrajectoryParamFile(cv::FileStorage &fSettings);
    void newParameterLoader(Settings *settings);
    bool CheckFinish();
    bool SaveTrajectory(const string &filename=string()); // debug use
};
}    // namespace ORB_SLAM3
#endif    // REALTIMETRAJECTORY_H