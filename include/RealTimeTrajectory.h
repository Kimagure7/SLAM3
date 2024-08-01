#ifndef REALTIMETRAJECTORY_H
#define REALTIMETRAJECTORY_H
#include <System.h>
#include <mutex>
using namespace std;
class RealTimeTrajectory {
public:
    RealTimeTrajectory(const float fps = 30, const string targetPort = "", const string fileSavePath = "");
    ~RealTimeTrajectory() = default;
    void Run();
    void RequestFinish();
    void AddTcw(std::pair< Sophus::SE3f, bool > result);

private:
    // bool ParseRealTimeTrajcetoryParamFile(cv::FileStorage &fSettings);
    // MapDrawer *mpMapDrawer;
    float mT;    // 1/fps in ms
    bool mbFinishRequested = false;
    string tPort;    // socket target port
    const string mFileSavePath;
    // Tracking *mpTracker;
    std::mutex mMutexFinish, mMutexQueue;
    std::vector< std::pair< Sophus::SE3f, bool > > mHistoryTcw;
    std::queue< std::pair< Sophus::SE3f, bool > > mQueueTcw;
    // bool ParseRealTimeTrajectoryParamFile(cv::FileStorage &fSettings);
    // void newParameterLoader(Settings *settings);
    bool CheckFinish();
    bool SaveTrajectory();    // debug use
    std::pair< Sophus::SE3f, bool > GetTcw();
    bool CheckTcw();
};
#endif    // REALTIMETRAJECTORY_H