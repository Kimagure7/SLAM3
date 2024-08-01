#ifndef REALTIMETRAJECTORY_H
#define REALTIMETRAJECTORY_H
#include <System.h>
#include <arpa/inet.h>
#include <json.h>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <sys/socket.h>
#include <vector>

using namespace std;
class RealTimeTrajectory {
public:
    RealTimeTrajectory(const float fps = 30, const int targetPort = 0, const string targetIP = "", const string fileSavePath = "");
    void Run();
    void RequestFinish();
    void AddTcw(std::pair< Sophus::SE3f, bool > result);

private:
    // bool ParseRealTimeTrajcetoryParamFile(cv::FileStorage &fSettings);
    // MapDrawer *mpMapDrawer;
    float mT;    // 1/fps in ms
    bool mbFinishRequested = false;
    const int tPort;    // socket target port
    const string tIP;
    const string mFileSavePath;
    int sock;
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
    void SendTcw(std::pair< Sophus::SE3f, bool > data);
    bool CreateSocket(const int targetPort, const string targetIP);
};
#endif    // REALTIMETRAJECTORY_H