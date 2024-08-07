#ifndef REALTIMETRAJECTORY_H
#define REALTIMETRAJECTORY_H

#include "base64.h"
#include <System.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <json.h>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
using namespace std;
class RealTimeTrajectory {
public:
    struct TcwData {
        bool isOK;
        Sophus::SE3f Tcw;
        cv::Mat img, depth;

        TcwData(bool ok, const Sophus::SE3f &t)
            : isOK(ok), Tcw(t), img(cv::Size(0, 0), CV_8UC3), depth(cv::Size(0, 0), CV_16U) {}
        TcwData(bool ok, const Sophus::SE3f &t, const cv::Mat &i, const cv::Mat &d)
            : isOK(ok), Tcw(t), img(i), depth(d) {}
    };
    enum STATE {    // indicate the working state of the system
        START,
        CALIB,
        TRACK,
    };
    enum frameState {
        ACK,    // for track
        THIS_FRAME_OK,
        THIS_FRAME_FAIL,
        CALIB_OK,
    };

    RealTimeTrajectory(const int targetPort, const string targetIP, const float fps = 30, const string fileSavePath = "");
    RealTimeTrajectory(const int t_Port, const string t_IP, const int c_Port, const string c_IP, const float fps = 30, const string fileSavePath = "");

    void Run();
    void RequestFinish();
    void AddTcw(TcwData result);
    bool CheckState();

    const float *intrinsicsMatrix;

private:
    // bool ParseRealTimeTrajcetoryParamFile(cv::FileStorage &fSettings);

    float mT;    // 1/fps in ms
    bool mbFinishRequested = false;
    bool mbCalibFinished   = false;
    const string mFileSavePath;
    STATE mState = START;

    const int tPort, cPort;    // socket target port, track and calib port
    const string tIP, cIP;
    bool mbAckReceived;
    int frameCount;
    int sock;
    int max_connect_time = 3;


    // Tracking *mpTracker;
    std::mutex mMutexFinish, mMutexQueue, mMutexState;
    std::vector< std::pair< Sophus::SE3f, bool > > mHistoryTcw;
    std::queue< TcwData > mQueueTcw;
    // std::queue< std::pair< Sophus::SE3f, bool > > mQueueTcw;

    // bool ParseRealTimeTrajectoryParamFile(cv::FileStorage &fSettings);
    // void newParameterLoader(Settings *settings);
    bool CheckFinish();
    bool SaveTrajectory();    // debug use
    string Mat2Base64(const cv::Mat &img, string imgType);
    cv::Mat Base642Mat(const string &base64_data);

    // Wait for SLAM track to be stable. Set to true if isOK is true for the last 60 frames
    void WaitForStableTrack();
    void RunCalibration();
    void SendIntrinsic();

    TcwData GetTcw();
    bool CheckTcw();
    void CleanTcw();
    int QueryTcw();
    bool SendTcw(TcwData data);

    bool CreateSocket(const int targetPort, const string targetIP);
    bool ReconnectSocket(const int targetPort, const string targetIP);
    bool RecvAck(int frameID);
};
#endif    // REALTIMETRAJECTORY_H