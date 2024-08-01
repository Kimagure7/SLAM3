// 包含内容：实时获取轨迹， socket发送， 保存到文件（测试阶段）

#include "RealTimeTrajectory.h"

RealTimeTrajectory::RealTimeTrajectory(const float fps, const string targetPort, const string fileSavePath)
    : mT(1e3 / fps), tPort(targetPort), mFileSavePath(fileSavePath), mbFinishRequested(false) {
}

void RealTimeTrajectory::Run() {
    cout << "RealTimeTrajectory::Run()" << endl;
    while(1) {
        if(!CheckTcw()) {
            usleep(mT / 2);
            continue;
        }
        auto result = GetTcw();
        // send to socket
        // tobedone
        if(CheckFinish())
            break;
    }
    cout << "RealTimeTrajectory::Run() finished" << endl;
    SaveTrajectory();
}

bool RealTimeTrajectory::SaveTrajectory() {
    string saveFilePath;
    mFileSavePath.empty() ? saveFilePath = "trajectory.csv" : saveFilePath = mFileSavePath;
    ofstream f;
    f.open(saveFilePath.c_str());
    f << fixed;
    // add header
    f << "is_lost,x,y,z,q_x,q_y,q_z,q_w" << endl;
    for(size_t i = 0; i < mHistoryTcw.size(); i++) {
        auto result = mHistoryTcw[i];
        if(result.second) {
            Sophus::SE3f Tcw     = result.first;
            Sophus::SE3f Twc     = Tcw.inverse();
            Eigen::Vector3f twc  = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << "0,";
            f << setprecision(9) << twc(0) << ',' << twc(1) << ',' << twc(2) << ',';
            f << setprecision(9) << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << endl;
        } else {
            f << "1,0,0,0,0,0,0,1" << endl;
        }
    }
    f.close();
    cout << "Trajectory saved to " << saveFilePath << endl;
    return true;
}


void RealTimeTrajectory::AddTcw(std::pair< Sophus::SE3f, bool > result) {
    unique_lock< mutex > lock(mMutexQueue);
    mQueueTcw.push(result);
    mHistoryTcw.push_back(result);
}

bool RealTimeTrajectory::CheckTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    return !mQueueTcw.empty();
}

std::pair< Sophus::SE3f, bool > RealTimeTrajectory::GetTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    std::pair< Sophus::SE3f, bool > result = mQueueTcw.front();
    mQueueTcw.pop();
    return result;
}

void RealTimeTrajectory::RequestFinish() {
    unique_lock< mutex > lock(mMutexFinish);
    mbFinishRequested = true;
}

bool RealTimeTrajectory::CheckFinish() {
    unique_lock< mutex > lock(mMutexFinish);
    return mbFinishRequested;
}