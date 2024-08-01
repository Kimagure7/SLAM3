// 包含内容：实时获取轨迹， socket发送， 保存到文件（测试阶段）

#include "RealTimeTrajectory.h"
using nlohmann::json;
RealTimeTrajectory::RealTimeTrajectory(const float fps, const int targetPort, const string targetIP, const string fileSavePath)
    : mT(1e3 / fps), tPort(targetPort), mFileSavePath(fileSavePath), mbFinishRequested(false), tIP(targetIP) {
        CreateSocket(targetPort, targetIP);
}

void RealTimeTrajectory::Run() {
    cout << "RealTimeTrajectory::Run()" << endl;
    while(1) {
        if(!CheckTcw()) {
            usleep(mT / 2);
            continue;
        }
        auto result = GetTcw();
        SendTcw(result);
        // tobedone
        if(CheckFinish()) {
            break;
        }
    }
    cout << "RealTimeTrajectory::Run() finished" << endl;
    close(sock);
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
void RealTimeTrajectory::SendTcw(std::pair< Sophus::SE3f, bool > data) {
    json j;
    if(data.second) {
        Sophus::SE3f Tcw     = data.first;
        Sophus::SE3f Twc     = Tcw.inverse();
        Eigen::Vector3f twc  = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        j["is_lost"]         = 0;
        j["x"]               = twc(0);
        j["y"]               = twc(1);
        j["z"]               = twc(2);
        j["q_x"]             = q.x();
        j["q_y"]             = q.y();
        j["q_z"]             = q.z();
        j["q_w"]             = q.w();
    } else {
        j["is_lost"] = 1;
        j["x"]       = 0;
        j["y"]       = 0;
        j["z"]       = 0;
        j["q_x"]     = 0;
        j["q_y"]     = 0;
        j["q_z"]     = 0;
        j["q_w"]     = 1;
    }
    string s = j.dump();
    send(sock, s.c_str(), s.size(), 0);
}

bool RealTimeTrajectory::CreateSocket(const int targetPort, const string targetIP) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if(s == -1) {
        cout << "socket create failed" << endl;
        return false;
    }
    struct sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(targetPort);
    addr.sin_addr.s_addr = inet_addr(targetIP.c_str());
    if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        cout << "connect failed" << endl;
        return false;
    }
    sock = s;
    cout << "socket connect to" << targetIP << ":" << targetPort << endl;
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