// 包含内容：实时获取轨迹， socket发送， 保存到文件（测试阶段）

#include "RealTimeTrajectory.h"

namespace ORB_SLAM3 {
RealTimeTrajectory::RealTimeTrajectory(MapDrawer *pMapDrawer, const string &strSettingPath, Settings *settings) {
    mpMapDrawer = pMapDrawer;
    if(settings) {
        newParameterLoader(settings);
    }
}

void RealTimeTrajectory::Run() {
    Eigen::Matrix4f Twc;
    mbFinishRequested = false;
    cout << "RealTimeTrajectory::Run()" << endl;
    while(1) {
        usleep(mT * 1e3);
        mpMapDrawer->GetCurrentEigenCameraMatrix(Twc);
        historyTwc.push_back(Twc);
        // Eigen::Quaternionf q(Twc.block<3, 3>(0, 0));
        // Eigen::Vector3f t = Twc.block<3, 1>(0, 3);
        if(CheckFinish())
            break;
    }
    cout << "RealTimeTrajectory::Run() finished" << endl;
    SaveTrajectory();
}

bool RealTimeTrajectory::SaveTrajectory(const string &filename) {
    string saveFilePath;
    filename.empty() ? saveFilePath = "trajectory.csv" : saveFilePath = filename;
    ofstream f;
    f.open(saveFilePath.c_str());
    f << fixed;
    // add header
	f << "x,y,z,q_x,q_y,q_z,q_w" << endl;
    for(size_t i = 0; i < historyTwc.size(); i++) {
        Eigen::Quaternionf q(historyTwc[i].block<3, 3>(0, 0));
        Eigen::Vector3f t = historyTwc[i].block<3, 1>(0, 3);
        f << t(0) << "," << t(1) << "," << t(2) << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << endl;
    }
    f.close();
    cout << "Trajectory saved to " << saveFilePath << endl;
    return true;
}
void RealTimeTrajectory::newParameterLoader(Settings *settings) {
    float fps = settings->fps();
    if(fps < 1)
        fps = 30;
    mT = 1e3 / fps;
}

void RealTimeTrajectory::RequestFinish() {
    unique_lock< mutex > lock(mMutexFinish);
    mbFinishRequested = true;
}

bool RealTimeTrajectory::CheckFinish() {
    unique_lock< mutex > lock(mMutexFinish);
    return mbFinishRequested;
}
}    // namespace ORB_SLAM3