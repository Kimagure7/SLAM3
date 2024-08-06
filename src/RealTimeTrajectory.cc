// 包含内容：实时获取轨迹， socket发送， 保存到文件（测试阶段）

#include "RealTimeTrajectory.h"
using nlohmann::json;
RealTimeTrajectory::RealTimeTrajectory(const int targetPort, const string targetIP, const float fps, const string fileSavePath)
    : mT(1e3 / fps), tPort(targetPort), mFileSavePath(fileSavePath), mbFinishRequested(false), tIP(targetIP), mbAckReceived(false), frameCount(0), max_connect_time(3), cPort(0), cIP(""), mState(STATE::START) {
    // CreateSocket(targetPort, targetIP);
}
RealTimeTrajectory::RealTimeTrajectory(const int t_Port, const string t_IP, const int c_Port, const string c_IP, const float fps, const string fileSavePath)
    : mT(1e3 / fps), tPort(t_Port), mFileSavePath(fileSavePath), mbFinishRequested(false), tIP(t_IP), mbAckReceived(false), frameCount(0), max_connect_time(3), cPort(c_Port), cIP(c_IP), mState(STATE::START) {
    // CreateSocket(targetPort, targetIP);
}
void RealTimeTrajectory::Run() {
    cout << "RealTimeTrajectory::Run()" << endl;
    if(cPort) {
        WaitForStableTrack();
        RunCalibration();
    }
    while(!CreateSocket(tPort, tIP)) {
        usleep(1000);
    }
    // start tracking
    {
        unique_lock< mutex > lock(mMutexState);
        mState = STATE::TRACK;
    }

    while(1) {
        if(!CheckTcw()) {
            usleep(mT / 2);
            continue;
        }
        auto result = GetTcw();
        SendTcw(result);
        if(CheckFinish()) {
            break;
        }
    }
    cout << "RealTimeTrajectory::Run() finished" << endl;
    close(sock);
    SaveTrajectory();
}

void RealTimeTrajectory::WaitForStableTrack() {

    int stable_count = 0;
    while(1) {
        if(!CheckTcw()) {
            usleep(mT / 2);
            continue;
        }
        auto result = GetTcw();
        if(result.isOK) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        if(stable_count > 60) {
            break;
        }
    }
    cout << "RealTimeTrajectory::WaitForStableTrack() finished" << endl;
}

void RealTimeTrajectory::RunCalibration() {
    cout << "RealTimeTrajectory::RunCalibration()" << endl;

    bool initSocket = false;
    while(!CreateSocket(cPort, cIP)) {
        usleep(1000);
    }

    {
        unique_lock< mutex > lock(mMutexState);
        mState = STATE::CALIB;
    }

    int localFrameCount = 0;
    while(1) {
        if(!CheckTcw()) {
            usleep(mT / 2);
            continue;
        }
        auto result = GetTcw();
        if(CheckFinish()) {
            break;
        }
        if(result.isOK) {
            if(result.img.empty() || result.depth.empty()) {
                // still waiting for image data
                continue;
            }
            if(localFrameCount++ % 30 == 0) {
                // send every 30 frames
                SendTcw(result);
                if(mbCalibFinished) {
                    break;
                }
            }
        } else {
            cout << "RTT find a frame islost!!!!" << endl;
        }
    }
    cout << "RealTimeTrajectory::RunCalibration() finished" << endl;
    close(sock);
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

void RealTimeTrajectory::SendTcw(TcwData data) {
    // use base64 to encode image and depth
    json j;
    if(data.isOK) {
        Sophus::SE3f Tcw     = data.Tcw;
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
        j["frame_count"]     = frameCount;
        if(mState == STATE::CALIB) {
            j["img"]   = Mat2Base64(data.img, "jpg");
            j["depth"] = Mat2Base64(data.depth, "png");
        }
    } else {
        j["is_lost"]     = 1;
        j["x"]           = 0;
        j["y"]           = 0;
        j["z"]           = 0;
        j["q_x"]         = 0;
        j["q_y"]         = 0;
        j["q_z"]         = 0;
        j["q_w"]         = 1;
        j["frame_count"] = frameCount;
    }
    string s           = j.dump();
    ssize_t bytes_sent = send(sock, s.c_str(), s.size(), 0);
    if(bytes_sent == -1 || !RecvAck(frameCount)) {
        if(mState == STATE::TRACK) {
            ReconnectSocket(tPort, tIP);
        } else if(mState == STATE::CALIB) {
            ReconnectSocket(cPort, cIP);
        } else {
            cout << "Unknown state 166" << endl;
        }
    }
    frameCount++;
}

bool RealTimeTrajectory::RecvAck(int frameID) {
    // edit 8.6 : not use frameID, use normal ack

    char buffer[1024];
    fd_set readfds;
    struct timeval timeout;

    // Clear the file descriptor set
    FD_ZERO(&readfds);
    // Add socket to the file descriptor set
    FD_SET(sock, &readfds);

    // Set the timeout value (1 second)
    if(mState == STATE::TRACK) {
        timeout.tv_sec = 1;
    } else if(mState == STATE::CALIB) {
        timeout.tv_sec = 3;
    } else {
        cout << "Unknown state 186" << endl;
    }
    timeout.tv_usec = 0;

    int ret = select(sock + 1, &readfds, NULL, NULL, &timeout);

    if(ret == -1) {
        cout << "select failed" << endl;
        return false;
    } else if(ret == 0) {
        cout << "recv timed out" << endl;
        return false;
    }

    int recvSize = recv(sock, buffer, 1024, 0);
    if(recvSize == -1) {
        cout << "recv failed" << endl;
        return false;
    }
    string recvStr(buffer, recvSize);
    try {
        int ack = stoi(recvStr);
        if(ack == frameState::ACK || ack == frameState::THIS_FRAME_OK || ack == frameState::THIS_FRAME_FAIL) {
            return true;
        } else if(ack == frameState::CALIB_OK) {
            mbCalibFinished = true;
            return true;
        } else {
            cout << "recv ack failed" << endl;
            return false;
        }
    } catch(exception &e) {
        // print e
        cout << "recv ack failed,exeception, str=" << recvStr << endl;
        return false;
    }
}

bool RealTimeTrajectory::ReconnectSocket(const int targetPort, const string targetIP) {
    cout << "ReconnectSocket" << endl;
    close(sock);
    return CreateSocket(targetPort, targetIP);
}

bool RealTimeTrajectory::CreateSocket(const int targetPort, const string targetIP) {
    int s             = socket(AF_INET, SOCK_STREAM, 0);
    int retryingTimes = 0;
    if(s == -1) {
        cout << "socket create failed" << endl;
        return false;
    }
    struct sockaddr_in addr;
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(targetPort);
    addr.sin_addr.s_addr = inet_addr(targetIP.c_str());

    while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        cout << "connect failed, retrying times:" << ++retryingTimes << endl;
        usleep(200);
        if(retryingTimes > max_connect_time) {
            return false;
        }
    }
    sock = s;
    cout << "socket connect to" << targetIP << ":" << targetPort << endl;
    return true;
}

void RealTimeTrajectory::AddTcw(TcwData result) {
    unique_lock< mutex > lock(mMutexQueue);
    mQueueTcw.push(result);
    mHistoryTcw.push_back(
        std::make_pair(result.Tcw, result.isOK));
}

bool RealTimeTrajectory::CheckTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    return !mQueueTcw.empty();
}

void RealTimeTrajectory::CleanTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    while(!mQueueTcw.empty()) {
        mQueueTcw.pop();
    }
}
int RealTimeTrajectory::QueryTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    return mQueueTcw.size();
}

RealTimeTrajectory::TcwData RealTimeTrajectory::GetTcw() {
    unique_lock< mutex > lock(mMutexQueue);
    TcwData result = mQueueTcw.front();
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

bool RealTimeTrajectory::CheckState() {
    unique_lock< mutex > lock(mMutexState);
    return mState == STATE::CALIB;
}

std::string RealTimeTrajectory::Mat2Base64(const cv::Mat &image, std::string imgType) {
    // Mat转base64
    std::vector< uchar > buf;
    cv::imencode(imgType, image, buf);
    std::string img_data = base64_encode(buf.data(), buf.size(), false);
    return img_data;
}

cv::Mat RealTimeTrajectory::Base642Mat(const std::string &base64_data) {
    cv::Mat img;
    std::string s_mat;
    s_mat = base64_decode(base64_data, false);
    std::vector< char > base64_img(s_mat.begin(), s_mat.end());
    img = cv::imdecode(base64_img, cv::IMREAD_COLOR);    // CV::IMREAD_UNCHANGED
    return img;
}