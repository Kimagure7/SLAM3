/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Frame.h"

#include "Converter.h"
#include "G2oTypes.h"
#include "GeometricCamera.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"

#include <include/CameraModels/KannalaBrandt8.h>
#include <include/CameraModels/Pinhole.h>
#include <thread>

namespace ORB_SLAM3 {

long unsigned int Frame::nNextId  = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

// For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame()
    : mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast< KeyFrame * >(NULL)), mbIsSet(false), mbImuPreintegrated(false), mbHasPose(false), mbHasVelocity(false) {}


// Copy Constructor
Frame::Frame(const Frame &frame)
    : mpcpi(frame.mpcpi), mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
      mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)), mDistCoef(frame.mDistCoef.clone()),
      mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
      mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
      markerIds(frame.markerIds), markerCorners(frame.markerCorners), rejectedCandidates(frame.rejectedCandidates),
      mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
      mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
      mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs),
      mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
      mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
      mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
      mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
      mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
      monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
      mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
      mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl),
      mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false) {
    for(int i = 0; i < FRAME_GRID_COLS; i++)
        for(int j = 0; j < FRAME_GRID_ROWS; j++) {
            mGrid[i][j] = frame.mGrid[i][j];
            if(frame.Nleft > 0) {
                mGridRight[i][j] = frame.mGridRight[i][j];
            }
        }

    if(frame.mbHasPose)
        SetPose(frame.GetPose());

    if(frame.HasVelocity()) {
        SetVelocity(frame.GetVelocity());
    }

    mmProjectPoints  = frame.mmProjectPoints;
    mmMatchedInImage = frame.mmMatchedInImage;
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast< KeyFrame * >(NULL)), mbIsSet(false), mbImuPreintegrated(false),
      mpCamera(pCamera), mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false) {
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels     = mpORBextractorLeft->GetLevels();
    mfScaleFactor     = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor  = log(mfScaleFactor);
    mvScaleFactors    = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2     = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2  = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
    thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();
    ComputeStereoMatches();

    mvpMapPoints = vector< MapPoint * >(N, static_cast< MapPoint * >(NULL));
    mvbOutlier   = vector< bool >(N, false);
    mmProjectPoints.clear();
    mmMatchedInImage.clear();


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations) {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv  = static_cast< float >(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast< float >(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);



        fx    = K.at< float >(0, 0);
        fy    = K.at< float >(1, 1);
        cx    = K.at< float >(0, 2);
        cy    = K.at< float >(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    if(pPrevF) {
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    } else {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    // Set no stereo fisheye information
    Nleft              = -1;
    Nright             = -1;
    mvLeftToRightMatch = vector< int >(0);
    mvRightToLeftMatch = vector< int >(0);
    mvStereo3Dpoints   = vector< Eigen::Vector3f >(0);
    monoLeft           = -1;
    monoRight          = -1;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast< ORBextractor * >(NULL)),
      mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast< KeyFrame * >(NULL)), mbIsSet(false), mbImuPreintegrated(false),
      mpCamera(pCamera), mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false) {
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels     = mpORBextractorLeft->GetLevels();
    mfScaleFactor     = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor  = log(mfScaleFactor);
    mvScaleFactors    = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2     = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2  = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0, imGray, 0, 0);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector< MapPoint * >(N, static_cast< MapPoint * >(NULL));

    mmProjectPoints.clear();
    mmMatchedInImage.clear();

    mvbOutlier = vector< bool >(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations) {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv  = static_cast< float >(FRAME_GRID_COLS) / static_cast< float >(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast< float >(FRAME_GRID_ROWS) / static_cast< float >(mnMaxY - mnMinY);

        fx    = K.at< float >(0, 0);
        fy    = K.at< float >(1, 1);
        cx    = K.at< float >(0, 2);
        cy    = K.at< float >(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    if(pPrevF) {
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    } else {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    // Set no stereo fisheye information
    Nleft              = -1;
    Nright             = -1;
    mvLeftToRightMatch = vector< int >(0);
    mvRightToLeftMatch = vector< int >(0);
    mvStereo3Dpoints   = vector< Eigen::Vector3f >(0);
    monoLeft           = -1;
    monoRight          = -1;

    AssignFeaturesToGrid();
}

/**
 * @brief 构造函数：初始化帧对象
 * 
 * 该构造函数用于初始化一个帧（Frame）对象。它接受多个参数，包括灰度图像、时间戳、ORB提取器、ORB词汇表、几何相机模型、畸变系数等。
 * 它还负责执行以下操作：
 * - 设置帧的唯一ID。
 * - 初始化尺度层级信息，如尺度因子和逆尺度因子等。
 * - 检测图像中的ArUcO标记。
 * - 提取图像的ORB特征点。
 * - 对特征点进行畸变校正。
 * - 分配特征点到网格中以加速后续处理。
 *
 * @param imGray 灰度图像
 * @param timeStamp 时间戳
 * @param extractor ORB特征提取器
 * @param voc ORB词汇表
 * @param pCamera 相机模型指针
 * @param distCoef 相机畸变系数矩阵
 * @param bf 基线焦距比率（用于计算深度）
 * @param thDepth 深度阈值（用于过滤深度信息）
 * @param pPrevF 上一帧指针（如果存在）
 */
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame *pPrevF, const IMU::Calib &ImuCalib, cv::Ptr< cv::aruco::Dictionary > aruco_dict)
    : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast< ORBextractor * >(NULL)),
      mTimeStamp(timeStamp), mK(static_cast< Pinhole * >(pCamera)->toK()), mK_(static_cast< Pinhole * >(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast< KeyFrame * >(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera),
      mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false) {
    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels     = mpORBextractorLeft->GetLevels();
    mfScaleFactor     = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor  = log(mfScaleFactor);
    mvScaleFactors    = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2     = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2  = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ArUcO detection
    cv::Ptr< cv::aruco::DetectorParameters > parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(imGray, aruco_dict, markerCorners, markerIds, parameters, rejectedCandidates);

    // ORB extraction
    ExtractORB(0, imGray, 0, 1000);

    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight   = vector< float >(N, -1);
    mvDepth    = vector< float >(N, -1);
    mnCloseMPs = 0;

    mvpMapPoints = vector< MapPoint * >(N, static_cast< MapPoint * >(NULL));

    mmProjectPoints.clear();    // = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector< bool >(N, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations) {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv  = static_cast< float >(FRAME_GRID_COLS) / static_cast< float >(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast< float >(FRAME_GRID_ROWS) / static_cast< float >(mnMaxY - mnMinY);

        fx    = static_cast< Pinhole    *>(mpCamera)->toK().at< float >(0, 0);
        fy    = static_cast< Pinhole    *>(mpCamera)->toK().at< float >(1, 1);
        cx    = static_cast< Pinhole    *>(mpCamera)->toK().at< float >(0, 2);
        cy    = static_cast< Pinhole    *>(mpCamera)->toK().at< float >(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }


    mb = mbf / fx;

    // Set no stereo fisheye information
    Nleft              = -1;
    Nright             = -1;
    mvLeftToRightMatch = vector< int >(0);
    mvRightToLeftMatch = vector< int >(0);
    mvStereo3Dpoints   = vector< Eigen::Vector3f >(0);
    monoLeft           = -1;
    monoRight          = -1;

    AssignFeaturesToGrid();

    if(pPrevF) {
        if(pPrevF->HasVelocity()) {
            SetVelocity(pPrevF->GetVelocity());
        }
    } else {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();
}

/**
 * @brief 将特征点分配到网格中
 * 
 * 该函数的主要目的是将当前帧的所有特征点根据其在图像中的位置，分配到一个预定义的网格系统中。
 * 这样做可以加速后续的特征匹配过程，因为只需要在局部区域内搜索匹配项，而不是在整个图像上。
 * 
 * 如果存在右图（即Nleft != -1），则也会为右图创建一个类似的网格，并将右图的特征点分配到相应的网格中。
 */
void Frame::AssignFeaturesToGrid() {
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

    int nReserve = 0.5f * N / (nCells);

    for(unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for(unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1) {
                mGridRight[i][j].reserve(nReserve);
            }
        }



    for(int i = 0; i < N; i++) {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                 : (i < Nleft) ? mvKeys[i]
                                               : mvKeysRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp, nGridPosX, nGridPosY)) {
            if(Nleft == -1 || i < Nleft)
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

/**
 * @brief 从输入图像中提取ORB特征点和描述符的函数。
 *
 * @param flag 用于选择左或右ORB特征提取器的标志，0表示左，非0表示右。
 * @param im 输入图像，应为8位灰度图像，用于从中提取ORB特征。
 * @param x0 指定区域的最小x坐标，用于限制关键点检测范围。
 * @param x1 指定区域的最大x坐标，用于限制关键点检测范围。
 *
 * 此函数根据flag选择使用左或右ORB特征提取器来处理输入图像im，
 * 并在指定的x坐标范围内（x0至x1）进行关键点和描述符的提取。如果flag为0，
 * 将使用mpORBextractorLeft进行处理并将结果存储在monoLeft中；否则将使用
 * mpORBextractorRight进行处理并将结果存储在monoRight中。该函数不返回任何值，
 * 但会更新mvKeys、mDescriptors（对于左相机）或mvKeysRight、mDescriptorsRight（对于右相机）
 */
void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1) {
    vector< int > vLapping = { x0, x1 };
    if(flag == 0)
        monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
    else
        monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, vLapping);
}

bool Frame::isSet() const {
    return mbIsSet;
}

void Frame::SetPose(const Sophus::SE3< float > &Tcw) {
    mTcw = Tcw;

    UpdatePoseMatrices();
    mbIsSet   = true;
    mbHasPose = true;
}

void Frame::SetNewBias(const IMU::Bias &b) {
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(Eigen::Vector3f Vwb) {
    mVw           = Vwb;
    mbHasVelocity = true;
}

Eigen::Vector3f Frame::GetVelocity() const {
    return mVw;
}

/**
 * @brief 设置帧的IMU位置、旋转和速度
 *
 * 该函数接收IMU测量得到的位置、旋转和速度信息，更新帧的内部表示以反映这些新的状态。
 * 它首先将接收到的旋转和平移信息封装到一个Sophus::SE3f对象中，然后计算相机在世界坐标系下的变换，
 * 并通过调用UpdatePoseMatrices()来更新所有相关的姿态矩阵。同时，它标记帧为已设置其IMU状态和姿势。
 *
 * @param Rwb 世界坐标系下的IMU旋转（Eigen::Matrix3f）
 * @param twb 世界坐标系下的IMU平移（Eigen::Vector3f）
 * @param Vwb IMU的速度（Eigen::Vector3f）
 */
void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb) {
    mVw           = Vwb;
    mbHasVelocity = true;

    Sophus::SE3f Twb(Rwb, twb);
    Sophus::SE3f Tbw = Twb.inverse();

    mTcw = mImuCalib.mTcb * Tbw;

    UpdatePoseMatrices();
    mbIsSet   = true;
    mbHasPose = true;
}

/**
 * @brief 更新帧的姿态矩阵
 *
 * 此函数负责根据当前帧在世界坐标系中的变换来更新所有相关的位置和方向矩阵。
 * 这些矩阵包括从相机到世界的方向矩阵mRwc，从世界到相机的平移向量mOw，
 * 以及从相机到世界的平移向量mtcw和方向矩阵mRcw。这些更新后的矩阵对于后续处理如特征匹配或地图点投影至关重要。
 */
void Frame::UpdatePoseMatrices() {
    Sophus::SE3< float > Twc = mTcw.inverse();
    mRwc                     = Twc.rotationMatrix();
    mOw                      = Twc.translation();
    mRcw                     = mTcw.rotationMatrix();
    mtcw                     = mTcw.translation();
}

Eigen::Matrix< float, 3, 1 > Frame::GetImuPosition() const {
    return mRwc * mImuCalib.mTcb.translation() + mOw;
}

Eigen::Matrix< float, 3, 3 > Frame::GetImuRotation() {
    return mRwc * mImuCalib.mTcb.rotationMatrix();
}

Sophus::SE3< float > Frame::GetImuPose() {
    return mTcw.inverse() * mImuCalib.mTcb;
}

Sophus::SE3f Frame::GetRelativePoseTrl() {
    return mTrl;
}

Sophus::SE3f Frame::GetRelativePoseTlr() {
    return mTlr;
}

Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation() {
    return mTlr.rotationMatrix();
}

Eigen::Vector3f Frame::GetRelativePoseTlr_translation() {
    return mTlr.translation();
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
    if(Nleft == -1) {
        pMP->mbTrackInView = false;
        pMP->mTrackProjX   = -1;
        pMP->mTrackProjY   = -1;

        // 3D in absolute coordinates
        Eigen::Matrix< float, 3, 1 > P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix< float, 3, 1 > Pc = mRcw * P + mtcw;
        const float Pc_dist                   = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f / PcZ;
        if(PcZ < 0.0f)
            return false;

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        if(uv(0) < mnMinX || uv(0) > mnMaxX)
            return false;
        if(uv(1) < mnMinY || uv(1) > mnMaxY)
            return false;

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance  = pMP->GetMaxDistanceInvariance();
        const float minDistance  = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - mOw;
        const float dist         = PO.norm();

        if(dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if(viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX   = uv(0);
        pMP->mTrackProjXR  = uv(0) - mbf * invz;

        pMP->mTrackDepth = Pc_dist;

        pMP->mTrackProjY       = uv(1);
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos     = viewCos;

        return true;
    } else {
        pMP->mbTrackInView      = false;
        pMP->mbTrackInViewR     = false;
        pMP->mnTrackScaleLevel  = -1;
        pMP->mnTrackScaleLevelR = -1;

        pMP->mbTrackInView  = isInFrustumChecks(pMP, viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}

bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v) {

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3f Pc = mRcw * P + mtcw;
    const float &PcX         = Pc(0);
    const float &PcY         = Pc(1);
    const float &PcZ         = Pc(2);

    // Check positive depth
    if(PcZ < 0.0f) {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    u                = fx * PcX * invz + cx;
    v                = fy * PcY * invz + cy;

    if(u < mnMinX || u > mnMaxX)
        return false;
    if(v < mnMinY || v > mnMaxY)
        return false;

    float u_distort, v_distort;

    float x  = (u - cx) * invfx;
    float y  = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at< float >(0);
    float k2 = mDistCoef.at< float >(1);
    float p1 = mDistCoef.at< float >(2);
    float p2 = mDistCoef.at< float >(3);
    float k3 = 0;
    if(mDistCoef.total() == 5) {
        k3 = mDistCoef.at< float >(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;


    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw) {
    return mRcw * pCw + mtcw;
}

vector< size_t > Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel, const bool bRight) const {
    vector< size_t > vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
    if(nMinCellX >= FRAME_GRID_COLS) {
        return vIndices;
    }

    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
    if(nMaxCellX < 0) {
        return vIndices;
    }

    const int nMinCellY = max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
    if(nMinCellY >= FRAME_GRID_ROWS) {
        return vIndices;
    }

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
    if(nMaxCellY < 0) {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for(int ix = nMinCellX; ix <= nMaxCellX; ix++) {
        for(int iy = nMinCellY; iy <= nMaxCellY; iy++) {
            const vector< size_t > vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j = 0, jend = vCell.size(); j < jend; j++) {
                const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                           : (!bRight)   ? mvKeys[vCell[j]]
                                                         : mvKeysRight[vCell[j]];
                if(bCheckLevels) {
                    if(kpUn.octave < minLevel)
                        continue;
                    if(maxLevel >= 0)
                        if(kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if(fabs(distx) < factorX && fabs(disty) < factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
    posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
    posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

    // Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
        return false;

    return true;
}

/**
 * @brief 计算当前帧的Bag of Words (BoW)表示。
 *
 * 此函数用于将帧中的ORB描述符转换为其在视觉词典中的BoW表示形式。如果尚未计算过BoW向量，
 * 则使用提供的ORB词汇树进行转换，并同时计算特征向量，这有助于后续的匹配过程。
 *
 * @note 此函数会检查mBowVec是否为空，以避免重复计算。
 */
void Frame::ComputeBoW() {
    if(mBowVec.empty()) {
        vector< cv::Mat > vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
}

/**
 * @brief UndistortKeyPoints 函数用于对当前帧中的特征点进行去畸变处理。
 * 
 * 如果相机的畸变系数为0，则直接将原始特征点赋值给去畸变后的特征点向量。
 * 否则，函数会先将特征点坐标填充到一个矩阵中，然后调用 OpenCV 的 undistortPoints 函数，
 * 使用相机内参和畸变系数对这些坐标进行去畸变。最后，将去畸变后的坐标重新填充到新的特征点向量中。
 */
void Frame::UndistortKeyPoints() {
    if(mDistCoef.at< float >(0) == 0.0) {
        mvKeysUn = mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N, 2, CV_32F);

    for(int i = 0; i < N; i++) {
        mat.at< float >(i, 0) = mvKeys[i].pt.x;
        mat.at< float >(i, 1) = mvKeys[i].pt.y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, static_cast< Pinhole * >(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);


    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i = 0; i < N; i++) {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x         = mat.at< float >(i, 0);
        kp.pt.y         = mat.at< float >(i, 1);
        mvKeysUn[i]     = kp;
    }
}

/**
 * @brief ComputeImageBounds 函数用于计算在去畸变后图像的边界。
 * 
 * 如果相机的畸变系数不为0，则首先创建一个4x2矩阵，表示图像四个角点的位置。
 * 然后调用 OpenCV 的 undistortPoints 函数对这些角点进行去畸变处理，使用相机内参和畸变系数。
 * 最后，计算去畸变后的最小和最大横纵坐标值来确定新的图像边界。
 * 如果相机没有畸变（即畸变系数为0），则直接将原始图像的边界作为结果返回。
 */
void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
    if(mDistCoef.at< float >(0) != 0.0) {
        cv::Mat mat(4, 2, CV_32F);
        mat.at< float >(0, 0) = 0.0;
        mat.at< float >(0, 1) = 0.0;
        mat.at< float >(1, 0) = imLeft.cols;
        mat.at< float >(1, 1) = 0.0;
        mat.at< float >(2, 0) = 0.0;
        mat.at< float >(2, 1) = imLeft.rows;
        mat.at< float >(3, 0) = imLeft.cols;
        mat.at< float >(3, 1) = imLeft.rows;

        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, static_cast< Pinhole * >(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at< float >(0, 0), mat.at< float >(2, 0));
        mnMaxX = max(mat.at< float >(1, 0), mat.at< float >(3, 0));
        mnMinY = min(mat.at< float >(0, 1), mat.at< float >(1, 1));
        mnMaxY = max(mat.at< float >(2, 1), mat.at< float >(3, 1));
    } else {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches() {
    mvuRight = vector< float >(N, -1.0f);
    mvDepth  = vector< float >(N, -1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    // Assign keypoints to row table
    vector< vector< size_t > > vRowIndices(nRows, vector< size_t >());

    for(int i = 0; i < nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR = 0; iR < Nr; iR++) {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY       = kp.pt.y;
        const float r          = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr         = ceil(kpY + r);
        const int minr         = floor(kpY - r);

        for(int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf / minZ;

    // For each left keypoint search a match in the right image
    vector< pair< int, int > > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL = 0; iL < N; iL++) {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL       = kpL.octave;
        const float &vL         = kpL.pt.y;
        const float &uL         = kpL.pt.x;

        const vector< size_t > &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL - maxD;
        const float maxU = uL - minD;

        if(maxU < 0)
            continue;

        int bestDist    = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC = 0; iC < vCandidates.size(); iC++) {
            const size_t iR         = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR >= minU && uR <= maxU) {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist    = ORBmatcher::DescriptorDistance(dL, dR);

                if(dist < bestDist) {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist < thOrbDist) {
            // coordinates in image pyramid at keypoint scale
            const float uR0         = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL    = round(kpL.pt.x * scaleFactor);
            const float scaledvL    = round(kpL.pt.y * scaleFactor);
            const float scaleduR0   = round(uR0 * scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL  = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L  = 5;
            vector< float > vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if(iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR = -L; incR <= +L; incR++) {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if(dist < bestDist) {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L + incR] = dist;
            }

            if(bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

            if(deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if(disparity >= minD && disparity < maxD) {
                if(disparity <= 0) {
                    disparity = 0.01;
                    bestuR    = uL - 0.01;
                }
                mvDepth[iL]  = mbf / disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair< int, int >(bestDist, iL));
            }
        }
    }

    sort(vDistIdx.begin(), vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size() / 2].first;
    const float thDist = 1.5f * 1.4f * median;

    for(int i = vDistIdx.size() - 1; i >= 0; i--) {
        if(vDistIdx[i].first < thDist)
            break;
        else {
            mvuRight[vDistIdx[i].second] = -1;
            mvDepth[vDistIdx[i].second]  = -1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
    mvuRight = vector< float >(N, -1);
    mvDepth  = vector< float >(N, -1);

    for(int i = 0; i < N; i++) {
        const cv::KeyPoint &kp  = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at< float >(v, u);

        if(d > 0) {
            mvDepth[i]  = d;
            mvuRight[i] = kpU.pt.x - mbf / d;
        }
    }
}

bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D) {
    const float z = mvDepth[i];
    if(z > 0) {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        Eigen::Vector3f x3Dc(x, y, z);
        x3D = mRwc * x3Dc + mOw;
        return true;
    } else
        return false;
}

bool Frame::imuIsPreintegrated() {
    unique_lock< std::mutex > lock(*mpMutexImu);
    return mbImuPreintegrated;
}

void Frame::setIntegrated() {
    unique_lock< std::mutex > lock(*mpMutexImu);
    mbImuPreintegrated = true;
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, GeometricCamera *pCamera2, Sophus::SE3f &Tlr, Frame *pPrevF, const IMU::Calib &ImuCalib)
    : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
      mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast< KeyFrame * >(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2),
      mbHasPose(false), mbHasVelocity(false)

{
    imgLeft  = imLeft.clone();
    imgRight = imRight.clone();

    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
    mnScaleLevels     = mpORBextractorLeft->GetLevels();
    mfScaleFactor     = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor  = log(mfScaleFactor);
    mvScaleFactors    = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2     = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2  = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, static_cast< KannalaBrandt8 * >(mpCamera)->mvLappingArea[0], static_cast< KannalaBrandt8 * >(mpCamera)->mvLappingArea[1]);
    thread threadRight(&Frame::ExtractORB, this, 1, imRight, static_cast< KannalaBrandt8 * >(mpCamera2)->mvLappingArea[0], static_cast< KannalaBrandt8 * >(mpCamera2)->mvLappingArea[1]);
    threadLeft.join();
    threadRight.join();
    Nleft  = mvKeys.size();
    Nright = mvKeysRight.size();
    N      = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations) {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv  = static_cast< float >(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast< float >(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

        fx    = K.at< float >(0, 0);
        fy    = K.at< float >(1, 1);
        cx    = K.at< float >(0, 2);
        cy    = K.at< float >(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    mTlr = Tlr;
    mTrl = mTlr.inverse();
    mRlr = mTlr.rotationMatrix();
    mtlr = mTlr.translation();

    ComputeStereoFishEyeMatches();

    // Put all descriptors in the same matrix
    cv::vconcat(mDescriptors, mDescriptorsRight, mDescriptors);

    mvpMapPoints = vector< MapPoint * >(N, static_cast< MapPoint * >(nullptr));
    mvbOutlier   = vector< bool >(N, false);

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();
}

void Frame::ComputeStereoFishEyeMatches() {
    // Speed it up by matching keypoints in the lapping area
    vector< cv::KeyPoint > stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
    vector< cv::KeyPoint > stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

    cv::Mat stereoDescLeft  = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
    cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

    mvLeftToRightMatch = vector< int >(Nleft, -1);
    mvRightToLeftMatch = vector< int >(Nright, -1);
    mvDepth            = vector< float >(Nleft, -1.0f);
    mvuRight           = vector< float >(Nleft, -1);
    mvStereo3Dpoints   = vector< Eigen::Vector3f >(Nleft);
    mnCloseMPs         = 0;

    // Perform a brute force between Keypoint in the left and right image
    vector< vector< cv::DMatch > > matches;

    BFmatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

    int nMatches    = 0;
    int descMatches = 0;

    // Check matches using Lowe's ratio
    for(vector< vector< cv::DMatch > >::iterator it = matches.begin(); it != matches.end(); ++it) {
        if((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7) {
            // For every good match, check parallax and reprojection error to discard spurious matches
            Eigen::Vector3f p3D;
            descMatches++;
            float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
            float depth = static_cast< KannalaBrandt8 * >(mpCamera)->TriangulateMatches(mpCamera2, mvKeys[(*it)[0].queryIdx + monoLeft], mvKeysRight[(*it)[0].trainIdx + monoRight], mRlr, mtlr, sigma1, sigma2, p3D);
            if(depth > 0.0001f) {
                mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft]  = (*it)[0].trainIdx + monoRight;
                mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
                mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft]    = p3D;
                mvDepth[(*it)[0].queryIdx + monoLeft]             = depth;
                nMatches++;
            }
        }
    }
}

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    Eigen::Matrix3f mR;
    Eigen::Vector3f mt, twc;
    if(bRight) {
        Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
        Eigen::Vector3f trl = mTrl.translation();
        mR                  = Rrl * mRcw;
        mt                  = Rrl * mtcw + trl;
        twc                 = mRwc * mTlr.translation() + mOw;
    } else {
        mR  = mRcw;
        mt  = mtcw;
        twc = mOw;
    }

    // 3D in camera coordinates
    Eigen::Vector3f Pc  = mR * P + mt;
    const float Pc_dist = Pc.norm();
    const float &PcZ    = Pc(2);

    // Check positive depth
    if(PcZ < 0.0f)
        return false;

    // Project in image and check it is not outside
    Eigen::Vector2f uv;
    if(bRight)
        uv = mpCamera2->project(Pc);
    else
        uv = mpCamera->project(Pc);

    if(uv(0) < mnMinX || uv(0) > mnMaxX)
        return false;
    if(uv(1) < mnMinY || uv(1) > mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance  = pMP->GetMaxDistanceInvariance();
    const float minDistance  = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3f PO = P - twc;
    const float dist         = PO.norm();

    if(dist < minDistance || dist > maxDistance)
        return false;

    // Check viewing angle
    Eigen::Vector3f Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if(viewCos < viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist, this);

    if(bRight) {
        pMP->mTrackProjXR       = uv(0);
        pMP->mTrackProjYR       = uv(1);
        pMP->mnTrackScaleLevelR = nPredictedLevel;
        pMP->mTrackViewCosR     = viewCos;
        pMP->mTrackDepthR       = Pc_dist;
    } else {
        pMP->mTrackProjX       = uv(0);
        pMP->mTrackProjY       = uv(1);
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos     = viewCos;
        pMP->mTrackDepth       = Pc_dist;
    }

    return true;
}

Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i) {
    return mRwc * mvStereo3Dpoints[i] + mOw;
}

}    // namespace ORB_SLAM3
