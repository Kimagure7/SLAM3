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


#include "Tracking.h"

#include "Converter.h"
#include "FrameDrawer.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Pinhole.h"

#include <iostream>

#include <chrono>
#include <mutex>


using namespace std;

namespace ORB_SLAM3 {


Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, Settings *settings,
                   const cv::Ptr< cv::aruco::Dictionary > aruco_dict,
                   const int init_tag_id,
                   const float init_tag_size)
    : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
      mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
      mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
      mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(3),
      mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast< KeyFrame * >(NULL)),
      maruco_dict(aruco_dict), minit_tag_id(init_tag_id), minit_tag_size(init_tag_size) {
    // Load camera parameters from settings file
    if(settings) {
        newParameterLoader(settings);
    } else {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam) {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if(!b_parse_orb) {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO || sensor == System::IMU_RGBD) {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu) {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu) {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try {
                throw -1;
            } catch(exception &e) {
            }
        }
    }

    initID         = 0;
    lastID         = 0;
    mbInitWith3KFs = false;
    mnNumDataset   = 0;

    vector< GeometricCamera * > vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera *pCam : vpCams) {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
            std::cout << " is pinhole" << std::endl;
        } else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
            std::cout << " is fisheye" << std::endl;
        } else {
            std::cout << " is unknown" << std::endl;
        }
    }
    mbLoadedMap = pSys->isLoadingMap();
}

Tracking::~Tracking() {
    // f_track_stats.close();
}

void Tracking::newParameterLoader(Settings *settings) {
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if(settings->needToUndistort()) {
        mDistCoef = settings->camera1DistortionCoef();
    } else {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK                   = cv::Mat::eye(3, 3, CV_32F);
    mK.at< float >(0, 0) = mpCamera->getParameter(0);
    mK.at< float >(1, 1) = mpCamera->getParameter(1);
    mK.at< float >(0, 2) = mpCamera->getParameter(2);
    mK.at< float >(1, 2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0, 0) = mpCamera->getParameter(0);
    mK_(1, 1) = mpCamera->getParameter(1);
    mK_(0, 2) = mpCamera->getParameter(2);
    mK_(1, 2) = mpCamera->getParameter(3);

    if((mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && settings->cameraType() == Settings::KannalaBrandt) {
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    if(mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        mbf      = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if(mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB      = settings->rgb();

    // ORB parameters
    int nFeatures      = settings->nFeatures();
    int nLevels        = settings->nLevels();
    int fIniThFAST     = settings->initThFAST();
    int fMinThFAST     = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if(mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    // IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost   = settings->insertKFsWhenLost();
    mImuFreq         = settings->imuFrequency();
    mImuPer          = 0.001;    // 1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng         = settings->noiseGyro();
    float Na         = settings->noiseAcc();
    float Ngw        = settings->gyroWalk();
    float Naw        = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib     = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings) {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    cout << endl
         << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];

    if(sCameraName == "PinHole") {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal()) {
            fx = node.real();
        } else {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal()) {
            fy = node.real();
        } else {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal()) {
            cx = node.real();
        } else {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal()) {
            cy = node.real();
        } else {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal()) {
            mDistCoef.at< float >(0) = node.real();
        } else {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal()) {
            mDistCoef.at< float >(1) = node.real();
        } else {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal()) {
            mDistCoef.at< float >(2) = node.real();
        } else {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal()) {
            mDistCoef.at< float >(3) = node.real();
        } else {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal()) {
            mDistCoef.resize(5);
            mDistCoef.at< float >(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal()) {
            mImageScale = node.real();
        }

        if(b_miss_params) {
            return false;
        }

        if(mImageScale != 1.f) {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        vector< float > vCamCalib{ fx, fy, cx, cy };

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at< float >(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at< float >(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at< float >(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at< float >(3) << std::endl;

        if(mDistCoef.rows == 5)
            std::cout << "- k3: " << mDistCoef.at< float >(4) << std::endl;

        mK                   = cv::Mat::eye(3, 3, CV_32F);
        mK.at< float >(0, 0) = fx;
        mK.at< float >(1, 1) = fy;
        mK.at< float >(0, 2) = cx;
        mK.at< float >(1, 2) = cy;

        mK_.setIdentity();
        mK_(0, 0) = fx;
        mK_(1, 1) = fy;
        mK_(0, 2) = cx;
        mK_(1, 2) = cy;
    } else if(sCameraName == "KannalaBrandt8") {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal()) {
            fx = node.real();
        } else {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal()) {
            fy = node.real();
        } else {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal()) {
            cx = node.real();
        } else {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal()) {
            cy = node.real();
        } else {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal()) {
            k1 = node.real();
        } else {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal()) {
            k2 = node.real();
        } else {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal()) {
            k3 = node.real();
        } else {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal()) {
            k4 = node.real();
        } else {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal()) {
            mImageScale = node.real();
        }

        if(!b_miss_params) {
            if(mImageScale != 1.f) {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector< float > vCamCalib{ fx, fy, cx, cy, k1, k2, k3, k4 };
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK                   = cv::Mat::eye(3, 3, CV_32F);
            mK.at< float >(0, 0) = fx;
            mK.at< float >(1, 1) = fy;
            mK.at< float >(0, 2) = cx;
            mK.at< float >(1, 2) = cy;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 2) = cx;
            mK_(1, 2) = cy;
        }

        if(mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal()) {
                fx = node.real();
            } else {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal()) {
                fy = node.real();
            } else {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal()) {
                cx = node.real();
            } else {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal()) {
                cy = node.real();
            } else {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal()) {
                k1 = node.real();
            } else {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal()) {
                k2 = node.real();
            } else {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal()) {
                k3 = node.real();
            } else {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal()) {
                k4 = node.real();
            } else {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd   = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd   = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt()) {
                leftLappingBegin = node.operator int();
            } else {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt()) {
                leftLappingEnd = node.operator int();
            } else {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt()) {
                rightLappingBegin = node.operator int();
            } else {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt()) {
                rightLappingEnd = node.operator int();
            } else {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if(!node.empty()) {
                cvTlr = node.mat();
                if(cvTlr.rows != 3 || cvTlr.cols != 4) {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            } else {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params) {
                if(mImageScale != 1.f) {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin  = leftLappingBegin * mImageScale;
                    leftLappingEnd    = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd   = rightLappingEnd * mImageScale;
                }

                static_cast< KannalaBrandt8 * >(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast< KannalaBrandt8 * >(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector< float > vCamCalib2{ fx, fy, cx, cy, k1, k2, k3, k4 };
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast< KannalaBrandt8 * >(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast< KannalaBrandt8 * >(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl
                          << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n"
                          << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params) {
            return false;
        }

    } else {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal()) {
            mbf = node.real();
            if(mImageScale != 1.f) {
                mbf *= mImageScale;
            }
        } else {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    float fps = fSettings["Camera.fps"];
    if(fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB    = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        float fx          = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty() && node.isReal()) {
            mThDepth = node.real();
            mThDepth = mbf * mThDepth / fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        } else {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    if(mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal()) {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        } else {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    if(b_miss_params) {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings) {
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt()) {
        nFeatures = node.operator int();
    } else {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal()) {
        fScaleFactor = node.real();
    } else {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt()) {
        nLevels = node.operator int();
    } else {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt()) {
        fIniThFAST = node.operator int();
    } else {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt()) {
        fMinThFAST = node.operator int();
    } else {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params) {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if(mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    cout << endl
         << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings) {
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty()) {
        cvTbc = node.mat();
        if(cvTbc.rows != 4 || cvTbc.cols != 4) {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    } else {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    cout << endl;
    cout << "Left camera to Imu Transform (Tbc): " << endl
         << cvTbc << endl;
    Eigen::Matrix< float, 4, 4, Eigen::RowMajor > eigTbc(cvTbc.ptr< float >(0));
    Sophus::SE3f Tbc(eigTbc);

    node           = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if(!node.empty() && node.isInt()) {
        mInsertKFsLost = (bool)node.operator int();
    }

    if(!mInsertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;



    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt()) {
        mImuFreq = node.operator int();
        mImuPer  = 0.001;    // 1.0 / (double) mImuFreq;
    } else {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal()) {
        Ng = node.real();
    } else {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal()) {
        Na = node.real();
    } else {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal()) {
        Ngw = node.real();
    } else {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal()) {
        Naw = node.real();
    } else {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node      = fSettings["IMU.fastInit"];
    mFastInit = false;
    if(!node.empty()) {
        mFastInit = static_cast< int >(fSettings["IMU.fastInit"]) != 0;
    }

    if(mFastInit)
        cout << "Fast IMU initialization. Acceleration is not checked \n";

    if(b_miss_params) {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    cout << endl;
    cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer) {
    mpViewer = pViewer;
}

void Tracking::SetStepByStep(bool bSet) {
    bStepByStep = bSet;
}

bool Tracking::GetStepByStep() {
    return bStepByStep;
}



Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename) {
    // cout << "GrabImageStereo" << endl;

    mImGray             = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight            = imRectRight;

    if(mImGray.channels() == 3) {
        // cout << "Image with 3 channels" << endl;
        if(mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
        }
    } else if(mImGray.channels() == 4) {
        // cout << "Image with 4 channels" << endl;
        if(mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
        }
    }

    // cout << "Incoming frame creation" << endl;

    if(mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

    // cout << "Incoming frame ended" << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    // cout << "Tracking start" << endl;
    Track();
    // cout << "Tracking end" << endl;

    return mCurrentFrame.GetPose();
}


Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename) {
    mImGray         = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels() == 3) {
        if(mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if(mImGray.channels() == 4) {
        if(mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    if(mSensor == System::RGBD)
        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
    else if(mSensor == System::IMU_RGBD)
        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);






    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    Track();

    return mCurrentFrame.GetPose();
}

/**
 * @brief 抓取单目图像并进行视觉跟踪
 *
 * 此函数接收一个OpenCV图像矩阵、时间戳和文件名作为输入，首先将图像转换为灰度图（如果输入的是彩色图像），
 * 然后根据系统的传感器类型（单目或IMU单目）创建帧对象。该帧被用于初始化或进行后续的跟踪过程。
 * 如果系统状态是未初始化、没有接收过任何图像，或者从初始化到当前接收到的帧数少于最大允许帧数，
 * 则使用初始ORB特征提取器；否则使用主ORB特征提取器。最后，该函数执行跟踪，并返回当前帧的姿态。
 *
 * @param im 输入的OpenCV图像矩阵
 * @param timestamp 图像的时间戳
 * @param filename 图像文件名
 *
 * @return Sophus::SE3f 当前帧的姿态表示为Sophus库中的SE3浮点型对象
 */
Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename) {
    mImGray = im;
    if(mImGray.channels() == 3) {
        if(mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if(mImGray.channels() == 4) {
        if(mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if(mSensor == System::MONOCULAR) {
        if(mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
    } else if(mSensor == System::IMU_MONOCULAR) {
        if(mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib, maruco_dict);
        } else {
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib, maruco_dict);
        }
    }

    if(mState == NO_IMAGES_YET)
        t0 = timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    lastID = mCurrentFrame.mnId;

    Track();

    return mCurrentFrame.GetPose();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
    unique_lock< mutex > lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

/**
 * @brief 预积分IMU数据以更新当前帧的IMU状态
 *
 * 该函数负责从IMU队列中提取并处理自上一帧以来的所有IMU测量数据，预积分这些测量值以估计当前帧相对于上一帧的运动。
 * 它首先检查是否有前一帧的存在以及IMU队列是否非空。然后，它将收集所有相关的IMU点，并使用这些点来创建一个预积分对象，
 * 这个对象会根据加速度和角速度以及时间步长来更新其内部状态。最后，函数将更新后的预积分信息存储在当前帧中。
 *
 * @details
 * - 如果没有前一帧或IMU队列为空，则直接标记当前帧为已集成并返回。
 * - 从IMU队列中取出所有与当前和前一帧时间戳相关的测量数据，并进行预积分计算。
 */
void Tracking::PreintegrateIMU() {

    if(!mCurrentFrame.mpPrevFrame) {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0) {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true) {
        bool bSleep = false;
        {
            unique_lock< mutex > lock(mMutexImuQueue);
            if(!mlQueueImuData.empty()) {
                IMU::Point *m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
                    mlQueueImuData.pop_front();
                } else if(m->t < mCurrentFrame.mTimeStamp - mImuPer) {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                } else {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            } else {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }

    const int n = mvImuFromLastFrame.size() - 1;
    if(n == 0) {
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    IMU::Preintegrated *pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

    for(int i = 0; i < n; i++) {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if((i == 0) && (i < (n - 1))) {
            float tab  = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc        = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a - (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) * 0.5f;
            angVel     = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w - (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) * 0.5f;
            tstep      = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
        } else if(i < (n - 1)) {
            acc    = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
            tstep  = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
        } else if((i > 0) && (i == (n - 1))) {
            float tab  = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
            acc        = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a - (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) * 0.5f;
            angVel     = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w - (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) * 0.5f;
            tstep      = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
        } else if((i == 0) && (i == (n - 1))) {
            acc    = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep  = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if(!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated      = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame          = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    // Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}

/**
 * @brief 预测基于IMU的系统状态
 *
 * 此函数尝试预测系统的当前状态（位置、旋转和速度）基于最近的预积分结果和可能的关键帧信息。
 * 如果存在一个最近的关键帧并且地图已经更新，则使用关键帧的数据和预积分结果来预测新的姿态；否则，
 * 如果地图尚未更新，则使用上一帧的信息来进行预测。如果既没有关键帧也没有上一帧可用，或者地图没有更新，
 * 则返回false表示无法进行有效的预测。
 *
 * @return bool 成功预测则返回true，否则返回false
 */
bool Tracking::PredictStateIMU() {
    if(!mCurrentFrame.mpPrevFrame) {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame) {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias  = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else if(!mbMapUpdated) {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias  = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU() {
    // TODO To implement...
}


void Tracking::Track() {

    if(bStepByStep) {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while(!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu) {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        {
            // unique_lock< mutex > lock(mlMutexSave);
            //  logging before return
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(true);
            mlState.push_back(mState);
            return;
        }
    }

    Map *pCurrentMap = mpAtlas->GetCurrentMap();
    if(!pCurrentMap) {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    if(mState != NO_IMAGES_YET) {
        if(mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            cerr << "mLastFrame.mTimeStamp=" << mLastFrame.mTimeStamp << endl;
            cerr << "mCurrentFrame.mTimeStamp=" << mCurrentFrame.mTimeStamp << endl;
            unique_lock< mutex > lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            {
                // unique_lock< mutex > lock(mlMutexSave);
                //  logging before return
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(true);
                mlState.push_back(mState);
                return;
            }

        } else if(mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0) {
            // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
            // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial()) {

                if(mpAtlas->isImuInitialized()) {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2()) {
                        mpSystem->ResetActiveMap();
                    } else {
                        CreateMapInAtlas();
                    }
                } else {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }

                // logging before return
                {
                    // unique_lock< mutex > lock(mlMutexSave);
                    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                    mlpReferences.push_back(mlpReferences.back());
                    mlFrameTimes.push_back(mlFrameTimes.back());
                    mlbLost.push_back(true);
                    mlState.push_back(mState);
                    return;
                }
            }
        }
    }


    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState == NO_IMAGES_YET) {
        if(mbOnlyTracking) {
            // assume mbOnlyTracking will only be used for localizing from disk-loaded map
            // trigger relocalization in localization mode.
            mState     = LOST;
            mLastFrame = mCurrentFrame;
        } else {
            // trigger initialization.
            cout << "line 1269 keyframes " << pCurrentMap->KeyFramesInMap() << " MapPoints " << pCurrentMap->MapPointsInMap() << endl;
            if(pCurrentMap->KeyFramesInMap() == 0) {
                mState = NOT_INITIALIZED;
            } else {
                cout << "line 1273 ";
                cout << "KeyFrame::nNextId=" << KeyFrame::nNextId << endl;
                // Relocalization();
                mState = INIT_RELOCALIZE;
            }
        }
        cout << "No images yet. " << mState << endl;
    }

    mLastProcessedState = mState;

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap) {
        PreintegrateIMU();
    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock< mutex > lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex    = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex > nMapChangeIndex) {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }


    if(mState == NOT_INITIALIZED) {
        if(mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            StereoInitialization();
        } else {
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if(mState != OK)    // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            // cout<<"Initialization failed! tracking:1533.Still record the frames. mState "<< mState <<endl;
            {
                // unique_lock< mutex > lock(mlMutexSave);
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(true);
                mlState.push_back(mState);
                return;
            }
        }

        if(mpAtlas->GetAllMaps().size() == 1) {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    } else {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking) {
            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState == OK) {

                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                } else {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }


                if(!bOK) {
                    if(mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
                        mState = LOST;
                    } else if(pCurrentMap->KeyFramesInMap() > 10) {
                        cout << "Recently lost, KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState         = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    } else {
                        mState = LOST;
                    }
                }
            } else {
                if(mState == RECENTLY_LOST) {
                    // 如果追踪位姿还能狗住 后面tracklocalmap说不定能救回来
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if(mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost) {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    } else {
                        // Relocalization
                        bOK = Relocalization();
                        // std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        // std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if(mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                } else if(mState == LOST) {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if(pCurrentMap->KeyFramesInMap() < 10) {
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                    } else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast< KeyFrame * >(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
                    {
                        // unique_lock< mutex > lock(mlMutexSave);
                        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                        mlpReferences.push_back(mlpReferences.back());
                        mlFrameTimes.push_back(mlFrameTimes.back());
                        mlbLost.push_back(true);
                        mlState.push_back(mState);
                        return;
                    }
                    // logging before return

                } else if(mState == INIT_RELOCALIZE) {
                    bOK = Relocalization();
                    if(bOK) {
                        cout << "INIT_RELOCALIZE success!" << endl;
                        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

                        // update prev and next pointers
                        vector< KeyFrame * > vpKFs = pCurrentMap->GetAllKeyFrames();
                        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
                        KeyFrame *pKFend = vpKFs.back();
                        pKFcur->mPrevKF  = pKFend;
                        pKFend->mNextKF  = pKFcur;

                        // update timestamps
                        // goproslam 中时间戳都是0开始 需要把地图中旧的帧时间戳改掉
                        // 在修改rsd435i_rgbd后 实时的开始时间戳也是0了
                        cout << "vpKFs.back()->mpImuPreintegrated: " << vpKFs.back()->mpImuPreintegrated << endl;
                        double dt = 1.0 / 29.97;
                        if(vpKFs.back()->mpImuPreintegrated != nullptr) {
                            double dt = vpKFs.back()->mpImuPreintegrated->dT;
                        }
                        double t_offset = -vpKFs.back()->mTimeStamp - dt;
                        for(auto kf : vpKFs) {
                            kf->mTimeStamp += t_offset;
                        }

                        cout << "pKFCur->mTimeStamp: " << pKFcur->mTimeStamp << endl;
                        cout << "vpKFs.front()->mTimeStamp: " << vpKFs.front()->mTimeStamp << endl;
                        cout << "vpKFs.back()->mTimeStamp: " << vpKFs.back()->mTimeStamp << endl;

                        // update map point matches
                        // actual update is done in local mapper
                        // 这里似乎只是打印一下？
                        vector< MapPoint * > vpMapPointMatches = pKFcur->GetMapPointMatches();
                        cout << "pKFcur->GetMapPointMatches().size(): " << vpMapPointMatches.size() << endl;

                        cout << "1454 pKFcur->GetVelocity(): " << pKFcur->GetVelocity() << endl;

                        cout << "mCurrentFrame.mnId:" << mCurrentFrame.mnId << endl;

                        // update last keyframe
                        mnLastKeyFrameId = pKFcur->mnId;
                        mpLastKeyFrame   = pKFcur;

                        cout << "pKFcur->mnId " << pKFcur->mnId << endl;
                        mpAtlas->AddKeyFrame(pKFcur);
                        pKFcur->UpdateConnections();
                        mpLocalMapper->InsertKeyFrame(pKFcur);
                        mState = OK;
                    } else {
                        cout << "Relocalization() failed." << endl;
                    }
                }
            }

        } else {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState == LOST) {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            } else if(mState == RECENTLY_LOST) {
                if(pCurrentMap->isImuInitialized()) {
                    bOK = PredictStateIMU();
                } else {
                    bOK = false;
                }
                if(mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost) {
                    mState = LOST;
                    Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                    bOK = false;
                }
            } else {
                if(!mbVO) {
                    // In last frame we tracked enough MapPoints in the map
                    if(mbVelocity) {
                        bOK = TrackWithMotionModel();
                    } else {
                        bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM    = false;
                    bool bOKReloc = false;
                    vector< MapPoint * > vpMPsMM;
                    vector< bool > vbOutMM;
                    Sophus::SE3f TcwMM;
                    if(mbVelocity) {
                        bOKMM   = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM   = mCurrentFrame.GetPose();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier   = vbOutMM;

                        if(mbVO) {
                            for(int i = 0; i < mCurrentFrame.N; i++) {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    } else if(bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking) {
            if(bOK) {
                bOK = TrackLocalMap();
            }
            if(!bOK) {
                if(mbLoadedMap) {
                    // if we loaded map from file, it means we are running relocalization
                    // in that case we don't want to reset current map
                    // instead we should try to relocalize
                    mState = INIT_RELOCALIZE;
                    cout << "1552 Failed to track local map. Trying to relocalize..." << endl;
                } else {
                    cout << "Fail to track local map!" << endl;
                }
            }
        } else {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else if(mState == OK) {    // 意味着TrackLocalMap失败 但是前面的追踪位姿成功了
            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2()) {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState = RECENTLY_LOST;
            } else
                mState = RECENTLY_LOST;    // visual to lost

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized()) {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            // if (!mbOnlyTracking){
            //     Frame* pF = new Frame(mCurrentFrame);
            //     Frame* lF = new Frame(mLastFrame);
            //     pF->mpPrevFrame = lF;

            //     // Load preintegration
            //     pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            // }
        }

        if(pCurrentMap->isImuInitialized()) {
            if(bOK) {
                if(mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
                    cout << "RESETING FRAME!!!(actually nothing to do)" << mState << "mnID" << mCurrentFrame.mnId << endl;
                    ResetFrameIMU();
                } else if(mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

        // Update drawer
        mpFrameDrawer->Update(this);
        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        if(bOK || mState == RECENTLY_LOST) {
            // Update motion model
            if(mLastFrame.isSet() && mCurrentFrame.isSet()) {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                mVelocity            = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity           = true;
            } else {
                mbVelocity = false;
            }

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            for(int i = 0; i < mCurrentFrame.N; i++) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations() < 1) {
                        mCurrentFrame.mvbOutlier[i]   = false;
                        mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list< MapPoint * >::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++) {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            bool bNeedKF = NeedNewKeyFrame();

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if(bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
                CreateNewKeyFrame();


            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            for(int i = 0; i < mCurrentFrame.N; i++) {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // in localiztaion mode, do not reset map. just let it fail and wait for relocalization success
        if(mState == LOST && (!mbOnlyTracking)) {
            if(pCurrentMap->KeyFramesInMap() <= 10) {
                mpSystem->ResetActiveMap();

                // logging before return
                {
                    // unique_lock< mutex > lock(mlMutexSave);
                    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                    mlpReferences.push_back(mlpReferences.back());
                    mlFrameTimes.push_back(mlFrameTimes.back());
                    mlbLost.push_back(true);
                    mlState.push_back(mState);
                    return;
                }
            }
            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                if(!pCurrentMap->isImuInitialized()) {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    {
                        // unique_lock< mutex > lock(mlMutexSave);
                        //  logging before return
                        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                        mlpReferences.push_back(mlpReferences.back());
                        mlFrameTimes.push_back(mlFrameTimes.back());
                        mlbLost.push_back(true);
                        mlState.push_back(mState);
                        return;
                    }
                }

            CreateMapInAtlas();
            {
                // unique_lock< mutex > lock(mlMutexSave);
                //  logging before return
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(true);
                mlState.push_back(mState);
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }


    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    bool is_lost = !(mState == OK || mState == RECENTLY_LOST);
    if(!is_lost && mCurrentFrame.isSet()) {
        Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        {
            // unique_lock< mutex > lock(mlMutexSave);
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(is_lost);
            mlState.push_back(mState);
        }

    } else {
        // This can happen if tracking is lost
        {
            // unique_lock< mutex > lock(mlMutexSave);
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(is_lost);
            mlState.push_back(mState);
        }
    }
}


void Tracking::StereoInitialization() {
    if(mCurrentFrame.N > 500) {
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            if(!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
                cout << "not IMU meas" << endl;
                return;
            }

            if(!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() < 0.5) {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF     = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        } else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Create KeyFrame
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2) {
            for(int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if(z > 0) {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }
        } else {
            for(int i = 0; i < mCurrentFrame.Nleft; i++) {
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1) {
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini, i);
                    pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP, i);
                    pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]                                = pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        // cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame       = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame   = pKFini;
        // mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints           = mpAtlas->GetAllMapPoints();
        mpReferenceKF               = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState = OK;
    }
}

/**
 * @brief 初始化单目视觉追踪
 *
 * 该函数负责初始化单目视觉追踪过程。首先，它检查是否满足初始化条件，如关键点数量和时间间隔。
 * 如果条件不满足，则返回并等待下一次尝试。如果满足条件，函数会寻找初始帧与当前帧之间的特征匹配，
 * 并检查匹配数量是否足够。然后，使用两个视图的三角化方法来估计相机位姿和3D点位置。
 * 如果三角化成功，将设置初始帧和当前帧的位姿，并调用`CreateInitialMapMonocular`函数来创建地图。
 */
void Tracking::MonocularInitialization() {
    if(!mbReadyToInitializate) {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size() > 100) {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame    = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if(mSensor == System::IMU_MONOCULAR) {
                if(mpImuPreintegratedFromLastKF) {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF     = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    } else {
        if(((int)mCurrentFrame.mvKeys.size() <= 100) || ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 0.31))) {
            mbReadyToInitializate = false;
            std::cout << "Failed to init: n_keypoints " << (int)mCurrentFrame.mvKeys.size() << " time_since_init_frame  " << mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp << std::endl;
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 300);

        // Check if there are enough correspondences
        if(nmatches < 50) {
            mbReadyToInitializate = false;
            std::cout << "Failed to init: nmatches " << nmatches << std::endl;
            return;
        }

        Sophus::SE3f Tcw;
        vector< bool > vbTriangulated;    // Triangulated Correspondences (mvIniMatches)
        bool reconstruct_success = mpCamera->ReconstructWithTwoViews(
            mInitialFrame.mvKeysUn,
            mCurrentFrame.mvKeysUn,
            mvIniMatches,
            Tcw,
            mvIniP3D,
            vbTriangulated);
        std::cout << "init_success=" << reconstruct_success << endl;
        if(!reconstruct_success) {
            reconstruct_success = mpCamera->ReconstructWithTwoViewsAndTags(
                mInitialFrame.markerIds,
                mCurrentFrame.markerIds,
                mInitialFrame.markerCorners,
                mCurrentFrame.markerCorners,
                mInitialFrame.mvKeysUn,
                mCurrentFrame.mvKeysUn,
                mvIniMatches,
                minit_tag_id,
                minit_tag_size,
                Tcw,
                mvIniP3D,
                vbTriangulated);
            std::cout << " tag_init_success=" << reconstruct_success << endl;
        }

        if(reconstruct_success) {
            for(size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if(mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}


/**
 * @brief 为单目视觉创建初始地图
 *
 * 此函数用于在单目视觉模式下初始化地图。它首先通过当前帧和初始帧创建两个关键帧，
 * 并根据传感器类型处理IMU信息。然后，计算Bag of Words（BoW）表示并添加关键帧到地图中。
 * 接下来，基于初始匹配和3D点，创建新的MapPoint，并将它们与关键帧关联。更新所有相关连接，
 * 进行全局束调整优化，并根据深度信息缩放关键帧和MapPoint的位置。
 * 最后，检查初始化是否成功，如果失败则重置系统；否则，更新本地映射器、追踪状态等变量，
 * 并设置参考关键帧和地图点以进行后续处理。
 */
void Tracking::CreateInitialMapMonocular() {
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
    KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for(size_t i = 0; i < mvIniMatches.size(); i++) {
        if(mvIniMatches[i] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]]   = false;

        // Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set< MapPoint * > sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;    // 4.0f
    else
        invMedianDepth = 1.0f / medianDepth;

    if(medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50)    // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector< MapPoint * > vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if(vpAllMapPoints[iMP]) {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if(mSensor == System::IMU_MONOCULAR) {
        pKFcur->mPrevKF            = pKFini;
        pKFini->mNextKF            = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame   = pKFcur;
    // mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints           = mpAtlas->GetAllMapPoints();
    mpReferenceKF               = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector< KeyFrame * > vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity          = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) / (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;

    initID = pKFcur->mnId;
}

/**
 * @brief 在地图集中创建新地图
 *
 * 此函数用于在系统初始化或需要创建新地图时调用。它首先记录当前帧的ID作为初始化帧的ID，
 * 然后在地图集中创建一个新的地图实例。如果传感器类型是IMU相关的，将设置惯性传感器标志。
 * 接着，重置一些状态变量，包括设置初始帧ID为当前帧ID加1，将追踪状态设为NO_IMAGES_YET，
 * 以及清除与上一关键帧和参考关键帧相关的信息。最后，清除初始匹配列表并设置一个标记表示已创建新地图。
 */
void Tracking::CreateMapInAtlas() {
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if(mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState           = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL);
    mbVO = false;    // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
        mbReadyToInitializate = false;
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF) {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast< KeyFrame * >(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast< KeyFrame * >(NULL);

    mLastFrame    = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

/**
 * @brief 检查并替换上一帧中的地图点
 *
 * 该函数遍历上一帧中所有被观测到的地图点，检查它们是否已经被新的地图点所替代。
 * 如果某个地图点有替代者（即被标记为已删除并由另一个地图点取代），则在上一帧的特征列表中用新的地图点替换原地图点。
 * 这对于确保系统内部数据结构的一致性和准确性至关重要，尤其是在处理动态环境或进行局部映射更新时。
 */
void Tracking::CheckReplacedInLastFrame() {
    for(int i = 0; i < mLastFrame.N; i++) {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if(pMP) {
            MapPoint *pRep = pMP->GetReplaced();
            if(pRep) {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/**
 * @brief 跟踪参考关键帧
 *
 * 此函数通过使用当前帧与参考关键帧之间的ORB匹配来更新当前帧的姿态。
 * 首先，计算当前帧的Bag of Words向量，然后进行ORB匹配。如果找到足够的匹配点，
 * 则设置PnP求解器并优化姿态。最后，清除异常值，并检查剩余的匹配点数量以确定跟踪是否成功。
 *
 * @return true 跟踪成功
 * @return false 跟踪失败
 */
bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector< MapPoint * > vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if(nmatches < 15) {
        cout << "TRACK_REF_KF: Less than 15 matches!! current match = " << nmatches << endl;
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    // mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i = 0; i < mCurrentFrame.N; i++) {
        // if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
                mCurrentFrame.mvbOutlier[i]   = false;
                if(i < mCurrentFrame.Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView   = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

/**
 * @brief 更新上一帧的位姿并根据需要创建地图点
 *
 * 此函数首先更新上一帧相对于参考关键帧的位姿。如果当前跟踪模式不涉及关键帧更新或传感器为单目或仅进行跟踪，
 * 则直接返回。对于双目或RGB-D传感器，函数将基于深度信息创建“视觉里程计”地图点。
 * 地图点按照深度排序，并优先插入深度较小的点，直到达到预设条件。
 *
 * @param void 无输入参数
 */
void Tracking::UpdateLastFrame() {
    // Update pose according to reference keyframe
    KeyFrame *pRef   = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if(mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector< pair< float, int > > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i = 0; i < Nfeat; i++) {
        float z = mLastFrame.mvDepth[i];
        if(z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations() < 1)
            bCreateNew = true;

        if(bCreateNew) {
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1) {
                mLastFrame.UnprojectStereo(i, x3D);
            } else {
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint *pNewMP           = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else {
            nPoints++;
        }

        if(vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

/**
 * @brief 使用运动模型进行跟踪
 *
 * 该函数尝试使用上一帧和当前帧之间的运动模型预测并更新当前帧的姿态。
 * 如果IMU初始化并且不需要重置，则使用IMU预测状态，然后返回true。否则，使用上一帧的速度更新当前姿态，
 * 接着，通过投影上一帧的点到当前帧来搜索匹配点，并优化姿态。
 * 最后，检查匹配点的数量以确定跟踪是否成功。
 *
 * @return true 跟踪成功
 * @return false 跟踪失败
 */
bool Tracking::TrackWithMotionModel() {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if(mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    } else {
        cout << "TrackWithMotionModel: (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU " << mCurrentFrame.mnId << " > " << mnLastRelocFrameId <<"+"<< mnFramesToResetIMU << endl;
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }



    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast< MapPoint * >(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor == System::STEREO)
        th = 7;
    else
        th = 15;

    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches < 20) {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast< MapPoint * >(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);
    }

    if(nmatches < 20) {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i = 0; i < mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
                mCurrentFrame.mvbOutlier[i]   = false;
                if(i < mCurrentFrame.Nleft) {
                    pMP->mbTrackInView = false;
                } else {
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

/**
 * @brief TrackLocalMap - 跟踪局部地图
 *
 * 此函数尝试在局部地图中找到与当前帧匹配的特征点，优化相机姿态，并更新地图点统计信息。
 * 根据匹配的内点数量和系统传感器类型（单目、立体或RGB-D），决定跟踪是否成功。
 *
 * @return bool - 如果跟踪成功返回true，否则返回false
 */
bool Tracking::TrackLocalMap() {

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();

    // TOO check outliers before PO
    int aux1 = 0, aux2 = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
        if(mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    // if (!mpAtlas->isImuInitialized()){
    //  assuming mbOnlyTracking will only be on if we localize from disk loaded map
    if(mbOnlyTracking || !mpAtlas->isImuInitialized()) {
        Optimizer::PoseOptimization(&mCurrentFrame);
    } else {
        if(mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        } else {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated && mCurrentFrame.mpPrevFrame->mpcpi)    //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);    // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            } else {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);    // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
        if(mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i = 0; i < mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking) {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                } else
                    mnMatchesInliers++;
            } else if(mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
    if(mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 30) {
        cout << "TrackLocalMap() mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames (" << mCurrentFrame.mnId << " < " << mnLastRelocFrameId << " + " << mMaxFrames << " ) && mnMatchesInliers<30" << " = " << mnMatchesInliers << endl;
        return false;
    }

    if((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
        return true;


    if(mSensor == System::IMU_MONOCULAR) {
        // std::cout<< "Imu initialized: " << mpAtlas->isImuInitialized() << std::endl;
        if((mnMatchesInliers < 15 && mpAtlas->isImuInitialized())) {
            cout << "TrackLocalMap() mnMatchesInliers<15 && mpAtlas->isImuInitialized()" << " = " << mnMatchesInliers << endl;
            return false;
        } else if(mnMatchesInliers < 45 && !mpAtlas->isImuInitialized()) {
            cout << "TrackLocalMap() mnMatchesInliers<45 && !mpAtlas->isImuInitialized()" << " = " << mnMatchesInliers << endl;
            return false;
        } else {
            // cout << "TrackLocalMap() return true mnMatchesInliers" << " = " << mnMatchesInliers << endl;
            return true;
        }
    } else if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        if(mnMatchesInliers < 15) {
            return false;
        } else
            return true;
    } else {
        if(mnMatchesInliers < 30) {
            cout << "TrackLocalMap() mnMatchesInliers<30" << " = " << mnMatchesInliers << endl;
            return false;
        } else
            return true;
    }
}

/**
 * @brief NeedNewKeyFrame - 判断是否需要创建新的关键帧
 *
 * 该函数根据当前跟踪状态、传感器类型、地图点追踪情况以及局部映射的状态来决定是否应该插入新的关键帧。
 * 关键帧的创建条件包括时间间隔、与参考关键帧的匹配度、局部映射的空闲状态等。
 *
 * @return bool - 如果满足创建新关键帧的条件，返回true；否则返回false
 */
bool Tracking::NeedNewKeyFrame() {
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized()) {
        if(mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else if((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames) {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose    = 0;

    if(mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i = 0; i < N; i++) {
            if(mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
        // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs < 2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    if(mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2)
        thRefRatio = 0.75f;

    if(mSensor == System::IMU_MONOCULAR) {
        if(mnMatchesInliers > 350)    // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle);    // mpLocalMapper->KeyframesInQueue() < 2);
    // Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR && mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > 15);

    // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
    //  Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame) {
        if(mSensor == System::IMU_MONOCULAR) {
            if((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        } else if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            if((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR))    // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4 = true;
    else
        c4 = false;

    if(((c1a || c1b || c1c) && c2) || c3 || c4) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
            return true;
        } else {
            mpLocalMapper->InterruptBA();
            if(mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
                if(mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            } else {
                // std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    } else
        return false;
}

/**
 * @brief CreateNewKeyFrame - 创建新的关键帧
 *
 * 当确定需要创建新的关键帧时，此函数将负责生成并初始化一个新的关键帧对象，
 * 更新参考关键帧，并处理与IMU传感器相关的预积分过程。此外，对于非单目传感器，
 * 它还将基于当前深度图信息创建或更新MapPoints。
 */
void Tracking::CreateNewKeyFrame() {
    if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if(mpAtlas->isImuInitialized())    //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF               = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame) {
        pKF->mPrevKF            = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    } else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
    }

    if(mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)    // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        vector< pair< float, int > > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i = 0; i < N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if(z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if(!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for(size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations() < 1) {
                    bCreateNew                    = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast< MapPoint * >(NULL);
                }

                if(bCreateNew) {
                    Eigen::Vector3f x3D;

                    if(mCurrentFrame.Nleft == -1) {
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    } else {
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                        pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                } else {
                    nPoints++;
                }

                if(vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
                    break;
                }
            }
            // Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }
    }


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame   = pKF;
}

/**
 * @brief SearchLocalPoints 在当前帧中搜索局部地图点，以进行特征匹配和跟踪。
 *
 * 该函数首先清理当前帧中已经匹配的MapPoint，标记那些不好的MapPoint为NULL，
 * 并更新其他MapPoint的状态。然后，它检查局部地图中的每个MapPoint是否可见于当前帧，
 * 并统计需要匹配的点数。最后，使用ORBmatcher进行基于投影的特征匹配。
 * 匹配参数根据传感器类型、IMU初始化状态以及系统状态（如是否丢失）进行调整。
 */
void Tracking::SearchLocalPoints() {
    // Do not search map points already matched
    for(vector< MapPoint * >::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++) {
        MapPoint *pMP = *vit;
        if(pMP) {
            if(pMP->isBad()) {
                *vit = static_cast< MapPoint * >(NULL);
            } else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView   = false;
                pMP->mbTrackInViewR  = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for(vector< MapPoint * >::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++) {
        MapPoint *pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if(pMP->mbTrackInView) {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if(nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor == System::RGBD || mSensor == System::IMU_RGBD)
            th = 3;
        if(mpAtlas->isImuInitialized()) {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th = 2;
            else
                th = 6;
        } else if(!mpAtlas->isImuInitialized() && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;

        if(mState == LOST || mState == RECENTLY_LOST)    // Lost for less than 1 second
            th = 15;                                     // 15
        //cout << "mCurrentFramID: " << mCurrentFrame.mnId << " th: " << th << "mState:" << mState << endl;
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap() {
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

/**
 * @brief UpdateLocalPoints - 更新局部地图中的地图点
 *
 * 此函数负责更新局部地图中包含的地图点列表。它遍历所有局部关键帧，收集其中观察到的地图点，
 * 并确保每个地图点只被当前帧引用一次。这有助于保持局部地图的更新和一致性。
 */
void Tracking::UpdateLocalPoints() {
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for(vector< KeyFrame * >::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(), itEndKF = mvpLocalKeyFrames.rend(); itKF != itEndKF; ++itKF) {
        KeyFrame *pKF                    = *itKF;
        const vector< MapPoint * > vpMPs = pKF->GetMapPointMatches();

        for(vector< MapPoint * >::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {

            MapPoint *pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad()) {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 更新局部关键帧列表
 *
 * 此函数用于更新追踪线程中的局部关键帧列表。它通过统计每个地图点在哪些关键帧中被观测到，
 * 并将这些关键帧添加到局部地图中。同时，它还根据地图点的观测次数来确定哪个关键帧共享了最多的地图点，
 * 并将其设为参考关键帧。
 *
 * 这个过程包括以下步骤：
 * 1. 统计每个关键帧中的地图点数量。
 * 2. 清除并预留足够的空间给局部关键帧列表。
 * 3. 将所有包含至少一个地图点的关键帧加入到局部关键帧列表中，并标记它们与当前追踪的关联。
 * 4. 添加额外的关键邻域和子/父关系的关键帧，以丰富局部地图的信息。
 * 5. 如果系统使用IMU传感器，还会添加最近的临时性（时间上接近）的关键帧，以帮助IMU数据的融合。
 *
 */
void Tracking::UpdateLocalKeyFrames() {
    // Each map point vote for the keyframes in which it has been observed
    map< KeyFrame *, int > keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
        for(int i = 0; i < mCurrentFrame.N; i++) {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP) {
                if(!pMP->isBad()) {
                    const map< KeyFrame *, tuple< int, int > > observations = pMP->GetObservations();
                    for(map< KeyFrame *, tuple< int, int > >::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    } else {
        for(int i = 0; i < mLastFrame.N; i++) {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i]) {
                MapPoint *pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad()) {
                    const map< KeyFrame *, tuple< int, int > > observations = pMP->GetObservations();
                    for(map< KeyFrame *, tuple< int, int > >::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    }


    int max          = 0;
    KeyFrame *pKFmax = static_cast< KeyFrame * >(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map< KeyFrame *, int >::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++) {
        KeyFrame *pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second > max) {
            max    = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector< KeyFrame * >::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size() > 80)    // 80
            break;

        KeyFrame *pKF = *itKF;

        const vector< KeyFrame * > vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


        for(vector< KeyFrame * >::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++) {
            KeyFrame *pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad()) {
                if(pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set< KeyFrame * > spChilds = pKF->GetChilds();
        for(set< KeyFrame * >::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
            KeyFrame *pChildKF = *sit;
            if(!pChildKF->isBad()) {
                if(pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if(pParent) {
            if(pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mvpLocalKeyFrames.size() < 80) {
        KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i = 0; i < Nd; i++) {
            if(!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                tempKeyFrame                           = tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax) {
        mpReferenceKF               = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/**
 * @brief 执行重定位过程，当跟踪丢失时尝试恢复相机位置。
 *
 * 该函数首先计算当前帧的Bag of Words表示，然后从关键帧数据库中查询候选关键帧进行重定位。
 * 通过ORB匹配和PnP求解器来估计相机姿态并优化。如果找到足够多的匹配点（至少50个），则认为重定位成功。
 *
 * @return 如果重定位成功，则返回true；否则返回false。
 */
bool Tracking::Relocalization() {
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector< KeyFrame * > vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    // 0.75 -> 0.7
    ORBmatcher matcher(0.7, true);

    vector< MLPnPsolver * > vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector< vector< MapPoint * > > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector< bool > vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for(int i = 0; i < nKFs; i++) {
        KeyFrame *pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if(nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                MLPnPsolver *pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);    // This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while(nCandidates > 0 && !bMatch) {
        for(int i = 0; i < nKFs; i++) {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector< bool > vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver *pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw) {
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                set< MapPoint * > sFound;

                const int np = vbInliers.size();

                for(int j = 0; j < np; j++) {
                    if(vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood < 10)
                    continue;

                for(int io = 0; io < mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] = static_cast< MapPoint * >(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood < 50) {
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if(nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for(int ip = 0; ip < mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if(nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io = 0; io < mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch) {
        return false;
    } else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }
}

/**
 * @brief 重置整个系统状态，包括局部映射、闭环检测、数据库和地图。
 *
 * 此函数用于在出现跟踪失败或系统初始化前重置整个视觉SLAM系统的状态。
 * 它会停止并重置所有运行中的模块，清除数据库和地图信息，并将所有计数器归零。
 *
 * @param bLocMap 如果为true，则仅重置局部映射；如果为false，则进行完全重置。
 */
void Tracking::Reset(bool bLocMap) {
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer) {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if(!bLocMap) {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if(mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId    = 0;
    mState            = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit             = false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame      = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame         = Frame();
    mpReferenceKF      = static_cast< KeyFrame      *>(NULL);
    mpLastKeyFrame     = static_cast< KeyFrame     *>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

/**
 * @brief ResetActiveMap - 重置活动地图
 *
 * 此函数用于重置当前追踪器所使用的活动地图，这通常在系统初始化或遇到重大错误后调用。
 * 它会停止并重置所有相关模块（如局部映射器、闭环检测器），清除数据库和地图内容，
 * 并将追踪器的状态恢复到初始状态，准备重新开始处理图像帧。
 *
 * @param bLocMap 如果为true，则仅针对局部映射器进行重置；如果为false，则进行全面的重置。
 */
void Tracking::ResetActiveMap(bool bLocMap) {
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer) {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map *pMap = mpAtlas->GetCurrentMap();

    if(!bLocMap) {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap);    // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    // Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    // mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET;    // NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list< bool > lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map *pMap : mpAtlas->GetAllMaps()) {
        if(pMap->GetAllKeyFrames().size() > 0) {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    // cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list< bool >::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++) {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId   = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame  = Frame();
    mLastFrame     = Frame();
    mpReferenceKF  = static_cast< KeyFrame  *>(NULL);
    mpLastKeyFrame = static_cast< KeyFrame * >(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector< MapPoint * > Tracking::GetLocalMapMPS() {
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;

    cv::Mat K           = cv::Mat::eye(3, 3, CV_32F);
    K.at< float >(0, 0) = fx;
    K.at< float >(1, 1) = fy;
    K.at< float >(0, 2) = cx;
    K.at< float >(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at< float >(0) = fSettings["Camera.k1"];
    DistCoef.at< float >(1) = fSettings["Camera.k2"];
    DistCoef.at< float >(2) = fSettings["Camera.p1"];
    DistCoef.at< float >(3) = fSettings["Camera.p2"];
    const float k3          = fSettings["Camera.k3"];
    if(k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at< float >(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) {
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame) {
    Map *pMap                                    = pCurrentKeyFrame->GetMap();
    unsigned int index                           = mnFirstFrameId;
    list< ORB_SLAM3::KeyFrame * >::iterator lRit = mlpReferences.begin();
    list< bool >::iterator lbL                   = mlbLost.begin();
    for(auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbL++) {
        if(*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        while(pKF->isBad()) {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap) {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while(!mCurrentFrame.imuIsPreintegrated()) {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    } else {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12                  = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if(mCurrentFrame.mpImuPreintegrated) {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12                  = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                         twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                         Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset() {
    mnNumDataset++;
}

int Tracking::GetNumberDataset() {
    return mnNumDataset;
}

int Tracking::GetMatchesInliers() {
    return mnMatchesInliers;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder) {
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap) {
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if(!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale() {
    return mImageScale;
}

}    // namespace ORB_SLAM3