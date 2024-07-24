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

#include "Pinhole.h"
#include <opencv2/aruco.hpp>
#include <boost/serialization/export.hpp>

//BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Pinhole)

namespace ORB_SLAM3 {
//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

    long unsigned int GeometricCamera::nNextId=0;

    cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
        return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                           mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
    }

    Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
        Eigen::Vector2d res;
        res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
        res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];

        return res;
    }

    Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &v3D) {
        Eigen::Vector2f res;
        res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
        res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];

        return res;
    }

    Eigen::Vector2f Pinhole::projectMat(const cv::Point3f &p3D) {
        cv::Point2f point = this->project(p3D);
        return Eigen::Vector2f(point.x, point.y);
    }

    float Pinhole::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
    {
        return 1.0;
    }

    Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D) {
        return Eigen::Vector3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
                           1.f);
    }

    cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
        return cv::Point3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
                           1.f);
    }

    Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
        Eigen::Matrix<double, 2, 3> Jac;
        Jac(0, 0) = mvParameters[0] / v3D[2];
        Jac(0, 1) = 0.f;
        Jac(0, 2) = -mvParameters[0] * v3D[0] / (v3D[2] * v3D[2]);
        Jac(1, 0) = 0.f;
        Jac(1, 1) = mvParameters[1] / v3D[2];
        Jac(1, 2) = -mvParameters[1] * v3D[1] / (v3D[2] * v3D[2]);

        return Jac;
    }

    bool Pinhole::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                 Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated){
        if(!tvr){
            Eigen::Matrix3f K = this->toK_();
            tvr = new TwoViewReconstruction(K);
        }

        return tvr->Reconstruct(vKeys1,vKeys2,vMatches12,T21,vP3D,vbTriangulated);
    }
    bool Pinhole::ReconstructWithTwoViewsAndTags(
            const std::vector<int>& markerIds1,
            const std::vector<int>& markerIds2,
            const std::vector<std::vector<cv::Point2f>>& markerCorners1,
            const std::vector<std::vector<cv::Point2f>>& markerCorners2,
            const std::vector<cv::KeyPoint>& vKeys1, 
            const std::vector<cv::KeyPoint>& vKeys2,
            const std::vector<int> &vMatches12,
            const int tag_id,
            const float tag_size,
            Sophus::SE3f &T21,
            std::vector<cv::Point3f> &vP3D, 
            std::vector<bool> &vbTriangulated
        ){
        if(!tvr){
            Eigen::Matrix3f K = this->toK_();
            tvr = new TwoViewReconstruction(K);
        }
        vP3D.clear();
        vbTriangulated.clear();

        // Check if tags are avaliable
        auto tag_iter1 = std::find(markerIds1.begin(), markerIds1.end(), tag_id);
        auto tag_iter2 = std::find(markerIds2.begin(), markerIds2.end(), tag_id);
        if ((tag_iter1 == markerIds1.end()) || (tag_iter2 == markerIds2.end())){
            // tag not detected in both frames return false
            return false;
        }
        int tag_idx1 = tag_iter1 - markerIds1.begin();
        int tag_idx2 = tag_iter2 - markerIds2.begin();

        // undistort corners
        cv::Mat Dc = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
        cv::Mat Rc = cv::Mat::eye(3,3,CV_32F);
        cv::Mat Kc = this->toK();

        std::vector<std::vector<cv::Point2f>> tag1Corners(1, std::vector<cv::Point2f>(markerCorners1[tag_idx1].size()));
        std::vector<std::vector<cv::Point2f>> tag2Corners(1, std::vector<cv::Point2f>(markerCorners2[tag_idx2].size()));
        cv::fisheye::undistortPoints(markerCorners1[tag_idx1], tag1Corners[0], Kc,Dc,Rc,Kc);
        cv::fisheye::undistortPoints(markerCorners2[tag_idx2], tag2Corners[0], Kc,Dc,Rc,Kc);

        // localize tags
        cv::Mat distCoeffs = cv::Mat1f(5, 1);
        std::vector<cv::Vec3d> rvecs1, tvecs1, rvecs2, tvecs2;
        cv::aruco::estimatePoseSingleMarkers(tag1Corners, tag_size, Kc, distCoeffs, rvecs1, tvecs1);
        cv::aruco::estimatePoseSingleMarkers(tag2Corners, tag_size, Kc, distCoeffs, rvecs2, tvecs2);

        // if result contains Nan, pose estimation has failed.
        if (rvecs1[0][0] != rvecs1[0][0]) {return false;}
        if (rvecs2[0][0] != rvecs2[0][0]) {return false;}

        // calculate relative pose
        cv::Mat R1w;
        cv::Rodrigues(rvecs1[0], R1w);
        R1w.convertTo(R1w, CV_32F); // convert double to float, required
        Sophus::SE3f T1w(Converter::toMatrix3f(R1w), Converter::toVector3f(tvecs1[0]));

        cv::Mat R2w;
        cv::Rodrigues(rvecs2[0], R2w);
        R2w.convertTo(R2w, CV_32F);
        Sophus::SE3f T2w(Converter::toMatrix3f(R2w), Converter::toVector3f(tvecs2[0]));

        T21 = T2w * T1w.inverse();
        
        // reconstruct
        std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
        std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());
        for(size_t i = 0; i < vKeys1.size(); i++) vPts1[i] = vKeys1[i].pt;
        for(size_t i = 0; i < vKeys2.size(); i++) vPts2[i] = vKeys2[i].pt;
        cv::fisheye::undistortPoints(vPts1,vPts1,Kc,Dc,Rc,Kc);
        cv::fisheye::undistortPoints(vPts2,vPts2,Kc,Dc,Rc,Kc);
        for(size_t i = 0; i < vKeys1.size(); i++) vKeysUn1[i].pt = vPts1[i];
        for(size_t i = 0; i < vKeys2.size(); i++) vKeysUn2[i].pt = vPts2[i];
        return tvr->ReconstructWithTag(vKeysUn1,vKeysUn2,vMatches12,T21,vP3D,vbTriangulated);
    }


    cv::Mat Pinhole::toK() {
        cv::Mat K = (cv::Mat_<float>(3, 3)
                << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return K;
    }

    Eigen::Matrix3f Pinhole::toK_() {
        Eigen::Matrix3f K;
        K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
        return K;
    }


    bool Pinhole::epipolarConstrain(GeometricCamera* pCamera2,  const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) {
        //Compute Fundamental Matrix
        Eigen::Matrix3f t12x = Sophus::SO3f::hat(t12);
        Eigen::Matrix3f K1 = this->toK_();
        Eigen::Matrix3f K2 = pCamera2->toK_();
        Eigen::Matrix3f F12 = K1.transpose().inverse() * t12x * R12 * K2.inverse();
        
        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = kp1.pt.x*F12(0,0)+kp1.pt.y*F12(1,0)+F12(2,0);
        const float b = kp1.pt.x*F12(0,1)+kp1.pt.y*F12(1,1)+F12(2,1);
        const float c = kp1.pt.x*F12(0,2)+kp1.pt.y*F12(1,2)+F12(2,2);

        const float num = a*kp2.pt.x+b*kp2.pt.y+c;

        const float den = a*a+b*b;

        if(den==0)
            return false;

        const float dsqr = num*num/den;

        return dsqr<3.84*unc;
    }

    std::ostream & operator<<(std::ostream &os, const Pinhole &ph) {
        os << ph.mvParameters[0] << " " << ph.mvParameters[1] << " " << ph.mvParameters[2] << " " << ph.mvParameters[3];
        return os;
    }

    std::istream & operator>>(std::istream &is, Pinhole &ph) {
        float nextParam;
        for(size_t i = 0; i < 4; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            ph.mvParameters[i] = nextParam;

        }
        return is;
    }

    bool Pinhole::IsEqual(GeometricCamera* pCam)
    {
        if(pCam->GetType() != GeometricCamera::CAM_PINHOLE)
            return false;

        Pinhole* pPinholeCam = (Pinhole*) pCam;

        if(size() != pPinholeCam->size())
            return false;

        bool is_same_camera = true;
        for(size_t i=0; i<size(); ++i)
        {
            if(abs(mvParameters[i] - pPinholeCam->getParameter(i)) > 1e-6)
            {
                is_same_camera = false;
                break;
            }
        }
        return is_same_camera;
    }
}
