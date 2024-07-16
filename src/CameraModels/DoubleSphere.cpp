/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

// implemented by Steffen Urban, urbste@googlemail.com, 2021

#include "DoubleSphere.h"

#include <boost/serialization/export.hpp>

#include <Eigen/Core>

namespace ORB_SLAM3 {

cv::Point2f DoubleSphere::project(const cv::Point3f &p3D) {

  const double &xi = mvParameters[4];
  const double &alpha = mvParameters[5];

  const double &x = p3D.x;
  const double &y = p3D.y;
  const double &z = p3D.z;

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;

  const double r2 = xx + yy;

  const double d1_2 = r2 + zz;
  const double d1 = sqrt(d1_2);

  const double w1 = alpha > 0.5 ? (1.0 - alpha) / alpha : alpha / (1.0 - alpha);
  const double w2 = (w1 + xi) / sqrt(2.0 * w1 * xi + xi * xi + 1.0);
  if (z <= -w2 * d1) {
    return cv::Point2f(0.f, 0.f);
  }

  const double k = xi * d1 + z;
  const double kk = k * k;

  const double d2_2 = r2 + kk;
  const double d2 = sqrt(d2_2);

  const double norm = alpha * d2 + (double(1) - alpha) * k;

  const double mx = x / norm;
  const double my = y / norm;

  return cv::Point2f(mvParameters[0] * mx + mvParameters[2],
                     mvParameters[1] * my + mvParameters[3]);
}

Eigen::Vector2f DoubleSphere::project(const Eigen::Vector3f &m3D) {
  return this->project(Eigen::Vector3d(m3D(0), m3D(1), m3D(2)));
}

Eigen::Vector2d DoubleSphere::project(const Eigen::Vector3d &v3D) {

  Eigen::Vector2d res;
  res.setZero();
  const double &xi = mvParameters[4];
  const double &alpha = mvParameters[5];

  const double &x = v3D[0];
  const double &y = v3D[1];
  const double &z = v3D[2];

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;

  const double r2 = xx + yy;

  const double d1_2 = r2 + zz;
  const double d1 = sqrt(d1_2);

  const double w1 = alpha > 0.5 ? (1.0 - alpha) / alpha : alpha / (1.0 - alpha);
  const double w2 = (w1 + xi) / sqrt(2.0 * w1 * xi + xi * xi + 1.0);
  if (z <= -w2 * d1) {
    return res;
  }

  const double k = xi * d1 + z;
  const double kk = k * k;

  const double d2_2 = r2 + kk;
  const double d2 = sqrt(d2_2);

  const double norm = alpha * d2 + (double(1) - alpha) * k;

  const double mx = x / norm;
  const double my = y / norm;

  res[0] = mvParameters[0] * mx + mvParameters[2];
  res[1] = mvParameters[1] * my + mvParameters[3];

  return res;
}

Eigen::Vector2f DoubleSphere::projectMat(const cv::Point3f &p3D) {
  const cv::Point2f point = this->project(p3D);
  return Eigen::Vector2f(point.x, point.y);
}

float DoubleSphere::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) {
  return 1.f;
}

Eigen::Vector3f DoubleSphere::unprojectEig(const cv::Point2f &p2D) {
  const cv::Point3f ray = this->unproject(p2D);
  return Eigen::Vector3f(ray.x,ray.y,ray.z);
}

cv::Point3f DoubleSphere::unproject(const cv::Point2f &p2D) {

  const double &xi = mvParameters[4];
  const double &alpha = mvParameters[5];

  const double mx = (p2D.x - mvParameters[2]) / mvParameters[0];
  const double my = (p2D.y - mvParameters[3]) / mvParameters[1];

  const double r2 = mx * mx + my * my;

  if (alpha > double(0.5)) {
    if (r2 >= double(1) / (double(2) * alpha - double(1)))
      return cv::Point3f(0.f, 0.f, 1.f);
  }

  const double xi2_2 = alpha * alpha;
  const double xi1_2 = xi * xi;

  const double sqrt2 = sqrt(double(1) - (double(2) * alpha - double(1)) * r2);

  const double norm2 = alpha * sqrt2 + double(1) - alpha;

  const double mz = (double(1) - xi2_2 * r2) / norm2;
  const double mz2 = mz * mz;

  const double norm1 = mz2 + r2;
  const double sqrt1 = sqrt(mz2 + (double(1) - xi1_2) * r2);
  const double k = (mz * xi + sqrt1) / norm1;

  const double x = k * mx;
  const double y = k * my;
  const double z = k * mz - xi;

  return cv::Point3f(x / z, y / z, 1.f);
}

Eigen::Matrix<double, 2, 3>
DoubleSphere::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> JacGood;
  JacGood.setZero();
  const double &xi = mvParameters[4];
  const double &alpha = mvParameters[5];

  const double &fx = mvParameters[0];
  const double &fy = mvParameters[1];

  const double &x = v3D[0];
  const double &y = v3D[1];
  const double &z = v3D[2];

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;

  const double r2 = xx + yy;

  const double d1_2 = r2 + zz;
  const double d1 = sqrt(d1_2);

  const double w1 = alpha > 0.5 ? (1.0 - alpha) / alpha : alpha / (1.0 - alpha);
  const double w2 = (w1 + xi) / sqrt(2.0 * w1 * xi + xi * xi + 1.0);
  if (z <= -w2 * d1) {
    return JacGood;
  }

  const double k = xi * d1 + z;
  const double kk = k * k;

  const double d2_2 = r2 + kk;
  const double d2 = sqrt(d2_2);

  const double norm = alpha * d2 + (double(1) - alpha) * k;
  const double norm2 = norm * norm;
  const double xy = x * y;
  const double tt2 = xi * z / d1 + double(1);

  const double d_norm_d_r2 =
      (xi * (double(1) - alpha) / d1 + alpha * (xi * k / d1 + double(1)) / d2) /
      norm2;

  const double tmp2 =
      ((double(1) - alpha) * tt2 + alpha * k * tt2 / d2) / norm2;

  JacGood(0, 0) = fx * (double(1) / norm - xx * d_norm_d_r2);
  JacGood(1, 0) = -fy * xy * d_norm_d_r2;

  JacGood(0, 1) = -fx * xy * d_norm_d_r2;
  JacGood(1, 1) = fy * (double(1) / norm - yy * d_norm_d_r2);

  JacGood(0, 2) = -fx * x * tmp2;
  JacGood(1, 2) = -fy * y * tmp2;

  return JacGood;
}

bool DoubleSphere::ReconstructWithTwoViews(
        const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
        Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) {
  if (!tvr) {
    tvr = new TwoViewReconstruction(this->toK_());
  }

  // Correct FishEye distortion
  std::vector<cv::KeyPoint> vKeysUn1(vKeys1.size()), vKeysUn2(vKeys2.size());
  for (size_t i = 0; i < vKeys1.size(); i++) {
    const cv::Point3f pt3 = this->unproject(vKeys1[i].pt);
    vKeysUn1[i].pt.x = pt3.x / pt3.z;
    vKeysUn1[i].pt.y = pt3.y / pt3.z;
  }
  for (size_t i = 0; i < vKeys2.size(); i++) {
    const cv::Point3f pt3 = this->unproject(vKeys2[i].pt);
    vKeysUn2[i].pt.x = pt3.x / pt3.z;
    vKeysUn2[i].pt.y = pt3.y / pt3.z;
  }
  return tvr->Reconstruct(vKeysUn1,vKeysUn2,vMatches12,T21,vP3D,vbTriangulated);
}

cv::Mat DoubleSphere::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2],
               0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
  return K;
}

Eigen::Matrix3f DoubleSphere::toK_() {
  Eigen::Matrix3f K;
  K<<mvParameters[0],
                0.f,
                mvParameters[2],
                0.f,
                mvParameters[1],
                mvParameters[3],
                0.f,
                0.f,
                1.f;
  return K;
}

bool DoubleSphere::epipolarConstrain(GeometricCamera* pCamera2,
    const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
    const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12,
    const float sigmaLevel, const float unc) {
   Eigen::Vector3f p3D;
   return this->TriangulateMatches(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > 0.0001f;
}

bool DoubleSphere::matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                       Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                       const float sigmaLevel1, const float sigmaLevel2,
                                       Eigen::Vector3f& x3Dtriangulated){
  Eigen::Matrix<float,3,4> eigTcw1 = Tcw1.matrix3x4();
  Eigen::Matrix3f Rcw1 = eigTcw1.block<3,3>(0,0);
  Eigen::Matrix3f Rwc1 = Rcw1.transpose();
  Eigen::Matrix<float,3,4> eigTcw2 = Tcw2.matrix3x4();
  Eigen::Matrix3f Rcw2 = eigTcw2.block<3,3>(0,0);
  Eigen::Matrix3f Rwc2 = Rcw2.transpose();

  cv::Point3f ray1c = this->unproject(kp1.pt);
  cv::Point3f ray2c = pOther->unproject(kp2.pt);

  Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
  Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

  //Check parallax between rays
  Eigen::Vector3f ray1 = Rwc1 * r1;
  Eigen::Vector3f ray2 = Rwc2 * r2;

  const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

  //If parallax is lower than 0.9998, reject this match
  if(cosParallaxRays > 0.9998){
      return false;
  }

  //Parallax is good, so we try to triangulate
  cv::Point2f p11,p22;

  p11.x = ray1c.x;
  p11.y = ray1c.y;

  p22.x = ray2c.x;
  p22.y = ray2c.y;

  Eigen::Vector3f x3D;

  this->Triangulate(p11,p22,eigTcw1,eigTcw2,x3D);

  //Check triangulation in front of cameras
  float z1 = Rcw1.row(2).dot(x3D)+Tcw1.translation()(2);
  if(z1<=0){  //Point is not in front of the first camera
      return false;
  }


  float z2 = Rcw2.row(2).dot(x3D)+Tcw2.translation()(2);
  if(z2<=0){ //Point is not in front of the first camera
      return false;
  }

  //Check reprojection error in first keyframe
  //  -Transform point into camera reference system
  Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
  Eigen::Vector2f uv1 = this->project(x3D1);

  float errX1 = uv1(0) - kp1.pt.x;
  float errY1 = uv1(1) - kp1.pt.y;

  if((errX1*errX1+errY1*errY1)>5.991*sigmaLevel1){   //Reprojection error is high
      return false;
  }

  //Check reprojection error in second keyframe;
  //  -Transform point into camera reference system
  Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation(); // avoid using q
  Eigen::Vector2f uv2 = pOther->project(x3D2);

  float errX2 = uv2(0) - kp2.pt.x;
  float errY2 = uv2(1) - kp2.pt.y;

  if((errX2*errX2+errY2*errY2)>5.991*sigmaLevel2){   //Reprojection error is high
      return false;
  }

  //Since parallax is big enough and reprojection errors are low, this pair of points
  //can be considered as a match
  x3Dtriangulated = x3D;

  return true;
}

float DoubleSphere::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc, Eigen::Vector3f& p3D) {

    Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
    Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);

    //Check parallax
    Eigen::Vector3f r21 = R12 * r2;

    const float cosParallaxRays = r1.dot(r21)/(r1.norm() *r21.norm());

    if(cosParallaxRays > 0.9998){
        return -1;
    }

    //Parallax is good, so we try to triangulate
    cv::Point2f p11,p22;

    p11.x = r1[0];
    p11.y = r1[1];

    p22.x = r2[0];
    p22.y = r2[1];

    Eigen::Vector3f x3D;
    Eigen::Matrix<float,3,4> Tcw1;
    Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float,3,4> Tcw2;

    Eigen::Matrix3f R21 = R12.transpose();
    Tcw2 << R21, -R21 * t12;


    Triangulate(p11,p22,Tcw1,Tcw2,x3D);
    // cv::Mat x3Dt = x3D.t();

    float z1 = x3D(2);
    if(z1 <= 0){
        return -2;
    }

    float z2 = R21.row(2).dot(x3D)+Tcw2(2,3);
    if(z2<=0){
        return -3;
    }

    //Check reprojection error
    Eigen::Vector2f uv1 = this->project(x3D);

    float errX1 = uv1(0) - kp1.pt.x;
    float errY1 = uv1(1) - kp1.pt.y;

    if((errX1*errX1+errY1*errY1)>5.991 * sigmaLevel){   //Reprojection error is high
        return -4;
    }

    Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
    Eigen::Vector2f uv2 = pCamera2->project(x3D2);

    float errX2 = uv2(0) - kp2.pt.x;
    float errY2 = uv2(1) - kp2.pt.y;

    if((errX2*errX2+errY2*errY2)>5.991 * unc){   //Reprojection error is high
        return -5;
    }

    p3D = x3D;

    return z1;
}

std::ostream &operator<<(std::ostream &os, const DoubleSphere &kb) {
  os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " "
     << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
     << kb.mvParameters[4] << " " << kb.mvParameters[5];
  return os;
}

std::istream &operator>>(std::istream &is, DoubleSphere &kb) {
  float nextParam;
  for (int i = 0; i < kb.NUM_PARAMS_; i++) {
    assert(is.good()); // Make sure the input stream is good
    is >> nextParam;
    kb.mvParameters[i] = nextParam;
  }
  return is;
}

void DoubleSphere::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float,3,4> &Tcw1,
                               const Eigen::Matrix<float,3,4> &Tcw2, Eigen::Vector3f &x3D)
{
  Eigen::Matrix<float,4,4> A;
  A.row(0) = p1.x*Tcw1.row(2)-Tcw1.row(0);
  A.row(1) = p1.y*Tcw1.row(2)-Tcw1.row(1);
  A.row(2) = p2.x*Tcw2.row(2)-Tcw2.row(0);
  A.row(3) = p2.y*Tcw2.row(2)-Tcw2.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4f x3Dh = svd.matrixV().col(3);
  x3D = x3Dh.head(3)/x3Dh(3);
}
} // namespace ORB_SLAM3
