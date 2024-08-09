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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "KannalaBrandt8.h"
#include "Pinhole.h"

namespace ORB_SLAM3 {

/**
 * @brief 构造一个Atlas对象
 */
Atlas::Atlas() {
    mpCurrentMap = static_cast< Map * >(NULL);
}

/**
 * @brief 构造一个带有初始关键帧ID的Atlas对象
 * @param initKFid 初始关键帧ID
 */
Atlas::Atlas(int initKFid)
    : mnLastInitKFidMap(initKFid), mHasViewer(false) {
    mpCurrentMap = static_cast< Map * >(NULL);
    CreateNewMap();
}


/**
 * @brief 析构函数，释放所有地图资源
 */
Atlas::~Atlas() {
    for(std::set< Map * >::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;) {
        Map *pMi = *it;

        if(pMi) {
            delete pMi;
            pMi = static_cast< Map * >(NULL);

            it = mspMaps.erase(it);
        } else
            ++it;
    }
}

/**
 * @brief 创建一个新的地图实例，并设置为当前地图
 */
void Atlas::CreateNewMap() {
    unique_lock< mutex > lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap) {
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;    // The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        // if(mHasViewer)
        //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

/**
 * @brief 更改当前地图为指定的地图实例
 * @param pNewMp 新的地图实例指针
 */
void Atlas::ChangeMap(Map *pMap) {
    unique_lock< mutex > lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap) {
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer *pViewer) {
    mpViewer   = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame *pKF) {
    Map *pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint *pMP) {
    Map *pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam) {
    // Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam      = -1;
    for(size_t i = 0; i < mvpCameras.size(); ++i) {
        GeometricCamera *pCam_i = mvpCameras[i];
        if(!pCam)
            std::cout << "Not pCam" << std::endl;
        if(!pCam_i)
            std::cout << "Not pCam_i" << std::endl;
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
            if(((Pinhole *)pCam_i)->IsEqual(pCam)) {
                bAlreadyInMap = true;
                index_cam     = i;
            }
        } else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
            if(((KannalaBrandt8 *)pCam_i)->IsEqual(pCam)) {
                bAlreadyInMap = true;
                index_cam     = i;
            }
        }
    }

    if(bAlreadyInMap) {
        return mvpCameras[index_cam];
    } else {
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector< GeometricCamera * > Atlas::GetAllCameras() {
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector< MapPoint * > &vpMPs) {
    unique_lock< mutex > lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
    unique_lock< mutex > lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector< KeyFrame * > Atlas::GetAllKeyFrames() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector< MapPoint * > Atlas::GetAllMapPoints() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector< MapPoint * > Atlas::GetReferenceMapPoints() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

/**
 * @brief 获取所有地图的指针列表，按地图ID排序
 * 
 * 此函数在互斥锁保护下获取当前地图集中的所有地图实例的指针，并将其存储在一个向量中。
 * 向量中的地图按照其ID进行排序，以确保每次调用时返回的地图顺序一致。
 * 
 * @return vector<Map*> 包含所有地图实例指针的向量，按ID排序
 */
vector< Map * > Atlas::GetAllMaps() {
    unique_lock< mutex > lock(mMutexAtlas);
    struct compFunctor {
        inline bool operator()(Map *elem1, Map *elem2) {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector< Map * > vMaps(mspMaps.begin(), mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap() {
    unique_lock< mutex > lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
    unique_lock< mutex > lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap      = static_cast< Map      *>(NULL);
    mnLastInitKFidMap = 0;
}

Map *Atlas::GetCurrentMap() {
    unique_lock< mutex > lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map *pMap) {
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() {
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
    unique_lock< mutex > lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
    unique_lock< mutex > lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
    unique_lock< mutex > lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

/**
 * @brief Atlas::PreSave 在保存地图前的预处理函数，用于更新关键帧ID并备份当前地图信息。
 * 
 * 该函数首先更新初始化关键帧ID，确保它总是当前最大关键帧ID的下一个。然后，它复制所有地图到备份向量中，
 * 并按照地图的ID进行排序。接着，对于每个非空且有效的地图，如果该地图没有关键帧，则标记为坏图并跳过；
 * 否则，调用该地图的PreSave方法来进一步准备保存工作。
 */
void Atlas::PreSave() {
    if(mpCurrentMap) {
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;    // The init KF is the next of current maximum
    }

    struct compFunctor {
        inline bool operator()(Map *elem1, Map *elem2) {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set< GeometricCamera * > spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map *pMi : mvpBackupMaps) {
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

/**
 * @brief Atlas::PostLoad 负载后处理函数，用于恢复和整合所有地图数据。
 *
 * 该函数首先创建一个相机映射以方便查找相机。然后清空当前的地图集合，并遍历所有备份的地图，
 * 将它们插入到当前的地图集合中，并调用每个地图的PostLoad方法来恢复数据。同时计算总的关键帧和特征点数量。
 */
void Atlas::PostLoad() {
    map< unsigned int, GeometricCamera * > mpCams;
    for(GeometricCamera *pCam : mvpCameras) {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for(Map *pMi : mvpBackupMaps) {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB) {
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase *Atlas::GetKeyFrameDatabase() {
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc) {
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary *Atlas::GetORBVocabulary() {
    return mpORBVocabulary;
}
/**
 * @brief Atlas::GetNumLivedKF 计算并返回当前所有地图中的活动关键帧数量。
 *
 * 该函数首先锁定Atlas的互斥锁以确保线程安全。然后，遍历所有地图，累加每个地图的关键帧数量，
 * 最后返回总的关键帧数量。
 */
long unsigned int Atlas::GetNumLivedKF() {
    unique_lock< mutex > lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map *pMap_i : mspMaps) {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

/**
 * @brief Atlas::GetNumLivedMP 计算并返回当前所有地图中的活动特征点数量。
 *
 * 与GetNumLivedKF类似，该函数也首先锁定Atlas的互斥锁以确保线程安全。然后，遍历所有地图，
 * 累加每个地图的特征点数量，并返回总的特征点数量。
 */
long unsigned int Atlas::GetNumLivedMP() {
    unique_lock< mutex > lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map *pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}


/**
 * @brief Atlas::GetAtlasKeyframes 返回一个包含所有关键帧的地图（字典），键为关键帧ID，值为关键帧指针。
 *
 * 遍历备份的所有地图集合mvpBackupMaps，获取每个地图的所有关键帧，并将它们存储在mpIdKFs中，
 * 其中键是关键帧的ID（mnId），值是对应的关键帧指针。最后返回这个包含所有关键帧的地图。
 */
map< long unsigned int, KeyFrame * > Atlas::GetAtlasKeyframes() {
    map< long unsigned int, KeyFrame * > mpIdKFs;
    for(Map *pMap_i : mvpBackupMaps) {
        vector< KeyFrame * > vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFrame *pKF_j_Mi : vpKFs_Mi) {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

}    // namespace ORB_SLAM3
