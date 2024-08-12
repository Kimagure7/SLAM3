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


#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace ORB_SLAM3 {

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc)
    : mpVoc(&voc) {
    mvInvertedFile.resize(voc.size());
}


void KeyFrameDatabase::add(KeyFrame *pKF) {
    unique_lock< mutex > lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame *pKF) {
    unique_lock< mutex > lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
        // List of keyframes that share the word
        list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

        for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
            if(pKF == *lit) {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear() {
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map *pMap) {
    unique_lock< mutex > lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(std::vector< list< KeyFrame * > >::iterator vit = mvInvertedFile.begin(), vend = mvInvertedFile.end(); vit != vend; vit++) {
        // List of keyframes that share the word
        list< KeyFrame * > &lKFs = *vit;

        for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend;) {
            KeyFrame *pKFi = *lit;
            if(pMap == pKFi->GetMap()) {
                lit = lKFs.erase(lit);
                // Dont delete the KF because the class Map clean all the KF when it is destroyed
            } else {
                ++lit;
            }
        }
    }
}

vector< KeyFrame * > KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, float minScore) {
    set< KeyFrame * > spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list< KeyFrame * > lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock< mutex > lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
            list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

            for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                KeyFrame *pKFi = *lit;
                if(pKFi->GetMap() == pKF->GetMap())    // For consider a loop candidate it a candidate it must be in the same map
                {
                    if(pKFi->mnLoopQuery != pKF->mnId) {
                        pKFi->mnLoopWords = 0;
                        if(!spConnectedKeyFrames.count(pKFi)) {
                            pKFi->mnLoopQuery = pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                }
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector< KeyFrame * >();

    list< pair< float, KeyFrame * > > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        if((*lit)->mnLoopWords > maxCommonWords)
            maxCommonWords = (*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;

        if(pKFi->mnLoopWords > minCommonWords) {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si >= minScore)
                lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector< KeyFrame * >();

    list< pair< float, KeyFrame * > > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
        KeyFrame *pKFi                = it->second;
        vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore   = it->first;
        float accScore    = it->first;
        KeyFrame *pBestKF = pKFi;
        for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
            KeyFrame *pKF2 = *vit;
            if(pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords) {
                accScore += pKF2->mLoopScore;
                if(pKF2->mLoopScore > bestScore) {
                    pBestKF   = pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if(accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;

    set< KeyFrame * > spAlreadyAddedKF;
    vector< KeyFrame * > vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
        if(it->first > minScoreToRetain) {
            KeyFrame *pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi)) {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

/**
 * @brief 检测KeyFrame的候选者用于循环闭合和地图合并
 *
 * 该函数搜索与给定KeyFrame共享视觉词汇的其他KeyFrames，作为潜在的循环闭合或地图合并候选者。
 * 它首先确定所有共享词汇的KeyFrames，然后计算它们之间的相似度得分。
 * 得分通过共视图邻居累积，并返回满足最小得分要求的所有候选者列表。
 *
 * @param pKF 给定的KeyFrame，用于查找其候选者
 * @param minScore 最小相似度得分阈值，用于过滤候选者
 * @param vpLoopCand 输出参数，存储为潜在循环闭合候选者的KeyFrames列表
 * @param vpMergeCand 输出参数，存储为潜在地图合并候选者的KeyFrames列表
 */
void KeyFrameDatabase::DetectCandidates(KeyFrame *pKF, float minScore, vector< KeyFrame * > &vpLoopCand, vector< KeyFrame * > &vpMergeCand) {
    set< KeyFrame * > spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list< KeyFrame * > lKFsSharingWordsLoop, lKFsSharingWordsMerge;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock< mutex > lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
            list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

            for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                KeyFrame *pKFi = *lit;
                if(pKFi->GetMap() == pKF->GetMap())    // For consider a loop candidate it a candidate it must be in the same map
                {
                    if(pKFi->mnLoopQuery != pKF->mnId) {
                        pKFi->mnLoopWords = 0;
                        if(!spConnectedKeyFrames.count(pKFi)) {
                            pKFi->mnLoopQuery = pKF->mnId;
                            lKFsSharingWordsLoop.push_back(pKFi);
                        }
                    }
                    pKFi->mnLoopWords++;
                } else if(!pKFi->GetMap()->IsBad()) {
                    if(pKFi->mnMergeQuery != pKF->mnId) {
                        pKFi->mnMergeWords = 0;
                        if(!spConnectedKeyFrames.count(pKFi)) {
                            pKFi->mnMergeQuery = pKF->mnId;
                            lKFsSharingWordsMerge.push_back(pKFi);
                        }
                    }
                    pKFi->mnMergeWords++;
                }
            }
        }
    }

    if(lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
        return;

    if(!lKFsSharingWordsLoop.empty()) {
        list< pair< float, KeyFrame * > > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for(list< KeyFrame * >::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++) {
            if((*lit)->mnLoopWords > maxCommonWords)
                maxCommonWords = (*lit)->mnLoopWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list< KeyFrame * >::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++) {
            KeyFrame *pKFi = *lit;

            if(pKFi->mnLoopWords > minCommonWords) {
                nscores++;

                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                if(si >= minScore)
                    lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if(!lScoreAndMatch.empty()) {
            list< pair< float, KeyFrame * > > lAccScoreAndMatch;
            float bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
                KeyFrame *pKFi                = it->second;
                vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore   = it->first;
                float accScore    = it->first;
                KeyFrame *pBestKF = pKFi;
                for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
                    KeyFrame *pKF2 = *vit;
                    if(pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords) {
                        accScore += pKF2->mLoopScore;
                        if(pKF2->mLoopScore > bestScore) {
                            pBestKF   = pKF2;
                            bestScore = pKF2->mLoopScore;
                        }
                    }
                }

                lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                if(accScore > bestAccScore)
                    bestAccScore = accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccScore;

            set< KeyFrame * > spAlreadyAddedKF;
            vpLoopCand.reserve(lAccScoreAndMatch.size());

            for(list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
                if(it->first > minScoreToRetain) {
                    KeyFrame *pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi)) {
                        vpLoopCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }
    }

    if(!lKFsSharingWordsMerge.empty()) {
        list< pair< float, KeyFrame * > > lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for(list< KeyFrame * >::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++) {
            if((*lit)->mnMergeWords > maxCommonWords)
                maxCommonWords = (*lit)->mnMergeWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list< KeyFrame * >::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++) {
            KeyFrame *pKFi = *lit;

            if(pKFi->mnMergeWords > minCommonWords) {
                nscores++;

                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mMergeScore = si;
                if(si >= minScore)
                    lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if(!lScoreAndMatch.empty()) {
            list< pair< float, KeyFrame * > > lAccScoreAndMatch;
            float bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
                KeyFrame *pKFi                = it->second;
                vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                float bestScore   = it->first;
                float accScore    = it->first;
                KeyFrame *pBestKF = pKFi;
                for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
                    KeyFrame *pKF2 = *vit;
                    if(pKF2->mnMergeQuery == pKF->mnId && pKF2->mnMergeWords > minCommonWords) {
                        accScore += pKF2->mMergeScore;
                        if(pKF2->mMergeScore > bestScore) {
                            pBestKF   = pKF2;
                            bestScore = pKF2->mMergeScore;
                        }
                    }
                }

                lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                if(accScore > bestAccScore)
                    bestAccScore = accScore;
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccScore;

            set< KeyFrame * > spAlreadyAddedKF;
            vpMergeCand.reserve(lAccScoreAndMatch.size());

            for(list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
                if(it->first > minScoreToRetain) {
                    KeyFrame *pKFi = it->second;
                    if(!spAlreadyAddedKF.count(pKFi)) {
                        vpMergeCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }
    }

    for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
        list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

        for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
            KeyFrame *pKFi     = *lit;
            pKFi->mnLoopQuery  = -1;
            pKFi->mnMergeQuery = -1;
        }
    }
}

/**
 * @brief 检测最佳的循环闭合和地图合并候选者
 *
 * 该函数识别与给定KeyFrame共享足够多视觉词汇的最佳循环闭合和地图合并候选者。
 * 它搜索所有相关的KeyFrames，并通过累积共视图邻居的得分来评估每个候选人。
 *
 * @param pKF 给定的Keyframe对象
 * @param vpLoopCand 输出参数，用于存储可能的循环闭合候选人列表
 * @param vpMergeCand 输出参数，用于存储可能的地图合并候选人列表
 * @param nMinWords 最小共享单词数阈值，低于此阈值将不考虑为候选人
 */
void KeyFrameDatabase::DetectBestCandidates(KeyFrame *pKF, vector< KeyFrame * > &vpLoopCand, vector< KeyFrame * > &vpMergeCand, int nMinWords) {
    list< KeyFrame * > lKFsSharingWords;
    set< KeyFrame * > spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock< mutex > lock(mMutex);

        spConnectedKF = pKF->GetConnectedKeyFrames();

        for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
            list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

            for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                KeyFrame *pKFi = *lit;
                if(spConnectedKF.find(pKFi) != spConnectedKF.end()) {
                    continue;
                }
                if(pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
                    pKFi->mnPlaceRecognitionWords = 0;
                    pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnPlaceRecognitionWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        if((*lit)->mnPlaceRecognitionWords > maxCommonWords)
            maxCommonWords = (*lit)->mnPlaceRecognitionWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    if(minCommonWords < nMinWords) {
        minCommonWords = nMinWords;
    }

    list< pair< float, KeyFrame * > > lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;

        if(pKFi->mnPlaceRecognitionWords > minCommonWords) {
            nscores++;
            float si                     = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
            pKFi->mPlaceRecognitionScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return;

    list< pair< float, KeyFrame * > > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
        KeyFrame *pKFi                = it->second;
        vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore   = it->first;
        float accScore    = bestScore;
        KeyFrame *pBestKF = pKFi;
        for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
            KeyFrame *pKF2 = *vit;
            if(pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                continue;

            accScore += pKF2->mPlaceRecognitionScore;
            if(pKF2->mPlaceRecognitionScore > bestScore) {
                pBestKF   = pKF2;
                bestScore = pKF2->mPlaceRecognitionScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if(accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    set< KeyFrame * > spAlreadyAddedKF;
    vpLoopCand.reserve(lAccScoreAndMatch.size());
    vpMergeCand.reserve(lAccScoreAndMatch.size());
    for(list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
        const float &si = it->first;
        if(si > minScoreToRetain) {
            KeyFrame *pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi)) {
                if(pKF->GetMap() == pKFi->GetMap()) {
                    vpLoopCand.push_back(pKFi);
                } else {
                    vpMergeCand.push_back(pKFi);
                }
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
}

bool compFirst(const pair< float, KeyFrame * > &a, const pair< float, KeyFrame * > &b) {
    return a.first > b.first;
}

/**
 * @brief 检测最佳的循环和合并候选KeyFrames
 *
 * 该函数用于识别与给定KeyFrame共享视觉词汇的最佳候选者，用于潜在的循环闭合或地图合并。
 * 它首先找到所有共享词汇的KeyFrames，然后计算它们之间的相似度得分。
 * 得分通过共视图邻居累积，并返回前nNumCandidates个候选者，分为可能的循环闭合和地图合并两类。
 *
 * @param pKF 给定的KeyFrame，用于查找其候选者
 * @param vpLoopCand 输出参数，用于存储作为潜在循环闭合候选者的KeyFrames
 * @param vpMergeCand 输出参数，用于存储作为潜在地图合并候选者的KeyFrames
 * @param nNumCandidates 最大数量的候选人要检测（对于每个类别）
 */
void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector< KeyFrame * > &vpLoopCand, vector< KeyFrame * > &vpMergeCand, int nNumCandidates) {
    list< KeyFrame * > lKFsSharingWords;
    set< KeyFrame * > spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock< mutex > lock(mMutex);

        spConnectedKF = pKF->GetConnectedKeyFrames();

        for(DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++) {
            list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

            for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                KeyFrame *pKFi = *lit;

                if(pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
                    pKFi->mnPlaceRecognitionWords = 0;
                    if(!spConnectedKF.count(pKFi)) {

                        pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnPlaceRecognitionWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        if((*lit)->mnPlaceRecognitionWords > maxCommonWords)
            maxCommonWords = (*lit)->mnPlaceRecognitionWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    list< pair< float, KeyFrame * > > lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;

        if(pKFi->mnPlaceRecognitionWords > minCommonWords) {
            nscores++;
            float si                     = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
            pKFi->mPlaceRecognitionScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return;

    list< pair< float, KeyFrame * > > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
        KeyFrame *pKFi                = it->second;
        vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore   = it->first;
        float accScore    = bestScore;
        KeyFrame *pBestKF = pKFi;
        for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
            KeyFrame *pKF2 = *vit;
            if(pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                continue;

            accScore += pKF2->mPlaceRecognitionScore;
            if(pKF2->mPlaceRecognitionScore > bestScore) {
                pBestKF   = pKF2;
                bestScore = pKF2->mPlaceRecognitionScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if(accScore > bestAccScore)
            bestAccScore = accScore;
    }

    lAccScoreAndMatch.sort(compFirst);

    vpLoopCand.reserve(nNumCandidates);
    vpMergeCand.reserve(nNumCandidates);
    set< KeyFrame * > spAlreadyAddedKF;
    int i                                          = 0;
    list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin();
    while(i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates)) {
        KeyFrame *pKFi = it->second;
        if(pKFi->isBad())
            continue;

        if(!spAlreadyAddedKF.count(pKFi)) {
            if(pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates) {
                vpLoopCand.push_back(pKFi);
            } else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad()) {
                vpMergeCand.push_back(pKFi);
            }
            spAlreadyAddedKF.insert(pKFi);
        }
        i++;
        it++;
    }
}

/**
 * @brief 从给定的Frame和Map中检测重定位候选KeyFrame
 *
 * 此函数搜索与当前帧共享视觉词汇的KeyFrames，然后计算它们之间的相似度得分。
 * 它只考虑那些共享足够多词汇的KeyFrames，并通过累积共视图邻居的得分来增强结果。
 * 最后，返回所有得分高于0.75倍最高累积得分的、属于指定Map的KeyFrames作为重定位候选。
 *
 * @param F 当前帧，用于比较和查找共享词汇
 * @param pMap 指定的地图，用于过滤属于该地图的KeyFrames
 *
 * @return vector<KeyFrame*> 一组被选为重定位候选者的KeyFrames
 */
vector< KeyFrame * > KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map *pMap) {
    list< KeyFrame * > lKFsSharingWords;

    // Step1: Search all keyframes that share a word with current frame
    {
        unique_lock< mutex > lock(mMutex);
        // iterate through all words in the current frame
        for(DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++) {
            // lKFs 包含了第vit->first个单词的所有KeyFrames
            // mvInvertedFile是一个vector<list<KeyFram*>> ，存储了一个单词对应的所有KeyFrames
            list< KeyFrame * > &lKFs = mvInvertedFile[vit->first];

            for(list< KeyFrame * >::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
                KeyFrame *pKFi = *lit;
                // 如果pKFi还没有被标记 则初始化一些重定位用的属性
                // 注意 两帧之间可能有多个共享单词 可能之前就标记过了
                if(pKFi->mnRelocQuery != F->mnId) {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                // 该帧与当前帧共享的单词数目
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector< KeyFrame * >();

    // Step2: Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        if((*lit)->mnRelocWords > maxCommonWords)
            maxCommonWords = (*lit)->mnRelocWords;
    }
    // 原装阈值是0.8 这里适当调低以增强relocalization的鲁棒性
    int minCommonWords = maxCommonWords * 0.6f;

    list< pair< float, KeyFrame * > > lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    // 有共同词汇的关键帧只有高过一定词汇阈值才有分数
    for(list< KeyFrame * >::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;

        if(pKFi->mnRelocWords > minCommonWords) {
            nscores++;
            float si          = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector< KeyFrame * >();

    list< pair< float, KeyFrame * > > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Step3: 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
    // 从有分数的帧出发找10个共视 这里面只要有共同词汇的帧 然后计算累计分数
    // 每一组找一个最高分的帧
    for(list< pair< float, KeyFrame * > >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
        KeyFrame *pKFi                = it->second;
        vector< KeyFrame * > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);    // 每个有共同词汇的关键帧基础上再找10个共视最多的的关键帧

        float bestScore   = it->first;
        float accScore    = bestScore;
        KeyFrame *pBestKF = pKFi;
        for(vector< KeyFrame * >::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
            KeyFrame *pKF2 = *vit;
            if(pKF2->mnRelocQuery != F->mnId) // 这个新招的关键帧也得是与当前帧有共同词汇
                continue;

            accScore += pKF2->mRelocScore; // 也就是说在这个新的关键帧还得有分数 不然0相当于跳过了
            if(pKF2->mRelocScore > bestScore) {
                pBestKF   = pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if(accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 修改到0.65以尝试增强鲁棒性
    float minScoreToRetain = 0.65f * bestAccScore;
    set< KeyFrame * > spAlreadyAddedKF;
    vector< KeyFrame * > vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list< pair< float, KeyFrame * > >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
        const float &si = it->first;
        if(si > minScoreToRetain) {
            KeyFrame *pKFi = it->second;
            if(pKFi->GetMap() != pMap)
                continue;
            if(!spAlreadyAddedKF.count(pKFi)) {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc) {
    ORBVocabulary **ptr;
    ptr  = (ORBVocabulary **)(&mpVoc);
    *ptr = pORBVoc;

    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

}    // namespace ORB_SLAM3
