/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>

#include "ORBVocabulary.h"
#include <System.h>
#include "Converter.h"
#include "ImuTypes.h"
#include "Optimizer.h"

#include "jsonl_reader.h"
#include "imu_sync.h"

#include "json.h"

//#include "system_ext.hpp"

using namespace std;

bool fileExists(const std::string filename) {
  struct stat statInfo;
  return (stat(filename.c_str(), &statInfo) == 0);
}

std::string findVideoSuffix(std::string videoPathNoSuffix) {
  for (std::string suffix : { "mov", "avi", "mp4" }) {
    const auto path = videoPathNoSuffix + "." + suffix;
    std::ifstream testFile(videoPathNoSuffix + "." + suffix);
    if (testFile.is_open()) return path;
  }
  return "";
}

inline bool ends_with(std::string const &value, std::string const &ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

// Adapted from stereo_inertial_euroc.cc example from ORBSLAM3 Examples
int main(int argc, char **argv)
{
    if(argc != 5) {
        cerr << endl << "Usage: ./main path_to_vocabulary_folder path_to_settings path_to_dataset_folder output_path" << endl;
        return 1;
    }

    std::string vocabFolder = argv[1];
    std::string settingsFile = argv[2];
    std::string datasetFolder = argv[3];
    std::string outputFile = argv[4];

    assert(ends_with(outputFile, ".jsonl") && "output must end with .jsonl");

    std::string vocabBinPath = vocabFolder + "/ORBvoc.bin";
    if (!fileExists(vocabBinPath)) {
        // On first run, convert ORBvoc to a binary file for faster loading in future runs
        ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();
        voc->loadFromTextFile(vocabFolder + "/ORBvoc.txt");
        voc->saveToBinaryFile(vocabBinPath);
    }

    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName) {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    JsonlReader reader;

    reader.onFrames = [&vTimestampsCam](std::vector<JsonlReader::FrameParameters> frames) {
      assert(frames.size() > 0);
      vTimestampsCam.push_back(frames[0].time);
    };
    reader.read(datasetFolder + "/data.jsonl");

    nImages = vTimestampsCam.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // SystemExt is an extended version of standard ORBSLAM3 System with support to export final trajectory
    ORB_SLAM3::System SLAM(
        vocabBinPath,
        settingsFile,
        ORB_SLAM3::System::STEREO,
        true // Viewer? Was true
    );

    std::vector<cv::VideoCapture> videoCaptures;
    std::string leftCamPath = findVideoSuffix(datasetFolder + "/data");
    std::string rightCamPath = findVideoSuffix(datasetFolder + "/data2");
    assert(!leftCamPath.empty() && "Video file missing");
    videoCaptures.push_back(cv::VideoCapture(leftCamPath));
    bool stereo = !rightCamPath.empty();
    std::cout << "Stereo: " << stereo << std::endl;
    if (stereo) videoCaptures.push_back(cv::VideoCapture(rightCamPath));

    cv::Mat imLeft, imRight;

    double t_rect = 0;
    cv::Mat tm;
    for(int ni = 0; ni < nImages; ni++) {
        // Read left and right images from file
        videoCaptures[0].read(imLeft);
        if (stereo) videoCaptures[1].read(imRight);

        if(imLeft.empty()) {
            cerr << endl << "Failed to load left frame" << endl;
            return 1;
        }

        if(stereo && imRight.empty()) {
            cerr << endl << "Failed to load right frame" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t_Start_Rect = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_Start_Rect = std::chrono::monotonic_clock::now();
#endif

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t_End_Rect = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_End_Rect = std::chrono::monotonic_clock::now();
#endif

        t_rect = std::chrono::duration_cast<std::chrono::duration<double> >(t_End_Rect - t_Start_Rect).count();
        double tframe = vTimestampsCam[ni];


#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        if (stereo)
            SLAM.TrackStereo(imLeft, imRight, tframe);
        else // TODO: monocular is untested
            SLAM.TrackMonocular(imLeft, tframe);


#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // TODO: Is this really necessary?
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
