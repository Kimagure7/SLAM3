#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include "librealsense2/rsutil.h"
#include <CLI11.hpp>
#include <RealTimeTrajectory.h>
#include <System.h>
#include <json.h>
#include <librealsense2/rs.hpp>
using namespace std;
using nlohmann::json;

// 本代码中 深度信息只提供给尺度标定使用 slam本身并不使用


void signal_callback_handler(int signum) {
    std::cout << "gopro_slam.cc Caught signal " << signum << endl;
    // Terminate program
    exit(signum);
}

rs2_stream find_stream_to_align(const std::vector< rs2::stream_profile > &streams);
bool profile_changed(const std::vector< rs2::stream_profile > &current, const std::vector< rs2::stream_profile > &prev);

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

int main(int argc, char **argv) {

    signal(SIGINT, signal_callback_handler);
    // CLI parsing
    CLI::App app{ "D435i SLAM" };
    std::string vocabulary = "../../Vocabulary/ORBvoc.txt";
    app.add_option("-v,--vocabulary", vocabulary)->capture_default_str();

    std::string setting = "../yaml/radtand435i.yaml";
    app.add_option("-s,--setting", setting)->capture_default_str();

    std::string output_trajectory_tum;
    app.add_option("--output_trajectory_tum", output_trajectory_tum);

    std::string output_trajectory_csv;
    app.add_option("-o,--output_trajectory_csv", output_trajectory_csv);

    std::string load_map;
    app.add_option("-l,--load_map", load_map);

    std::string save_map;
    app.add_option("--save_map", save_map);

    int num_threads = 4;
    app.add_option("-n,--num_threads", num_threads);

    // Aruco tag for initialization
    int aruco_dict_id = cv::aruco::DICT_4X4_50;
    app.add_option("--aruco_dict_id", aruco_dict_id);

    int init_tag_id = 13;
    app.add_option("--init_tag_id", init_tag_id);

    float init_tag_size = 0.16;    // in meters
    app.add_option("--init_tag_size", init_tag_size);

    string tIP = "127.0.0.1";
    app.add_option("--tIP", tIP);

    int tPort = 0;
    app.add_option("--tPort", tPort);

    string cIP = "127.0.0.1";
    app.add_option("--cIP", cIP);

    int cPort = 0;
    app.add_option("--cPort", cPort);


    // if lost more than max_lost_frames, terminate
    // disable the check if <= 0
    // int max_lost_frames = -1;
    // app.add_option("--max_lost_frames", max_lost_frames);

    try {
        app.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        return app.exit(e);
    }

    std::cout << "num_threads: " << num_threads << endl;
    cv::setNumThreads(num_threads);


    std::cout << "setting file: " << setting
              << "\nvocabulary file: " << vocabulary
              << "\noutput_trajectory_tum: " << output_trajectory_tum
              << "\noutput_trajectory_csv: " << output_trajectory_csv
              << endl;

    double offset = 0;    // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if(devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    } else
        selected_device = devices[0];

    std::vector< rs2::sensor > sensors = selected_device.query_sensors();
    int index                          = 0;
    // We can now iterate the sensors and print their names
    for(rs2::sensor sensor : sensors)
        if(sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if(index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);    // switch off emitter
            }
            std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            if(index == 2) {
                // RGB camera
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }

            if(index == 3) {
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
            }
        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector< double > v_accel_timestamp;
    vector< rs2_vector > v_accel_data;
    vector< double > v_gyro_timestamp;
    vector< rs2_vector > v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector< double > v_accel_timestamp_sync;
    vector< rs2_vector > v_accel_data_sync;

    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready       = false;
    int count_im_buffer    = 0;    // count dropped frames

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    pipe.stop();

    // Align depth and RGB frames
    // Pipeline could choose a device that does not have a color stream
    // If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    // The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
    rs2::frameset fsSLAM;

    auto imu_callback = [&](const rs2::frame &frame) {
        std::unique_lock< std::mutex > lock(imu_mutex);

        if(rs2::frameset fs = frame.as< rs2::frameset >()) {
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp() * 1e-3;
            if(abs(timestamp_image - new_timestamp_image) < 0.001) {
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            if(profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams())) {
                // If the profile was changed, update the align object, and also get the new device's depth scale
                pipe_profile = pipe.get_active_profile();
                align_to     = find_stream_to_align(pipe_profile.get_streams());
                align        = rs2::align(align_to);
            }

            // Align depth and rgb takes long time, move it out of the interruption to avoid losing IMU measurements
            fsSLAM = fs;

            timestamp_image = fs.get_timestamp() * 1e-3;
            image_ready     = true;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
                int index          = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();

        } else if(rs2::motion_frame m_frame = frame.as< rs2::motion_frame >()) {
            if(m_frame.get_profile().stream_name() == "Gyro") {
                // It runs at 200Hz
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
            } else if(m_frame.get_profile().stream_name() == "Accel") {
                // It runs at 60Hz
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data      = current_accel_data;

                current_accel_data      = m_frame.get_motion_data();
                current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

                while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
                    int index          = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];    // 其实是下一个gyro的时间

                    rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp,
                                                                prev_accel_data, prev_accel_timestamp);

                    v_accel_data_sync.push_back(interp_data);
                    v_accel_timestamp_sync.push_back(target_time);
                }
            }
        }
    };


    pipe_profile = pipe.start(cfg, imu_callback);
    vector< ORB_SLAM3::IMU::Point > vImuMeas;
    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    float *Rbc                     = cam_stream.get_extrinsics_to(imu_stream).rotation;
    float *tbc                     = cam_stream.get_extrinsics_to(imu_stream).translation;
    std::cout << "Tbc = " << std::endl;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++)
            std::cout << Rbc[i * 3 + j] << ", ";
        std::cout << tbc[i] << "\n";
    }

    rs2_intrinsics intrinsics_cam = cam_stream.as< rs2::video_stream_profile >().get_intrinsics();
    width_img                     = intrinsics_cam.width;
    height_img                    = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " << intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // not used DEPTH here, depth just for calibration

    double timestamp;
    cv::Mat im, depth;

    cv::Ptr< cv::aruco::Dictionary > aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);
    ORB_SLAM3::System SLAM(
        vocabulary, setting,
        ORB_SLAM3::System::IMU_MONOCULAR,
        true, load_map, save_map,
        aruco_dict, init_tag_id, init_tag_size);

    float imageScale = SLAM.GetImageScale();

    std::thread *pRtTraj;
    RealTimeTrajectory *mpRealTimeTrajectory;
    if(tPort) {
        mpRealTimeTrajectory = new RealTimeTrajectory(tPort, tIP,cPort,cIP,30, output_trajectory_csv);
        pRtTraj              = new std::thread(&RealTimeTrajectory::Run, mpRealTimeTrajectory);
    }


    // Clear IMU vectors
    v_gyro_data.clear();
    v_gyro_timestamp.clear();
    v_accel_data_sync.clear();
    v_accel_timestamp_sync.clear();

    while(!SLAM.isShutDown()) {
        std::vector< rs2_vector > vGyro;
        std::vector< double > vGyro_times;
        std::vector< rs2_vector > vAccel;
        std::vector< double > vAccel_times;
        bool needDepth = mpRealTimeTrajectory->CheckState();
        rs2::frameset fs;
        {
            std::unique_lock< std::mutex > lk(imu_mutex);
            if(!image_ready)
                cond_image_rec.wait(lk);

            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();

            fs = fsSLAM;
            if(count_im_buffer > 1)    // 处理不过来 丢帧咯
                std::cout << count_im_buffer - 1 << " dropped frs\n";
            count_im_buffer = 0;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {    // 不断同步
                int index          = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp);

                v_accel_data_sync.push_back(interp_data);
                // v_accel_data_sync.push_back(current_accel_data); // 0 interpolation
                v_accel_timestamp_sync.push_back(target_time);
            }

            // Copy the IMU data
            vGyro        = v_gyro_data;
            vGyro_times  = v_gyro_timestamp;
            vAccel       = v_accel_data_sync;
            vAccel_times = v_accel_timestamp_sync;

            timestamp = timestamp_image;

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;
        }

        if(needDepth) {
            // Perform alignment here
            auto processed = align.process(fs);

            // Trying to get both other and aligned depth frames
            rs2::video_frame color_frame = processed.first(align_to);
            rs2::depth_frame depth_frame = processed.get_depth_frame();

            im    = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void *)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
        } else {
            rs2::video_frame color_frame = fs.get_color_frame();
            im                           = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        }


        for(int i = 0; i < vGyro.size(); ++i) {
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                            vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                            vGyro_times[i]);
            vImuMeas.push_back(lastPoint);
        }

        if(imageScale != 1.f) {
            int width  = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            if(needDepth) {
                cv::resize(depth, depth, cv::Size(width, height));
            }
        }

        // Pass the image to the SLAM system
        auto result = SLAM.LocalizeMonocular(im, timestamp, vImuMeas);
        if(tPort) {
            if(needDepth) {
                RealTimeTrajectory::TcwData data(result.second, result.first, im, depth);
                mpRealTimeTrajectory->AddTcw(data);
            } else {
                RealTimeTrajectory::TcwData data(result.second, result.first);
                mpRealTimeTrajectory->AddTcw(data);
            }
        }
        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
    }
    std::cout << "System shutdown!\n";
    if(tPort) {
        mpRealTimeTrajectory->RequestFinish();
    }
    if(load_map.empty() && !save_map.empty()) {
        SLAM.SaveAtlas(ORB_SLAM3::System::FileType::BINARY_FILE, save_map, vocabulary);
    }
    // Save camera trajectory
    // if(!output_trajectory_tum.empty()) {
    //     SLAM.SaveTrajectoryTUM(output_trajectory_tum);
    // }

    // if(!output_trajectory_csv.empty()) {
    //     SLAM.SaveTrajectoryCSV(output_trajectory_csv);
    // }
    return 0;
}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time) {

    // If there are not previous information, the current data is propagated
    if(prev_time == 0) {
        return current_data;
    }

    rs2_vector increment;
    rs2_vector value_interp;

    if(target_time > current_time) {
        value_interp = current_data;
    } else if(target_time > prev_time) {
        increment.x = current_data.x - prev_data.x;
        increment.y = current_data.y - prev_data.y;
        increment.z = current_data.z - prev_data.z;

        double factor = (target_time - prev_time) / (current_time - prev_time);

        value_interp.x = prev_data.x + increment.x * factor;
        value_interp.y = prev_data.y + increment.y * factor;
        value_interp.z = prev_data.z + increment.z * factor;

        // zero interpolation
        value_interp = current_data;
    } else {
        value_interp = prev_data;
    }

    return value_interp;
}
rs2_stream find_stream_to_align(const std::vector< rs2::stream_profile > &streams) {
    // Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    // We prioritize color streams to make the view look better.
    // If color is not available, we take another stream that (other than depth)
    rs2_stream align_to     = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for(rs2::stream_profile sp : streams) {
        rs2_stream profile_stream = sp.stream_type();
        if(profile_stream != RS2_STREAM_DEPTH) {
            if(!color_stream_found)    // Prefer color
                align_to = profile_stream;

            if(profile_stream == RS2_STREAM_COLOR) {
                color_stream_found = true;
            }
        } else {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if(align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


bool profile_changed(const std::vector< rs2::stream_profile > &current, const std::vector< rs2::stream_profile > &prev) {
    for(auto &&sp : prev) {
        // If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile &current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if(itr == std::end(current))    // If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}