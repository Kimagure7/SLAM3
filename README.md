this repo is modified form [ORB_SLAM3](https://github.com/urbste/ORB_SLAM3) and [UMI_SLAM](https://github.com/cheng-chi/ORB_SLAM3)

below is all the differences between this repo and the original ORB_SLAM3 

# What UMI has done
1. Atlas: remove DoubleSphere and some mvpBackCam variables which are not used in the code
2. Converter: add transformMatrix function to convert cv::Mat to Eigen::Matrix, used in ReconstructionTwoViewsWithTag
3. Frame: Constructor for Monocular cameras.以及copy constructor 加入aruoco参数 构造函数加入aruco检测 检测结果存放在新加的三个变量里面markerIds, markerCorners, rejectedCandidates。
4. FrameDrawer:	mState为未初始化的时候同样追踪currentFrame 加入功能Draw detected markers in image
5. G2oTypes: 计算IMU偏差不再使用负单位矩阵 使用正单位矩阵
6. json_realer:	被chengchi删除, 转而使用json.h
7. G2oTypes：void EdgePriorAcc::linearizeOplus()使用正单位矩阵而非负单位矩阵，这一点像是ORB_SLAM3在合并代码的时候产生的错误，后面的提交被前面的提交覆盖了。总之现在G2oTypes与UMI_SLAM一致。
8. LocalMapping: 加入检测 if ((mpCurrentKeyFrame->mPrevKF != nullptr) && (mpCurrentKeyFrame->mPrevKF != nullptr))；keyframculling的时候加入pKF->mpImuPreintegrated检测。目的都是为了防止下面访问变量的时候报错。加大InitializeIMU处mScale阈值。
9. MLPnPsolover: 将assert改为异常 有着更多的提示
10. Optimizer:	添加对pKFi->mpImuPreintegrated==nullptr的判断，在FullInertialBA中；localInertialBA中添加预积分失效报错，移除check (mit->second<3)；InertialOptimization中添加预积分报错信息，预积分检查。总的来说这部分的改动目标是减少报错，很可能缺失ImuPreintegrated的就是重定位的那个关键帧。
11. System:	添加aruco支持。文件读写使用传入参数。
12. Tracking：添加aruco支持，修改默认recently lost阈值；mbLoadedMap判断，用于重定位而非重置地图；添加mpFrameDrawer->Update(this);INIT_RELOCALIZE状态处理（in track())。非只追踪添加recently lost处理；添加大量的logging（状态保存 用于恢复）1950行附近；track最后保存的时候修改了一点点；monocularInitialization中修改阈值，报错提示；ReconstructWithTwoViews修改；tracklocalmap修改mbOnlyTracking条件判断，条件判断mCurrentFrame.mpPrevFrame->mpcpi，防止优化报错。
13. TwoViewReconstruction：	添加ReconstructWithTag
14. KannalaBrandt8： 对应ReconstructWithTag功能，即使用aruco进行初始化

# What I modified or add
1. 移除大量的无用代码及变量
2. 校准RealSense d435i并提供了两个用于从bag文件中提取视频和imu数据的脚本，在example/data下
3. 修改了读取和保存文件的方式，防止内存溢出导致文件名被修改
4. Example添加，主要文件有：
   1. rsd435i_offline， 在视频上运行SLAM，测试用，过时。
   2. rsd435i_rt, 在实时流上运行，使用RGB视频流作为输入。少了时间戳归0功能，其他与rsd435i_rgbd一致。
   3. rsd435i_rgbd, 在实时流上运行，使用RGBD视频流作为输入，其中D暂时提供给校准端口，SLAM本身并不使用。载入地图后为了保证时间戳较为连续，需要保证时间戳都是从0开始的。
5. 实时轨迹发送，主要完成RealTimeTrajectory类，包含标定和遥操作发送功能，对应主函数入口中的参数tPort,tIP,cPort,cIP。t = teleportation，c = calibration。都是选择性启用，但是不允许只启用c不使用t。还包含发送内参功能。
6. 一些匹配阈值修改，希望能增强稳定性，但是实际效果不明显。
7. 大量的函数注释。
8. 由于我的docker中包含多个版本opencv，我在cmakelist中指定了搜索opencv库的路径，如果使用的不是我的docker请修改。

# docker
保存在rykj4090上，有一个运行中的实例，可以直接使用。images是
rsslam3:0.92,ID: d86d94fe88fb
由于仍在开发阶段，docker中仍未删除多余内容，包含了chengchi自身的slam代码(/ORB_SLAM3)，为了便于修改，源代码使用挂载的方式加载进容器。

docker运行命令如下：
```shell
docker run -td \
# --privileged=true \
--network host \
 -e NVIDIA_VISIBLE_DEVICES=all \
 -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e DISPLAY  \
 --env="QT_X11_NO_MITSHM=1" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="/dev:/dev" \
 --device-cgroup-rule "c 81:* rmw" \
 --device-cgroup-rule "c 189:* rmw" \
 -v /home/zhaozy/ZoeyChiu/umi/:/umi/ \
 --gpus all \
rsslam3:0.92 bash
```
其中/home/zhaozy/ZoeyChiu/umi/是我的本地路径，/umi/是docker中的路径，可以根据自己的情况修改。
其他挂载的卷用来读取realsense相机，后面更换相机可以更改；-e DISPLAY用来显示图像，可以进去后export；gpu是选用，可以不用；--network host方便配置网络，我还设置了ssh端口，10009。我使用跳板机连接，也可以试试直接连接，我不太行。