// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <cctype>
#include <stdexcept>
#include <unistd.h>
#include <sys/stat.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "map_storage.hpp"
#include "startup_relocalization.hpp"
#include "workspace_runtime.hpp"
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

/*** Localization (reference-map) mode: the ikd-Tree is pre-loaded from a PCD
 *** map built once per environment, no points are ever added/deleted, and the
 *** EKF state starts from a given initial pose. All runs therefore share the
 *** reference map's world frame, which is what makes cross-run trajectories
 *** (manual / VLN / baseline) directly comparable. ***/
bool localization_mode = false;
string loc_map_path = "";
double loc_map_voxel_size = 0.1;
vector<double> loc_initial_pose; // x, y, z, yaw(rad) of the start pose in map frame

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;
fastlio_runtime::HealthOptions tracking_options;
std::size_t max_lidar_queue=30,max_imu_queue=2000;
std::atomic<bool> input_discontinuity{false};

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

/* Keep user-provided metadata safe to embed in a CSV file name. */
static std::string sanitize_for_filename(const std::string &in)
{
    std::string out;
    for (size_t i = 0; i < in.size(); i++)
    {
        char c = in[i];
        if (std::isalnum((unsigned char)c) || c == '-' || c == '_') out += c;
        else out += '_';
    }
    return out;
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void clear_sensor_queues_unlocked() {
    lidar_buffer.clear(); time_buffer.clear(); imu_buffer.clear(); lidar_pushed=false;
    input_discontinuity.store(true);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        clear_sensor_queues_unlocked();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    if(lidar_buffer.size()>=max_lidar_queue) clear_sensor_queues_unlocked();
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    if(runtime_pos_log && scan_count<MAXN) s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        clear_sensor_queues_unlocked();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    if(lidar_buffer.size()>=max_lidar_queue) clear_sensor_queues_unlocked();
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    if(runtime_pos_log && scan_count<MAXN) s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        clear_sensor_queues_unlocked();
    }

    last_timestamp_imu = timestamp;

    if(imu_buffer.size()>=max_imu_queue) clear_sensor_queues_unlocked();
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    // IKFoM state order is position, SO(3), ...; its rotation error is a
    // right/body tangent. Pose covariance uses fixed axes in camera_init.
    Eigen::Matrix<double,6,6> tangent=Eigen::Matrix<double,6,6>::Identity();
    tangent.block<3,3>(3,3)=state_point.rot.toRotationMatrix();
    const auto filter_covariance=kf.get_P();
    const Eigen::Matrix<double,6,6> pose_covariance=tangent*filter_covariance.topLeftCorner<6,6>()*tangent.transpose();
    for(int i=0;i<6;++i) for(int j=0;j<6;++j) odomAftMapped.pose.covariance[i*6+j]=pose_covariance(i,j);
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath,std::size_t max_poses)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        if(path.poses.size()>max_poses) path.poses.erase(path.poses.begin());
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->resize(feats_down_size);
    corr_normvect->resize(feats_down_size);
    total_residual = 0.0; 
    res_mean_last = std::numeric_limits<double>::infinity();

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < tracking_options.min_points ||
        double(effct_feat_num)/std::max(1,feats_down_size)<tracking_options.min_ratio)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    if (!std::isfinite(res_mean_last) || res_mean_last>tracking_options.max_residual) {
        ekfom_data.valid=false; return;
    }
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("map_dir", string(ROOT_DIR) + "../../pcd_map");
        this->declare_parameter<string>("map_name", "map");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 400.);
        this->declare_parameter<float>("mapping.det_range", 100.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<bool>("localization.mode", false);
        this->declare_parameter<string>("localization.map_path", "");
        this->declare_parameter<double>("localization.map_voxel_size", 0.1);
        this->declare_parameter<vector<double>>("localization.initial_pose", vector<double>());
        declare_relocalization_parameters();
        this->declare_parameter<int>("record.sample_every_n", 1);
        this->declare_parameter<bool>("record.republish_saved", false);
        this->declare_parameter<string>("record.control_source", "");
        this->declare_parameter<string>("record.instruction_id", "");
        this->declare_parameter<string>("record.note", "");
        this->declare_parameter<bool>("preprocess.operator_filter_en", false);
        this->declare_parameter<double>("preprocess.operator_filter_rear_angle", 50.0);
        this->declare_parameter<double>("preprocess.operator_filter_range_min", 0.3);
        this->declare_parameter<double>("preprocess.operator_filter_range_max", 3.0);
        declare_runtime_parameters();

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,400.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,100.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<bool>("localization.mode", localization_mode, false);
        this->get_parameter_or<string>("localization.map_path", loc_map_path, string(""));
        this->get_parameter_or<double>("localization.map_voxel_size", loc_map_voxel_size, 0.1);
        this->get_parameter_or<vector<double>>("localization.initial_pose", loc_initial_pose, vector<double>());
        read_relocalization_parameters();
        this->get_parameter_or<int>("record.sample_every_n", record_sample_every_n_, 1);
        this->get_parameter_or<bool>("record.republish_saved", record_republish_saved_, false);
        this->get_parameter_or<bool>("preprocess.operator_filter_en", p_pre->operator_filter_en, false);
        this->get_parameter_or<double>("preprocess.operator_filter_rear_angle", p_pre->operator_filter_rear_angle, 50.0);
        this->get_parameter_or<double>("preprocess.operator_filter_range_min", p_pre->operator_filter_range_min, 0.3);
        this->get_parameter_or<double>("preprocess.operator_filter_range_max", p_pre->operator_filter_range_max, 3.0);
        if (record_sample_every_n_ < 1)
        {
            RCLCPP_WARN(this->get_logger(), "record.sample_every_n < 1 is invalid, reset to 1.");
            record_sample_every_n_ = 1;
        }

        initialize_runtime();
        parameter_callback_=this->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result; result.successful=true;
            for(const auto& p:parameters) {
                const auto name=p.get_name();
                const bool metadata=name=="record.control_source" || name=="record.instruction_id" || name=="record.note";
                const bool matched=name=="localization.matched_pose" || name=="localization.matched_rmse" || name=="localization.matched_overlap";
                if(metadata && p.get_type()!=rclcpp::ParameterType::PARAMETER_STRING) {
                    result.successful=false; result.reason="Recording labels must be strings"; break;
                }
                if(name=="record.sample_every_n" && (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER || p.as_int()<1)) {
                    result.successful=false; result.reason="record.sample_every_n must be a positive integer"; break;
                }
                if(!metadata && !matched && name!="record.sample_every_n") {
                    result.successful=false; result.reason="This parameter is startup-only; change YAML/launch arguments and restart"; break;
                }
            }
            return result;
        });
        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        if (!localization_mode && pcd_save_en)
        {
            map_file_path = fastlio_maps::prepare_output(
                this->get_parameter("map_dir").as_string(),
                this->get_parameter("map_name").as_string(), map_file_path).string();
            RCLCPP_INFO(this->get_logger(), "Mapping session output: %s", map_file_path.c_str());
            if (pcd_save_interval > 0)
                RCLCPP_WARN(this->get_logger(), "pcd_save.interval is not used: this workspace saves one complete map per session.");
        }

        if (localization_mode)
        {
            pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());
            if (loc_map_path.empty() ||
                pcl::io::loadPCDFile<PointType>(loc_map_path, *map_cloud) < 0 ||
                map_cloud->empty())
            {
                RCLCPP_ERROR(this->get_logger(),
                    "Localization mode: failed to load reference map from '%s'. "
                    "Refusing to start in SLAM mode to avoid overwriting/duplicating the reference frame.",
                    loc_map_path.c_str());
                throw std::runtime_error("localization map load failed: " + loc_map_path);
            }
            if (loc_map_voxel_size > 0)
            {
                pcl::VoxelGrid<PointType> map_voxel_filter;
                map_voxel_filter.setLeafSize((float)loc_map_voxel_size, (float)loc_map_voxel_size, (float)loc_map_voxel_size);
                map_voxel_filter.setInputCloud(map_cloud);
                map_voxel_filter.filter(*map_cloud);
            }
            ikdtree.set_downsample_param(filter_size_map_min);
            ikdtree.Build(map_cloud->points);
            if (relocalization_enabled_) {
                std::vector<fastlio_relocalization::Point> reference;
                reference.reserve(map_cloud->size());
                for (const auto& p : *map_cloud) {
                    const V3D level=map_to_level_*V3D(p.x,p.y,p.z);
                    reference.push_back({level.x(),level.y(),level.z()});
                }
                const auto center=relocalization_options_.center;
                const V3D level_center=map_to_level_*V3D(center.x,center.y,center.z);
                relocalization_options_.center={level_center.x(),level_center.y(),level_center.z()};
                startup_ = std::make_unique<fastlio_relocalization::Startup>(
                    std::make_shared<fastlio_relocalization::Matcher>(reference,relocalization_options_),
                    accumulation_frames_,confirmation_frames_);
                RCLCPP_INFO(this->get_logger(),
                    "Startup relocalization: level-frame center [%.2f, %.2f, %.2f] transformed from map center, radius %.2f m, height +/-%.2f m, heading 360 deg. Keep stationary until status=ready.",
                    relocalization_options_.center.x,relocalization_options_.center.y,relocalization_options_.center.z,
                    relocalization_options_.radius,relocalization_options_.z_range);
            }
            RCLCPP_INFO(this->get_logger(),
                "Localization mode: reference map loaded from '%s' (%zu points after %.2f m voxel filter).",
                loc_map_path.c_str(), map_cloud->points.size(), loc_map_voxel_size);

            /* Publish the reference map once on a latched topic: rviz shows it
             * whenever it subscribes, with zero recurring cost (unlike map_en,
             * which re-serializes an accumulating cloud every second). */
            pubReferenceMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/reference_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::VoxelGrid<PointType> display_filter;
            const float display_leaf=this->get_parameter("publish.reference_map_voxel_size").as_double();
            display_filter.setLeafSize(display_leaf,display_leaf,display_leaf); display_filter.setInputCloud(map_cloud);
            PointCloudXYZI display; display_filter.filter(display);
            const auto cap=std::size_t(this->get_parameter("publish.reference_map_max_points").as_int());
            if(display.size()>cap) {
                const std::size_t stride=(display.size()+cap-1)/cap; PointCloudXYZI sampled;
                for(std::size_t i=0;i<display.size();i+=stride) sampled.push_back(display[i]);
                display.swap(sampled);
            }
            pcl::toROSMsg(display, map_msg);
            map_msg.header.stamp = this->get_clock()->now();
            map_msg.header.frame_id = "camera_init";
            pubReferenceMap_->publish(map_msg);

            if (!relocalization_enabled_ && loc_initial_pose.size() >= 4)
            {
                double ix = loc_initial_pose[0], iy = loc_initial_pose[1];
                double iz = loc_initial_pose[2], iyaw = loc_initial_pose[3];
                state_ikfom init_state = kf.get_x();
                init_state.pos = V3D(ix, iy, iz);
                init_state.rot = Eigen::Quaterniond(cos(iyaw * 0.5), 0.0, 0.0, sin(iyaw * 0.5));
                kf.change_x(init_state);
                RCLCPP_INFO(this->get_logger(),
                    "Localization mode: initial pose set to [x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad] in map frame.",
                    ix, iy, iz, iyaw);
            }
            else if (!relocalization_enabled_)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Localization mode: localization.initial_pose not set (or < 4 values), starting at map origin [0,0,0,0].");
            }
            // Never write the reference map back in localization mode.
            pcd_save_en = false;
        }

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = runtime_pos_log ? fopen(pos_log_dir.c_str(),"w") : nullptr;

        // ofstream fout_pre, fout_out, fout_dbg;
        if(runtime_pos_log) {
            fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
            fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
            fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        }
        if (runtime_pos_log && fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else if(runtime_pos_log)
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        /* GT system: re-seed the localization pose (RViz "2D Pose Estimate",
         * scripts/gt_record.py, or any /initialpose publisher). */
        subInitialPose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1, std::bind(&LaserMappingNode::initial_pose_cbk, this, std::placeholders::_1));
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        pubTrackingStatus_=this->create_publisher<std_msgs::msg::String>("/tracking/status",rclcpp::QoS(1).transient_local());
        pubSaveStatus_=this->create_publisher<std_msgs::msg::String>("/map_save/status",rclcpp::QoS(1).transient_local());
        if (localization_mode) {
            pubLocalizationStatus_ = this->create_publisher<std_msgs::msg::String>(
                "/localization/status",rclcpp::QoS(1).transient_local());
            pubMatchedPose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/initial_pose",rclcpp::QoS(1).transient_local());
            publish_localization_status();
            relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>("relocalize",
                std::bind(&LaserMappingNode::relocalize_callback,this,std::placeholders::_1,std::placeholders::_2));
        }
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));
        housekeeping_timer_=this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&LaserMappingNode::housekeeping,this));
        publish_tracking_status("waiting_for_sensor_data");

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        start_path_record_srv_ = this->create_service<std_srvs::srv::Trigger>("start_path_record", std::bind(&LaserMappingNode::start_path_record_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_path_record_srv_ = this->create_service<std_srvs::srv::Trigger>("stop_path_record", std::bind(&LaserMappingNode::stop_path_record_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        finish_session();
        startup_.reset(); // cancel/join the bounded search before other members die
        fout_out.close();
        fout_pre.close();
        if(fp) fclose(fp);
    }

    bool finish_session() {
        if(finished_) return finish_success_;
        finished_=true;
        try { if(csv_.active()) csv_.finish("# interrupted: true\n# end_reason: node_shutdown\n"); }
        catch(const std::exception& e) { finish_success_=false; std::cerr<<"CSV finalization failed: "<<e.what()<<std::endl; }
        is_recording_=false;
        try {
            if(save_worker_.valid()) { save_worker_.wait(); poll_save(); }
            if(!localization_mode && pcd_save_en && archive_ && archive_->size() && saved_revision_!=map_revision_) {
                const auto cloud=map_snapshot(); const auto metadata=map_metadata(cloud->size());
                const auto result=write_snapshot(cloud,metadata,map_file_path,map_revision_);
                finish_success_=finish_success_ && result.success; std::cout<<result.message<<std::endl;
            }
        } catch(const std::exception& e) { finish_success_=false; std::cerr<<"Session finalization failed: "<<e.what()<<std::endl; }
        return finish_success_;
    }

private:
    void declare_runtime_parameters() {
        for(const auto& p:std::vector<std::pair<std::string,double>>{
            {"tracking.min_ratio",0.05},{"tracking.max_residual",0.20},{"tracking.stale_seconds",1.0},
            {"tracking.max_position_std",2.0},
            {"pcd_save.voxel_size",0.1},{"pcd_save.checkpoint_interval_sec",60.0},
            {"publish.map_interval_sec",5.0},{"publish.map_voxel_size",0.5},
            {"record.flush_interval_sec",1.0},{"localization.relocalization.max_tilt_deg",20.0}})
            this->declare_parameter<double>(p.first,p.second);
        for(const auto& p:std::vector<std::pair<std::string,int>>{
            {"tracking.min_points",20},{"tracking.lost_frames",10},{"tracking.recovery_frames",3},
            {"tracking.max_lidar_queue",30},{"tracking.max_imu_queue",2000},
            {"pcd_save.max_points",2000000},{"publish.map_max_points",100000},
            {"publish.path_max_poses",2000},{"record.max_display_poses",2000},{"record.max_saved_paths",3}})
            this->declare_parameter<int>(p.first,p.second);
        this->declare_parameter<bool>("pcd_save.dense_input",true);
        this->declare_parameter<bool>("localization.relocalization.gravity_alignment",true);
        this->declare_parameter<bool>("localization.metadata_verified",false);
        this->declare_parameter<vector<double>>("localization.map_gravity",vector<double>{0,0,-1});
        this->declare_parameter<string>("record.dir",string(ROOT_DIR)+"../../records");
        this->declare_parameter<double>("publish.reference_map_voxel_size",0.5);
        this->declare_parameter<int>("publish.reference_map_max_points",100000);
    }
    static M3D eigen_rotation(const fastlio_runtime::Rotation& r) {
        M3D result; for(int i=0;i<3;++i) for(int j=0;j<3;++j) result(i,j)=r[3*i+j]; return result;
    }
    void initialize_runtime() {
        auto number=[&](const char* key){return this->get_parameter(key).as_double();};
        auto integer=[&](const char* key){return this->get_parameter(key).as_int();};
        tracking_options.min_points=integer("tracking.min_points"); tracking_options.min_ratio=number("tracking.min_ratio");
        tracking_options.max_residual=number("tracking.max_residual"); tracking_options.lost_frames=integer("tracking.lost_frames");
        tracking_options.recovery_frames=integer("tracking.recovery_frames"); tracking_options.stale_seconds=number("tracking.stale_seconds");
        health_=std::make_unique<fastlio_runtime::Health>(tracking_options);
        if(integer("max_iteration")<1) throw std::invalid_argument("max_iteration must be positive");
        if(localization_mode && (!std::isfinite(loc_map_voxel_size) || loc_map_voxel_size<0 || (loc_map_voxel_size>0 && loc_map_voxel_size<1e-4)))
            throw std::invalid_argument("localization.map_voxel_size must be 0 or >= 0.0001 m");
        if(localization_mode && !relocalization_enabled_ && !loc_initial_pose.empty()) {
            if(loc_initial_pose.size()!=4) throw std::invalid_argument("localization.initial_pose requires [x,y,z,yaw]");
            for(double value:loc_initial_pose) if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite localization.initial_pose");
        }
        if(p_pre->point_filter_num<1 || p_pre->N_SCANS<1 || p_pre->SCAN_RATE<1 ||
            !std::isfinite(p_pre->blind) || p_pre->blind<0) throw std::invalid_argument("Invalid preprocessing count/rate/blind range");
        if(!std::isfinite(p_pre->operator_filter_rear_angle) || p_pre->operator_filter_rear_angle<0 || p_pre->operator_filter_rear_angle>180 ||
            !std::isfinite(p_pre->operator_filter_range_min) || p_pre->operator_filter_range_min<0 ||
            !std::isfinite(p_pre->operator_filter_range_max) || p_pre->operator_filter_range_max<p_pre->operator_filter_range_min)
            throw std::invalid_argument("Invalid operator filter angle/range");
        if(integer("tracking.max_lidar_queue")<1 || integer("tracking.max_imu_queue")<1) throw std::invalid_argument("Sensor queue limits must be positive");
        max_lidar_queue=integer("tracking.max_lidar_queue"); max_imu_queue=integer("tracking.max_imu_queue");
        if(extrinT.size()!=3 || extrinR.size()!=9) throw std::invalid_argument("mapping.extrinsic_T/R require 3/9 values");
        for(double value:extrinT) if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite LiDAR extrinsic translation");
        for(double value:extrinR) if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite LiDAR extrinsic rotation");
        M3D rotation; for(int i=0;i<3;++i) for(int j=0;j<3;++j) rotation(i,j)=extrinR[3*i+j];
        if((rotation.transpose()*rotation-M3D::Identity()).norm()>1e-5 || std::abs(rotation.determinant()-1)>1e-5)
            throw std::invalid_argument("mapping.extrinsic_R must be a proper orthonormal rotation");
        if(!localization_mode) fastlio_runtime::validate_window(cube_len,DET_RANGE);
        for(const char* key:{"filter_size_surf","filter_size_map","publish.map_interval_sec","publish.map_voxel_size","record.flush_interval_sec",
            "mapping.acc_cov","mapping.gyr_cov","mapping.b_acc_cov","mapping.b_gyr_cov","tracking.max_position_std"})
            if(!std::isfinite(number(key)) || number(key)<=0) throw std::invalid_argument(std::string(key)+" must be finite and positive");
        checkpoint_interval_=number("pcd_save.checkpoint_interval_sec");
        if(!std::isfinite(checkpoint_interval_) || checkpoint_interval_<0) throw std::invalid_argument("Checkpoint interval must be >= 0 (0 disables)");
        for(const char* key:{"publish.map_max_points","publish.path_max_poses","record.max_display_poses","record.max_saved_paths"})
            if(integer(key)<1) throw std::invalid_argument(std::string(key)+" must be positive");
        if(integer("publish.reference_map_max_points")<1 || !std::isfinite(number("publish.reference_map_voxel_size")) || number("publish.reference_map_voxel_size")<=0)
            throw std::invalid_argument("Reference-map display limits must be positive");
        if(integer("pcd_save.max_points")<0) throw std::invalid_argument("pcd_save.max_points must be >= 0 (0 explicitly disables the cap)");
        archive_=std::make_unique<fastlio_runtime::VoxelMap<PointType>>(number("pcd_save.voxel_size"),integer("pcd_save.max_points"));
        gravity_alignment_=this->get_parameter("localization.relocalization.gravity_alignment").as_bool();
        const auto gravity=this->get_parameter("localization.map_gravity").as_double_array();
        if(gravity.size()!=3) throw std::invalid_argument("localization.map_gravity needs 3 values");
        if(localization_mode && relocalization_enabled_ && gravity_alignment_)
            map_to_level_=eigen_rotation(fastlio_runtime::gravity_to_level({gravity[0],gravity[1],gravity[2]}));
        max_tilt_deg_=number("localization.relocalization.max_tilt_deg");
        if(!std::isfinite(max_tilt_deg_) || max_tilt_deg_<=0 || max_tilt_deg_>=90) throw std::invalid_argument("max_tilt_deg must be in (0,89) degrees");
        if(localization_mode && !this->get_parameter("localization.metadata_verified").as_bool())
            RCLCPP_WARN(this->get_logger(),"Reference map has no verified metadata. Legacy map: verify calibration and configure map_gravity if its initial frame was tilted.");
        last_checkpoint_=last_flush_=last_display_=last_data_time_=std::chrono::steady_clock::now();
        if(std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()<1577836800)
            RCLCPP_WARN(this->get_logger(),"System clock is before 2020. Synchronize host/Jetson clocks before cross-machine trajectory comparison; session filenames use wall time.");
    }
    PointCloudXYZI::Ptr map_snapshot(std::size_t limit=0) const {
        auto cloud=std::make_shared<PointCloudXYZI>(); archive_->snapshot_into(cloud->points,limit);
        cloud->width=cloud->size(); cloud->height=1; cloud->is_dense=true;
        return cloud;
    }
    void accumulate_map_frame() {
        if(map_revision_==0) map_gravity_=V3D(state_point.grav[0],state_point.grav[1],state_point.grav[2]);
        const auto source=this->get_parameter("pcd_save.dense_input").as_bool() ? feats_undistort : feats_down_body;
        for(const auto& p:*source) { PointType world=p; RGBpointBodyToWorld(&p,&world); archive_->insert(world); }
        ++map_revision_;
        if(!archive_->complete()) RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),5000,
            "Map capacity reached (%zu points). New regions are NOT stored; metadata marks map incomplete. Increase pcd_save.max_points or voxel_size and rebuild the map.",archive_->size());
    }
    std::string parameter_json(const rclcpp::Parameter& p) const {
        std::ostringstream out; out<<std::setprecision(17);
        switch(p.get_type()) {
            case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool()?"true":"false";
            case rclcpp::ParameterType::PARAMETER_INTEGER: out<<p.as_int(); break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE: out<<p.as_double(); break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
                out<<'['; bool first=true; for(double value:p.as_double_array()){if(!first)out<<','; first=false; out<<value;} out<<']'; break;
            }
            default: return fastlio_runtime::json_string(p.value_to_string());
        }
        return out.str();
    }
    std::string map_metadata(std::size_t points) const {
        std::ostringstream out; out<<std::setprecision(17);
        out<<"{\"schema_version\":1,\"workspace_version\":\"runtime_safety_v1\",\"frame_id\":\"camera_init\",\"pose_frame\":\"IMU\",\"complete\":"
           <<(archive_->complete()?"true":"false")<<",\"points\":"<<points<<",\"snapshot_revision\":"<<map_revision_
           <<",\"build_revision\":"<<fastlio_runtime::json_string(FASTLIO_BUILD_REVISION)
           <<",\"input_points\":"<<archive_->input()<<",\"capacity_rejected_points\":"<<archive_->rejected()
           <<",\"invalid_input_points\":"<<archive_->invalid()
           <<",\"gravity\":["<<map_gravity_.x()<<','<<map_gravity_.y()<<','<<map_gravity_.z()
           <<"],\"parameters\":{";
        bool first=true;
        for(const char* key:{"preprocess.lidar_type","preprocess.scan_line","preprocess.scan_rate","preprocess.timestamp_unit","preprocess.blind",
            "preprocess.operator_filter_en","preprocess.operator_filter_rear_angle","preprocess.operator_filter_range_min","preprocess.operator_filter_range_max",
            "mapping.extrinsic_T","mapping.extrinsic_R","mapping.extrinsic_est_en","mapping.acc_cov","mapping.gyr_cov","mapping.b_acc_cov","mapping.b_gyr_cov",
            "common.time_sync_en","common.time_offset_lidar_to_imu","point_filter_num","feature_extract_enable","filter_size_surf","filter_size_map",
            "pcd_save.voxel_size","pcd_save.dense_input","cube_side_length","mapping.det_range"}) {
            if(!first) out<<','; first=false; out<<fastlio_runtime::json_string(key)<<':'<<parameter_json(this->get_parameter(key));
        }
        out<<"}}"; return out.str();
    }
    struct SaveResult { bool success; std::string message; std::uint64_t revision; };
    static SaveResult write_snapshot(const PointCloudXYZI::Ptr& cloud,const std::string& metadata,const std::string& output,std::uint64_t revision) {
        std::string message;
        bool ok=fastlio_maps::write_atomic(output,*cloud,message);
        if(ok) {
            const std::string manifest=metadata.substr(0,metadata.size()-1)+",\"pcd_bytes\":"+
                std::to_string(std::filesystem::file_size(output))+",\"pcd_crc32\":"+
                fastlio_runtime::json_string(fastlio_runtime::file_crc32(output))+"}";
            ok=fastlio_maps::write_metadata(output+".json",manifest,message);
        }
        return {ok,message,revision};
    }
    void save_status(const std::string& state,const std::string& detail) {
        if(!pubSaveStatus_ || !rclcpp::ok()) return;
        std_msgs::msg::String message;
        message.data="{\"state\":"+fastlio_runtime::json_string(state)+",\"detail\":"+fastlio_runtime::json_string(detail)+",\"path\":"+fastlio_runtime::json_string(map_file_path)+"}";
        pubSaveStatus_->publish(message);
    }
    void poll_save() {
        if(!save_worker_.valid() || save_worker_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return;
        try { const auto result=save_worker_.get(); if(result.success) saved_revision_=result.revision;
            save_status(result.success?"saved":"failed",result.message);
            if(result.success) RCLCPP_INFO(this->get_logger(),"%s",result.message.c_str());
            else RCLCPP_ERROR(this->get_logger(),"%s",result.message.c_str());
        } catch(const std::exception& e) { save_status("failed",e.what()); RCLCPP_ERROR(this->get_logger(),"Map save failed: %s",e.what()); }
    }
    bool request_save(std::string& message) {
        poll_save();
        if(save_worker_.valid()) {message="A map save is already running";return false;}
        if(!archive_->size()) {message="No mapping points available; no PCD written.";return false;}
        try {
            const auto cloud=map_snapshot(); const auto metadata=map_metadata(cloud->size());
            const auto path=map_file_path; const auto revision=map_revision_;
            save_worker_=std::async(std::launch::async,[cloud,metadata,path,revision]{ return write_snapshot(cloud,metadata,path,revision); });
            message="Map save queued; check /map_save/status for saved/failed: "+path;
            save_status("saving",message); return true;
        } catch(const std::exception& e) { message=e.what(); return false; }
    }
    void publish_tracking_status(const std::string& reason) {
        if(!pubTrackingStatus_) return;
        if(reason!="quality_heartbeat") last_tracking_reason_=reason;
        std::ostringstream out;
        out<<"{\"state\":\""<<health_->name()<<"\",\"reason\":"<<fastlio_runtime::json_string(last_tracking_reason_)
           <<",\"effective_points\":"<<effct_feat_num<<",\"scan_points\":"<<feats_down_size
           <<",\"match_ratio\":"<<double(effct_feat_num)/std::max(1,feats_down_size)
           <<",\"last_sensor_stamp\":"<<std::setprecision(17)<<last_sensor_stamp_
           <<",\"position_std\":"<<(std::isfinite(last_position_std_) && last_position_std_>=0?std::to_string(last_position_std_):"null")
           <<",\"mean_residual\":"<<(std::isfinite(res_mean_last)?std::to_string(res_mean_last):"null")
           <<",\"map_points\":"<<archive_->size()<<",\"map_complete\":"<<(archive_->complete()?"true":"false")<<"}";
        std_msgs::msg::String message; message.data=out.str(); pubTrackingStatus_->publish(message);
    }
    void reject_tracking_frame(const std::string& reason,int points) {
        effct_feat_num=0; feats_down_size=points; res_mean_last=std::numeric_limits<double>::infinity();
        last_position_std_=-1;
        if(std::isfinite(lidar_end_time) && lidar_end_time>last_sensor_stamp_) {
            last_data_time_=std::chrono::steady_clock::now(); tracking_started_=true;
        }
        last_sensor_stamp_=lidar_end_time;
        health_->update(0,points,res_mean_last,false);
        publish_tracking_status(reason); publish_localization_status();
    }
    void housekeeping() {
        poll_save(); const auto now=std::chrono::steady_clock::now();
        if(input_discontinuity.exchange(false)) {
            health_->stale();
            if(startup_ && !startup_->ready()) startup_->invalidate("sensor_queue_overflow_or_timestamp_regression");
            publish_tracking_status("sensor_queue_overflow_or_timestamp_regression"); publish_localization_status();
        }
        if(tracking_started_ && health_->state()!=fastlio_runtime::Health::State::Lost && std::chrono::duration<double>(now-last_data_time_).count()>tracking_options.stale_seconds) {
            health_->stale(); publish_tracking_status("sensor_data_timeout");
            publish_localization_status();
        }
        if(std::chrono::duration<double>(now-last_flush_).count()>=this->get_parameter("record.flush_interval_sec").as_double()) {
            last_flush_=now;
            try { csv_.flush(); } catch(const std::exception& e) { is_recording_=false; RCLCPP_ERROR(this->get_logger(),"%s",e.what()); }
            publish_tracking_status("quality_heartbeat");
        }
        if(!localization_mode && pcd_save_en && checkpoint_interval_>0 &&
            std::chrono::duration<double>(now-last_checkpoint_).count()>=checkpoint_interval_) {
            last_checkpoint_=now; if(map_revision_!=saved_revision_ && !save_worker_.valid()) {std::string message; request_save(message);}
        }
    }
    void declare_relocalization_parameters() {
        const std::string prefix="localization.relocalization.";
        this->declare_parameter<bool>(prefix+"enabled",true);
        this->declare_parameter<vector<double>>(prefix+"center",vector<double>{0.0,0.0,0.0});
        this->declare_parameter<vector<double>>("localization.matched_pose",vector<double>());
        this->declare_parameter<double>("localization.matched_rmse",-1.0);
        this->declare_parameter<double>("localization.matched_overlap",0.0);
        for (const auto& p : std::vector<std::pair<std::string,double>>{
            {"radius",3.0},{"z_range",0.5},{"xy_step",0.75},{"z_step",0.5},{"yaw_step_deg",15.0},
            {"voxel_size",0.25},{"scan_range",20.0},{"coarse_distance",1.0},{"icp_distance",0.75},
            {"inlier_distance",0.35},{"min_overlap",0.65},{"max_rmse",0.18},{"ambiguity_margin",0.02},
            {"max_seconds",20.0},{"stationary_gyro",0.08},{"stationary_acc_fraction",0.05}})
            this->declare_parameter<double>(prefix+p.first,p.second);
        for (const auto& p : std::vector<std::pair<std::string,int>>{
            {"min_points",120},{"coarse_points",256},{"max_points",6000},{"candidates",24},
            {"iterations",50},{"accumulation_frames",15},{"confirmation_frames",3}})
            this->declare_parameter<int>(prefix+p.first,p.second);
    }
    void read_relocalization_parameters() {
        const std::string prefix="localization.relocalization.";
        relocalization_enabled_=this->get_parameter(prefix+"enabled").as_bool();
        if (!localization_mode || !relocalization_enabled_) return;
        auto center=this->get_parameter(prefix+"center").as_double_array();
        if (center.size()!=3) throw std::invalid_argument("Relocalization center needs exactly x,y,z");
        relocalization_options_.center={center[0],center[1],center[2]};
        auto number=[&](const char* key) { return this->get_parameter(prefix+key).as_double(); };
        auto integer=[&](const char* key) { return static_cast<int>(this->get_parameter(prefix+key).as_int()); };
        auto& o=relocalization_options_;
        o.radius=number("radius"); o.z_range=number("z_range"); o.xy_step=number("xy_step");
        o.z_step=number("z_step"); o.yaw_step_deg=number("yaw_step_deg"); o.voxel_size=number("voxel_size");
        o.scan_range=number("scan_range"); o.coarse_distance=number("coarse_distance");
        o.icp_distance=number("icp_distance"); o.inlier_distance=number("inlier_distance");
        o.min_overlap=number("min_overlap"); o.max_rmse=number("max_rmse");
        o.ambiguity_margin=number("ambiguity_margin"); o.max_seconds=number("max_seconds");
        o.min_points=integer("min_points"); o.coarse_points=integer("coarse_points");
        o.max_points=integer("max_points"); o.candidates=integer("candidates"); o.iterations=integer("iterations");
        accumulation_frames_=integer("accumulation_frames"); confirmation_frames_=integer("confirmation_frames");
        stationary_gyro_=number("stationary_gyro"); stationary_acc_fraction_=number("stationary_acc_fraction");
        o.validate();
        if (!std::isfinite(stationary_gyro_) || stationary_gyro_<=0 ||
            !std::isfinite(stationary_acc_fraction_) || stationary_acc_fraction_<=0)
            throw std::invalid_argument("Stationary detection thresholds must be finite and positive");
    }
    void publish_localization_status() {
        if (!pubLocalizationStatus_) return;
        std::string state=startup_ ? startup_->state() : "manual";
        std::string detail=startup_ ? startup_->detail() : "manual_initial_pose";
        if((!startup_ || startup_->ready()) && health_ &&
            (health_->state()==fastlio_runtime::Health::State::Degraded || health_->state()==fastlio_runtime::Health::State::Lost)) {
            state=health_->name(); detail="tracking_quality_"+state;
        }
        if (state+detail==last_localization_status_) return;
        last_localization_status_=state+detail;
        std_msgs::msg::String message;
        message.data="{\"state\":"+fastlio_runtime::json_string(state)+",\"detail\":"+fastlio_runtime::json_string(detail)+"}";
        pubLocalizationStatus_->publish(message);
        if (state=="failed" || state=="lost") RCLCPP_WARN(this->get_logger(),
            "Startup relocalization failed: %s. No odometry/trajectory will be published. Keep stationary, verify map/range, then call /relocalize.",detail.c_str());
        else RCLCPP_INFO(this->get_logger(),"Localization status: %s (%s)",state.c_str(),detail.c_str());
    }
    bool startup_stationary() {
        if (Measures.imu.empty()) return false;
        for (const auto& imu : Measures.imu) {
            V3D gyro(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z);
            V3D acc(imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z);
            if (!gyro.allFinite() || !acc.allFinite() || acc.norm()<1e-6 ||
                gyro.norm()>stationary_gyro_) return false;
            if (!stationary_acc_set_) { stationary_acc_=acc; stationary_acc_set_=true; }
            if ((acc-stationary_acc_).norm()/stationary_acc_.norm()>stationary_acc_fraction_) {
                stationary_acc_=acc; return false;
            }
        }
        return true;
    }
    void process_startup_relocalization() {
        std::vector<fastlio_relocalization::Point> frame;
        const bool stationary=startup_stationary();
        if(p_imu->initialized() && !stationary) {
            startup_->update({},false,true);
            if(startup_->phase()==fastlio_relocalization::Startup::Phase::Collecting) startup_tilt_set_=false;
            publish_localization_status(); return;
        }
        if (p_imu->initialized()) {
            if(!startup_tilt_set_ && gravity_alignment_) {
                const V3D body_gravity=state_point.rot.toRotationMatrix().transpose()*V3D(state_point.grav[0],state_point.grav[1],state_point.grav[2]);
                try { imu_to_level_=eigen_rotation(fastlio_runtime::gravity_to_level({body_gravity.x(),body_gravity.y(),body_gravity.z()},max_tilt_deg_)); }
                catch(const std::exception& e) { startup_->reject_initialization(e.what()); publish_localization_status(); return; }
            }
            startup_tilt_set_=true;
            frame.reserve(feats_undistort->size());
            for (const auto& p : *feats_undistort) {
                // Engine solves map <- IMU. Do not confuse it with map <- LiDAR.
                const V3D imu_point=imu_to_level_*(state_point.offset_R_L_I*V3D(p.x,p.y,p.z)+state_point.offset_T_L_I);
                frame.push_back({imu_point.x(),imu_point.y(),imu_point.z()});
            }
        }
        const bool became_ready=startup_->update(frame,stationary,p_imu->initialized());
        if (became_ready) {
            const auto& result=startup_->result(); const auto& pose=result.pose;
            state_ikfom seeded=kf.get_x(); const M3D old_rotation=seeded.rot.toRotationMatrix();
            seeded.pos=map_to_level_.transpose()*V3D(pose.x,pose.y,pose.z);
            seeded.rot=Eigen::Quaterniond(map_to_level_.transpose()*Eigen::AngleAxisd(pose.yaw,V3D::UnitZ()).toRotationMatrix()*imu_to_level_);
            const M3D rotation_delta=seeded.rot.toRotationMatrix()*old_rotation.transpose();
            seeded.grav=S2(rotation_delta*V3D(seeded.grav[0],seeded.grav[1],seeded.grav[2]));
            seeded.vel=Zero3d;
            kf.change_x(seeded);
            auto covariance=kf.get_P();
            for (int i : {0,1,2,3,4,5,12,13,14}) {
                covariance.row(i).setZero(); covariance.col(i).setZero(); covariance(i,i)=0.01;
            }
            kf.change_P(covariance);
            p_imu->rotate_world_history(rotation_delta);
            state_point=seeded; position_last=seeded.pos; path.poses.clear();
            geometry_msgs::msg::PoseWithCovarianceStamped message;
            message.header.frame_id="camera_init"; message.header.stamp=get_ros_time(lidar_end_time);
            message.pose.pose.position.x=seeded.pos.x(); message.pose.pose.position.y=seeded.pos.y(); message.pose.pose.position.z=seeded.pos.z();
            message.pose.pose.orientation.x=seeded.rot.x(); message.pose.pose.orientation.y=seeded.rot.y();
            message.pose.pose.orientation.z=seeded.rot.z(); message.pose.pose.orientation.w=seeded.rot.w();
            for (int i : {0,7,14,21,28,35}) message.pose.covariance[i]=0.01;
            pubMatchedPose_->publish(message);
            this->set_parameters({
                rclcpp::Parameter("localization.matched_pose",vector<double>{seeded.pos.x(),seeded.pos.y(),seeded.pos.z(),std::atan2(seeded.rot.matrix()(1,0),seeded.rot.matrix()(0,0))}),
                rclcpp::Parameter("localization.matched_rmse",result.rmse),
                rclcpp::Parameter("localization.matched_overlap",result.overlap)});
            RCLCPP_INFO(this->get_logger(),
                "Startup relocalization READY: initial_pose=[%.4f, %.4f, %.4f, %.6f] (yaw %.2f deg), overlap=%.3f, RMSE=%.3f m, search=%.2f s. Pose is map <- IMU; you may move now.",
                seeded.pos.x(),seeded.pos.y(),seeded.pos.z(),std::atan2(seeded.rot.matrix()(1,0),seeded.rot.matrix()(0,0)),
                std::atan2(seeded.rot.matrix()(1,0),seeded.rot.matrix()(0,0))*180/fastlio_relocalization::pi,result.overlap,result.rmse,result.seconds);
            health_->reset(); tracking_started_=true; last_data_time_=std::chrono::steady_clock::now();
        }
        publish_localization_status();
    }
    void relocalize_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!startup_) { response->success=false; response->message="Automatic relocalization is disabled."; return; }
        if (is_recording_ || csv_.active()) { response->success=false; response->message="Stop trajectory recording before relocalizing."; return; }
        if (!startup_->reset()) { response->success=false; response->message="Search is busy; wait for matching to finish."; return; }
        state_ikfom state=kf.get_x(); state.pos=Zero3d; state.vel=Zero3d;
        state.rot=Eigen::Quaterniond::Identity(); state.bg=Zero3d; state.ba=Zero3d;
        state.grav=S2(V3D(0,0,-G_m_s2)); kf.change_x(state);
        auto fresh_covariance=kf.get_P(); fresh_covariance.setIdentity(); kf.change_P(fresh_covariance);
        p_imu->Reset(); feats_undistort->clear(); path.poses.clear(); stationary_acc_set_=false;
        startup_tilt_set_=false; imu_to_level_.setIdentity(); health_->reset(); tracking_started_=false; last_sensor_stamp_=-1;
        last_position_std_=-1; effct_feat_num=0; feats_down_size=0; res_mean_last=std::numeric_limits<double>::infinity();
        publish_tracking_status("relocalization_requested");
        this->set_parameters({rclcpp::Parameter("localization.matched_pose",vector<double>()),
            rclcpp::Parameter("localization.matched_rmse",-1.0),rclcpp::Parameter("localization.matched_overlap",0.0)});
        { std::lock_guard<std::mutex> lock(mtx_buffer);
          lidar_buffer.clear(); time_buffer.clear(); imu_buffer.clear(); lidar_pushed=false; }
        flg_first_scan=true;
        input_discontinuity.store(false);
        publish_localization_status();
        response->success=true; response->message="Relocalization requested. Keep stationary; wait for /localization/status ready.";
    }
    void timer_callback()
    {
        if(input_discontinuity.load()) housekeeping();
        if(sync_packages(Measures))
        {
            if(Measures.imu.empty()) return;
            if(health_->state()==fastlio_runtime::Health::State::Lost && (!startup_ || startup_->ready())) return;
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            if (startup_ && !startup_->ready() && Measures.imu.empty()) return;
            if (startup_ && !startup_->ready() && !p_imu->initialized() && !startup_stationary()) {
                p_imu->Reset(); feats_undistort->clear();
                publish_localization_status();
                RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),3000,
                    "Keep stationary for IMU initialization and startup relocalization.");
                return;
            }
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (startup_ && !startup_->ready()) {
                process_startup_relocalization();
                return; // never run map tracking/publish odometry before acceptance
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                if(p_imu->initialized()) reject_tracking_frame("empty_undistorted_scan",0);
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            /* In localization mode the reference map is read-only: never trim it. */
            // Move/trim the local map only after a trusted LiDAR update below.

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            if(feats_down_size>100000) {
                health_->stale(); publish_tracking_status("scan_exceeds_algorithm_capacity"); publish_localization_status(); return;
            }
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                    tracking_started_=true; last_data_time_=std::chrono::steady_clock::now();
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                reject_tracking_frame("insufficient_scan_points",feats_down_size);
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            if(runtime_pos_log) fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            const auto pose_covariance=kf.get_P();
            const double max_position_variance=std::max({pose_covariance(0,0),pose_covariance(1,1),pose_covariance(2,2)});
            last_position_std_=max_position_variance>=0 ? std::sqrt(max_position_variance) : -1;
            const double limit=this->get_parameter("tracking.max_position_std").as_double();
            const bool finite=state_point.pos.allFinite() && state_point.vel.allFinite() && state_point.rot.matrix().allFinite() &&
                state_point.bg.allFinite() && state_point.ba.allFinite() && state_point.offset_T_L_I.allFinite() &&
                state_point.offset_R_L_I.matrix().allFinite() && V3D(state_point.grav[0],state_point.grav[1],state_point.grav[2]).allFinite() &&
                pose_covariance.allFinite() && pose_covariance(0,0)>=0 && pose_covariance(1,1)>=0 && pose_covariance(2,2)>=0 && max_position_variance<=limit*limit;
            const bool ordered=std::isfinite(lidar_end_time) && lidar_end_time>last_sensor_stamp_;
            last_sensor_stamp_=lidar_end_time;
            if(ordered) { last_data_time_=std::chrono::steady_clock::now(); tracking_started_=true; }
            const bool trusted=health_->update(effct_feat_num,feats_down_size,res_mean_last,finite && ordered);
            publish_tracking_status(trusted?"lidar_update_accepted":"lidar_update_untrusted");
            publish_localization_status();
            if(!trusted) {
                RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),3000,
                    "Tracking %s: withholding odometry, map insertion and trajectory samples (effective %d/%d). LOST is latched; localization: stop recording then call /relocalize; mapping: save the trusted map and restart.",health_->name(),effct_feat_num,feats_down_size);
                return;
            }
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            /* In localization mode the reference map is read-only: never add points. */
            if (!localization_mode) { lasermap_fov_segment(); map_incremental(); }
            // Accumulate once per valid frame, independent of RViz/topic publishing.
            if (!localization_mode && (pcd_save_en || map_pub_en)) accumulate_map_frame();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en) publish_path(pubPath_,this->get_parameter("publish.path_max_poses").as_int());
            if (is_recording_)
            {
                /* Record every `record.sample_every_n`-th LiDAR frame (default: every
                 * frame = 10 Hz at the MID360 scan rate). The full path is published
                 * once on stop; live monitoring uses the standard /path topic. */
                record_frame_cnt_++;
                if (record_frame_cnt_ % record_sample_every_n_ == 0)
                {
                    set_posestamp(msg_body_pose);
                    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
                    msg_body_pose.header.frame_id = "camera_init";
                    record_path_.poses.push_back(msg_body_pose);
                    append_record_sample(msg_body_pose);
                    const auto cap=std::size_t(this->get_parameter("record.max_display_poses").as_int());
                    if(record_path_.poses.size()>cap) record_path_.poses.erase(record_path_.poses.begin());
                }
            }
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                if(fp) dump_lio_state_to_log(fp);
                if(time_log_counter>=MAXN) { runtime_pos_log=false; RCLCPP_WARN(this->get_logger(),"Runtime debug buffer full; disabling debug capture."); }
            }
        }
    }

    void map_publish_callback()
    {
        const auto now=std::chrono::steady_clock::now();
        if (map_pub_en && !localization_mode && pubLaserCloudMap_->get_subscription_count()>0 &&
            std::chrono::duration<double>(now-last_display_).count()>=this->get_parameter("publish.map_interval_sec").as_double()) {
            last_display_=now;
            const auto cloud=map_snapshot(this->get_parameter("publish.map_max_points").as_int());
            pcl::VoxelGrid<PointType> filter; const float leaf=this->get_parameter("publish.map_voxel_size").as_double();
            filter.setLeafSize(leaf,leaf,leaf); filter.setInputCloud(cloud); PointCloudXYZI display; filter.filter(display);
            sensor_msgs::msg::PointCloud2 message; pcl::toROSMsg(display,message);
            message.header.frame_id="camera_init"; message.header.stamp=get_ros_time(lidar_end_time); pubLaserCloudMap_->publish(message);
        }
        if (record_republish_saved_)
        {
            for (auto &[pub, path] : saved_recordings_)
            {
                pub->publish(path);
            }
        }
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        (void)req;
        if (localization_mode)
        {
            res->success = false;
            res->message = "map_save is disabled in localization mode: the reference map is read-only.";
            RCLCPP_WARN(this->get_logger(), "map_save rejected: localization mode is active.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            res->success = request_save(res->message);
            if (!res->success) RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

    /* Re-seed the EKF pose from /initialpose (RViz "2D Pose Estimate" or
     * scripts/gt_record.py). Localization mode only: the map frame is the
     * reference-map frame, so this is a jump inside a known frame, not a new
     * frame definition. Refused while a GT recording is active. */
    void initial_pose_cbk(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!localization_mode)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received /initialpose but localization mode is off (SLAM mode defines its own frame). Ignored.");
            return;
        }
        if (is_recording_ || csv_.active())
        {
            RCLCPP_ERROR(this->get_logger(),
                "Received /initialpose while recording: refusing to jump the GT pose. Stop recording first.");
            return;
        }
        if (startup_) {
            RCLCPP_WARN(this->get_logger(),
                "Ignoring /initialpose in automatic startup mode. Use /relocalize, or restart with relocalize:=false for manual seeding.");
            return;
        }
        if (health_->state()==fastlio_runtime::Health::State::Lost) {
            RCLCPP_ERROR(this->get_logger(),
                "Manual localization is LOST. Restart to reset IMU integration; /initialpose cannot safely bridge a sensor discontinuity.");
            return;
        }
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);
        if ((!msg->header.frame_id.empty() && msg->header.frame_id!="camera_init") ||
            !V3D(p.x,p.y,p.z).allFinite() || !orientation.coeffs().allFinite() || orientation.norm()<1e-6) {
            RCLCPP_ERROR(this->get_logger(),"Ignoring invalid /initialpose: expected a finite pose in camera_init.");
            return;
        }
        orientation.normalize();
        const auto rotation=orientation.toRotationMatrix();
        const double yaw=std::atan2(rotation(1,0),rotation(0,0));
        state_ikfom init_state = kf.get_x();
        const M3D old_rotation=init_state.rot.toRotationMatrix();
        init_state.pos = V3D(p.x, p.y, p.z);
        init_state.rot = Eigen::Quaterniond(cos(yaw * 0.5), 0.0, 0.0, sin(yaw * 0.5));
        const M3D rotation_delta=init_state.rot.toRotationMatrix()*old_rotation.transpose();
        init_state.grav=S2(rotation_delta*V3D(init_state.grav[0],init_state.grav[1],init_state.grav[2]));
        init_state.vel=Zero3d;
        kf.change_x(init_state);
        auto covariance=kf.get_P();
        for(int i:{0,1,2,3,4,5,12,13,14}) {
            covariance.row(i).setZero(); covariance.col(i).setZero(); covariance(i,i)=0.01;
        }
        kf.change_P(covariance); p_imu->rotate_world_history(rotation_delta);
        state_point=init_state; position_last=init_state.pos; health_->reset();
        publish_tracking_status("manual_pose_seeded_awaiting_scan_validation");
        publish_localization_status();
        path.poses.clear(); // avoid a jump line in the /path display
        RCLCPP_INFO(this->get_logger(),
                    "Localization re-seeded to [x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad].",
                    p.x, p.y, p.z, yaw);
    }

    void start_path_record_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)    {
        (void)req;
        if (startup_ && !startup_->ready()) {
            res->success=false; res->message="Localization is not ready; wait for startup relocalization."; return;
        }
        if(!health_->trusted()) { res->success=false; res->message="Tracking is not healthy; wait for /tracking/status tracking."; return; }
        if (is_recording_ || csv_.active())
        {
            res->success = false;
            res->message = "Already recording. Please stop first.";
            return;
        }
        record_count_++;
        std::string topic_name = "/path_record_" + std::to_string(record_count_);
        record_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 20);
        record_path_ = nav_msgs::msg::Path();
        record_path_.header.stamp = this->get_clock()->now();
        record_path_.header.frame_id = "camera_init";
        /* Metadata is set on parameters before calling this service (see
         * scripts/gt_record.py), so it lands in the CSV file name and header. */
        this->get_parameter_or<string>("record.control_source", record_control_source_, string(""));
        this->get_parameter_or<string>("record.instruction_id", record_instruction_id_, string(""));
        this->get_parameter_or<string>("record.note", record_note_, string(""));
        this->get_parameter_or<int>("record.sample_every_n", record_sample_every_n_, 1);
        if (record_sample_every_n_ < 1) record_sample_every_n_ = 1;
        record_frame_cnt_ = 0;
        record_total_poses_=0;
        try {
            const auto session=fastlio_maps::prepare_output(this->get_parameter("record.dir").as_string(),"record");
            const auto prefix="path_record_"+std::to_string(record_count_)+"_"+sanitize_for_filename(record_control_source_)+"_"+sanitize_for_filename(record_instruction_id_)+"_"+session.stem().string();
            std::ostringstream header;
            header<<"# fastlio_gt_recording_version: 3\n# frame_id: camera_init\n# pose_frame: IMU\n# reference_map: "<<fastlio_runtime::json_string(loc_map_path)
                <<"\n# control_source: "<<fastlio_runtime::json_string(record_control_source_)
                <<"\n# instruction_id: "<<fastlio_runtime::json_string(record_instruction_id_)
                <<"\n# note: "<<fastlio_runtime::json_string(record_note_)
                <<"\n# sample_every_n: "<<record_sample_every_n_
                <<"\n# localization_mode: "<<(localization_mode?"true":"false")
                <<"\nx,y,z,qx,qy,qz,qw,stamp_sec,stamp_nanosec,effective_points,match_ratio,mean_residual\n";
            csv_.begin(session.parent_path(),prefix,header.str());
        } catch(const std::exception& e) { res->success=false; res->message=e.what(); return; }
        is_recording_ = true;
        res->success = true;
        res->message = "Recording started on topic: " + topic_name +
                       " [" + record_control_source_ + "/" + record_instruction_id_ + "] CSV: "+csv_.path().string();
        RCLCPP_INFO(this->get_logger(), "Path recording started: %s [source=%s instr=%s]",
                    topic_name.c_str(), record_control_source_.c_str(), record_instruction_id_.c_str());
    }

    void append_record_sample(const geometry_msgs::msg::PoseStamped& pose) {
        if(!is_recording_) return;
        std::ostringstream row; row<<std::setprecision(17)
            <<pose.pose.position.x<<','<<pose.pose.position.y<<','<<pose.pose.position.z<<','
            <<pose.pose.orientation.x<<','<<pose.pose.orientation.y<<','<<pose.pose.orientation.z<<','<<pose.pose.orientation.w<<','
            <<pose.header.stamp.sec<<','<<pose.header.stamp.nanosec<<','<<effct_feat_num<<','
            <<double(effct_feat_num)/std::max(1,feats_down_size)<<','<<res_mean_last<<"\n";
        try { csv_.append(row.str()); ++record_total_poses_; }
        catch(const std::exception& e) { is_recording_=false; RCLCPP_ERROR(this->get_logger(),"Recording paused after write failure: %s",e.what()); }
    }

    void stop_path_record_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        if(!is_recording_ && !csv_.active()) { res->success=false; res->message="Not currently recording."; return; }
        is_recording_=false;
        try {
            csv_.finish("# interrupted: false\n# pose_count: "+std::to_string(record_total_poses_)+"\n");
            if(record_pub_) {
                record_pub_->publish(record_path_);
                saved_recordings_.push_back({record_pub_,record_path_});
                const auto cap=std::size_t(this->get_parameter("record.max_saved_paths").as_int());
                if(saved_recordings_.size()>cap) saved_recordings_.erase(saved_recordings_.begin());
            }
            res->success=true; res->message="Recording stopped. "+std::to_string(record_total_poses_)+" trusted poses. CSV: "+csv_.path().string();
            RCLCPP_INFO(this->get_logger(),"%s",res->message.c_str());
        } catch(const std::exception& e) { res->success=false; res->message=e.what(); }
    }

private:
    bool relocalization_enabled_=true;
    std::unique_ptr<fastlio_runtime::Health> health_;
    std::unique_ptr<fastlio_runtime::VoxelMap<PointType>> archive_;
    std::future<SaveResult> save_worker_;
    std::uint64_t map_revision_=0,saved_revision_=0,record_total_poses_=0;
    std::string last_tracking_reason_="waiting_for_sensor_data";
    double checkpoint_interval_=60,max_tilt_deg_=20,last_sensor_stamp_=-1;
    double last_position_std_=-1;
    bool tracking_started_=false,finished_=false,finish_success_=true,gravity_alignment_=true,startup_tilt_set_=false;
    M3D map_to_level_=M3D::Identity(),imu_to_level_=M3D::Identity();
    V3D map_gravity_=V3D(0,0,-G_m_s2);
    std::chrono::steady_clock::time_point last_checkpoint_,last_flush_,last_display_,last_data_time_;
    fastlio_runtime::CsvJournal csv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubTrackingStatus_,pubSaveStatus_;
    rclcpp::TimerBase::SharedPtr housekeeping_timer_;
    fastlio_relocalization::Options relocalization_options_;
    std::unique_ptr<fastlio_relocalization::Startup> startup_;
    int accumulation_frames_=15,confirmation_frames_=3;
    double stationary_gyro_=0.08,stationary_acc_fraction_=0.05;
    bool stationary_acc_set_=false;
    V3D stationary_acc_=Zero3d;
    std::string last_localization_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubLocalizationStatus_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubMatchedPose_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubReferenceMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subInitialPose_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_path_record_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_path_record_srv_;

    bool is_recording_ = false;
    int record_count_ = 0;
    int record_frame_cnt_ = 0;
    int record_sample_every_n_ = 1;
    bool record_republish_saved_ = false;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::string record_control_source_, record_instruction_id_, record_note_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr record_pub_;
    nav_msgs::msg::Path record_path_;
    std::vector<std::pair<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr, nav_msgs::msg::Path>> saved_recordings_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Keep rclcpp's asynchronous signal handler. Calling shutdown() from a
    // raw POSIX signal handler can deadlock and prevent post-spin map saving.

    auto node=std::make_shared<LaserMappingNode>();
    rclcpp::spin(node);
    const bool finalized=node->finish_session();

    if (rclcpp::ok())
        rclcpp::shutdown();
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        if(!fp2) { std::cerr<<"Cannot write runtime debug log: "<<log_dir<<std::endl; return finalized ? 0 : 1; }
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return finalized ? 0 : 1;
}
