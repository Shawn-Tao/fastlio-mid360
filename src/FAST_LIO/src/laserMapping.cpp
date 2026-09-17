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
#include <std_msgs/msg/string.hpp>

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

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
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
        lidar_buffer.clear();
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
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
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
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

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

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
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

void accumulate_map_frame()
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
    *pcl_wait_pub += *laserCloudWorld;
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

bool save_to_pcd(std::string& message)
{
    return fastlio_maps::write_atomic(std::filesystem::path(map_file_path), *pcl_wait_pub, message);
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
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

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

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
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
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

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

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
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
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
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
        this->declare_parameter<bool>("record.republish_saved", true);
        this->declare_parameter<string>("record.control_source", "");
        this->declare_parameter<string>("record.instruction_id", "");
        this->declare_parameter<string>("record.note", "");
        this->declare_parameter<bool>("preprocess.operator_filter_en", false);
        this->declare_parameter<double>("preprocess.operator_filter_rear_angle", 50.0);
        this->declare_parameter<double>("preprocess.operator_filter_range_min", 0.3);
        this->declare_parameter<double>("preprocess.operator_filter_range_max", 3.0);

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
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
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
        this->get_parameter_or<bool>("record.republish_saved", record_republish_saved_, true);
        this->get_parameter_or<bool>("preprocess.operator_filter_en", p_pre->operator_filter_en, false);
        this->get_parameter_or<double>("preprocess.operator_filter_rear_angle", p_pre->operator_filter_rear_angle, 50.0);
        this->get_parameter_or<double>("preprocess.operator_filter_range_min", p_pre->operator_filter_range_min, 0.3);
        this->get_parameter_or<double>("preprocess.operator_filter_range_max", p_pre->operator_filter_range_max, 3.0);
        if (record_sample_every_n_ < 1)
        {
            RCLCPP_WARN(this->get_logger(), "record.sample_every_n < 1 is invalid, reset to 1.");
            record_sample_every_n_ = 1;
        }

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
            if (loc_map_voxel_size > 0.01)
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
                for (const auto& p : *map_cloud) reference.push_back({p.x,p.y,p.z});
                startup_ = std::make_unique<fastlio_relocalization::Startup>(
                    std::make_shared<fastlio_relocalization::Matcher>(reference,relocalization_options_),
                    accumulation_frames_,confirmation_frames_);
                RCLCPP_INFO(this->get_logger(),
                    "Startup relocalization: center [%.2f, %.2f, %.2f], radius %.2f m, height +/-%.2f m, heading 360 deg. Keep stationary until status=ready.",
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
            pcl::toROSMsg(*map_cloud, map_msg);
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
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
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

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        start_path_record_srv_ = this->create_service<std_srvs::srv::Trigger>("start_path_record", std::bind(&LaserMappingNode::start_path_record_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_path_record_srv_ = this->create_service<std_srvs::srv::Trigger>("stop_path_record", std::bind(&LaserMappingNode::stop_path_record_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        startup_.reset(); // cancel/join the bounded search before other members die
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
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
        const std::string state=startup_ ? startup_->state() : "manual";
        const std::string detail=startup_ ? startup_->detail() : "manual_initial_pose";
        if (state+detail==last_localization_status_) return;
        last_localization_status_=state+detail;
        std_msgs::msg::String message;
        message.data="{\"state\":\""+state+"\",\"detail\":\""+detail+"\"}";
        pubLocalizationStatus_->publish(message);
        if (state=="failed") RCLCPP_WARN(this->get_logger(),
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
        if (p_imu->initialized()) {
            frame.reserve(feats_undistort->size());
            for (const auto& p : *feats_undistort) {
                // Engine solves map <- IMU. Do not confuse it with map <- LiDAR.
                const V3D imu_point=state_point.offset_R_L_I*V3D(p.x,p.y,p.z)+state_point.offset_T_L_I;
                frame.push_back({imu_point.x(),imu_point.y(),imu_point.z()});
            }
        }
        const bool became_ready=startup_->update(frame,startup_stationary(),p_imu->initialized());
        if (became_ready) {
            const auto& result=startup_->result(); const auto& pose=result.pose;
            state_ikfom seeded=kf.get_x(); const M3D old_rotation=seeded.rot.toRotationMatrix();
            seeded.pos=V3D(pose.x,pose.y,pose.z);
            seeded.rot=Eigen::Quaterniond(Eigen::AngleAxisd(pose.yaw,V3D::UnitZ()));
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
            message.pose.pose.position.x=pose.x; message.pose.pose.position.y=pose.y; message.pose.pose.position.z=pose.z;
            message.pose.pose.orientation.z=std::sin(pose.yaw*0.5); message.pose.pose.orientation.w=std::cos(pose.yaw*0.5);
            for (int i : {0,7,14,21,28,35}) message.pose.covariance[i]=0.01;
            pubMatchedPose_->publish(message);
            this->set_parameters({
                rclcpp::Parameter("localization.matched_pose",vector<double>{pose.x,pose.y,pose.z,pose.yaw}),
                rclcpp::Parameter("localization.matched_rmse",result.rmse),
                rclcpp::Parameter("localization.matched_overlap",result.overlap)});
            RCLCPP_INFO(this->get_logger(),
                "Startup relocalization READY: initial_pose=[%.4f, %.4f, %.4f, %.6f] (yaw %.2f deg), overlap=%.3f, RMSE=%.3f m, search=%.2f s. Pose is map <- IMU; you may move now.",
                pose.x,pose.y,pose.z,pose.yaw,pose.yaw*180/fastlio_relocalization::pi,result.overlap,result.rmse,result.seconds);
        }
        publish_localization_status();
    }
    void relocalize_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!startup_) { response->success=false; response->message="Automatic relocalization is disabled."; return; }
        if (is_recording_) { response->success=false; response->message="Stop trajectory recording before relocalizing."; return; }
        if (!startup_->reset()) { response->success=false; response->message="Search is busy; wait for matching to finish."; return; }
        state_ikfom state=kf.get_x(); state.pos=Zero3d; state.vel=Zero3d;
        state.rot=Eigen::Quaterniond::Identity(); state.bg=Zero3d; state.ba=Zero3d;
        state.grav=S2(V3D(0,0,-G_m_s2)); kf.change_x(state);
        auto fresh_covariance=kf.get_P(); fresh_covariance.setIdentity(); kf.change_P(fresh_covariance);
        p_imu->Reset(); feats_undistort->clear(); path.poses.clear(); stationary_acc_set_=false;
        this->set_parameters({rclcpp::Parameter("localization.matched_pose",vector<double>()),
            rclcpp::Parameter("localization.matched_rmse",-1.0),rclcpp::Parameter("localization.matched_overlap",0.0)});
        { std::lock_guard<std::mutex> lock(mtx_buffer);
          lidar_buffer.clear(); time_buffer.clear(); imu_buffer.clear(); lidar_pushed=false; }
        flg_first_scan=true;
        publish_localization_status();
        response->success=true; response->message="Relocalization requested. Keep stationary; wait for /localization/status ready.";
    }
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
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
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            /* In localization mode the reference map is read-only: never trim it. */
            if (!localization_mode) lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
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
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
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
            if (!localization_mode) map_incremental();
            // Accumulate once per valid frame, independent of RViz/topic publishing.
            if (!localization_mode && (pcd_save_en || map_pub_en)) accumulate_map_frame();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
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
                }
            }
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

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
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
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
            res->success = save_to_pcd(res->message);
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
        if (is_recording_)
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
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        state_ikfom init_state = kf.get_x();
        init_state.pos = V3D(p.x, p.y, p.z);
        init_state.rot = Eigen::Quaterniond(cos(yaw * 0.5), 0.0, 0.0, sin(yaw * 0.5));
        kf.change_x(init_state);
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
        if (is_recording_)
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
        is_recording_ = true;
        res->success = true;
        res->message = "Recording started on topic: " + topic_name +
                       " [" + record_control_source_ + "/" + record_instruction_id_ + "]";
        RCLCPP_INFO(this->get_logger(), "Path recording started: %s [source=%s instr=%s]",
                    topic_name.c_str(), record_control_source_.c_str(), record_instruction_id_.c_str());
    }

    void stop_path_record_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        (void)req;
        if (!is_recording_)
        {
            res->success = false;
            res->message = "Not currently recording.";
            return;
        }
        is_recording_ = false;
        if (record_pub_)
        {
            record_pub_->publish(record_path_);
            saved_recordings_.push_back({record_pub_, record_path_});

            const char* home = std::getenv("HOME");
            std::string record_dir = home ? std::string(home) + "/Record_Path" : "/tmp/Record_Path";
            mkdir(record_dir.c_str(), 0755);

            auto now = std::chrono::system_clock::now();
            auto now_t = std::chrono::system_clock::to_time_t(now);
            auto local_tm = *std::localtime(&now_t);
            char ts_buf[32];
            std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", &local_tm);

            std::string base_name = "path_record_" + std::to_string(record_count_);
            std::string safe_source = sanitize_for_filename(record_control_source_);
            std::string safe_instr = sanitize_for_filename(record_instruction_id_);
            if (!safe_source.empty()) base_name += "_" + safe_source;
            if (!safe_instr.empty()) base_name += "_" + safe_instr;
            std::string csv_path = record_dir + "/" + base_name + "_" + ts_buf + ".csv";
            std::ofstream csv(csv_path);
            if (csv.is_open())
            {
                /* Metadata block ('#' lines) is ignored by pandas/genfromtxt with
                 * comment='#'; data columns are identical to recording v1. */
                csv << "# fastlio_gt_recording_version: 2\n";
                csv << "# control_source: " << record_control_source_ << "\n";
                csv << "# instruction_id: " << record_instruction_id_ << "\n";
                csv << "# note: " << record_note_ << "\n";
                csv << "# frame_id: " << record_path_.header.frame_id << "\n";
                csv << "# sample_every_n: " << record_sample_every_n_ << "\n";
                csv << "# localization_mode: " << (localization_mode ? "true" : "false") << "\n";
                csv << "# start_stamp_sec: " << record_path_.header.stamp.sec << "\n";
                csv << "# start_stamp_nanosec: " << record_path_.header.stamp.nanosec << "\n";
                csv << "# pose_count: " << record_path_.poses.size() << "\n";
                csv << "x,y,z,qx,qy,qz,qw,stamp_sec,stamp_nanosec\n";
                for (auto &pose : record_path_.poses)
                {
                    csv << pose.pose.position.x << ","
                        << pose.pose.position.y << ","
                        << pose.pose.position.z << ","
                        << pose.pose.orientation.x << ","
                        << pose.pose.orientation.y << ","
                        << pose.pose.orientation.z << ","
                        << pose.pose.orientation.w << ","
                        << pose.header.stamp.sec << ","
                        << pose.header.stamp.nanosec << "\n";
                }
                csv.close();
                res->success = true;
                res->message = "Recording stopped. Path has " + std::to_string(record_path_.poses.size()) + " poses. CSV saved to " + csv_path;
                RCLCPP_INFO(this->get_logger(), "Path recording stopped. %zu poses recorded. CSV: %s", record_path_.poses.size(), csv_path.c_str());
            }
            else
            {
                res->success = false;
                res->message = "Failed to open CSV file: " + csv_path;
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
            }
        }
        else
        {
            res->success = false;
            res->message = "No publisher found.";
        }
    }

private:
    bool relocalization_enabled_=true;
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
    bool record_republish_saved_ = true;
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

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (!localization_mode && pcd_save_en && !pcl_wait_pub->empty())
    {
        std::string message;
        const bool saved = save_to_pcd(message);
        std::cout << message << std::endl;
        if (!saved) return 1;
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
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

    return 0;
}
