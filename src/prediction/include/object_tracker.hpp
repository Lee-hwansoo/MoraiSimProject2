#ifndef OBJECT_TRACKER_HPP
#define OBJECT_TRACKER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "state_models.hpp"
#include "state_filters.hpp"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <prediction/TrackedPoint.h>
#include <prediction/PredictedObjectPath.h>
#include <prediction/PredictedObjectPathList.h>
#include <prediction/TrackedObjectPose.h>
#include <prediction/TrackedObjectPoseList.h>

class DynamicObstacleTracker {
public:
    DynamicObstacleTracker(double dt = 0.1, double T = 1);
    ~DynamicObstacleTracker() = default;

    void initialize(const Eigen::RowVectorXd& data);
    void update(const Eigen::RowVectorXd& data);
    Eigen::MatrixXd pred();

    double dt;
    double T;
    std::vector<std::shared_ptr<StateModel>> models;
    std::vector<std::shared_ptr<ExtendedKalmanFilter>> filters;
    std::unique_ptr<IMMFilter> IMM;
    std::vector<Eigen::VectorXd> Q_list;
    std::vector<Eigen::VectorXd> R_list;
    Eigen::RowVectorXd mu;
    Eigen::MatrixXd mat_trans;
    std::deque<Eigen::RowVectorXd> X;
    std::deque<Eigen::RowVectorXd> MM;
private:

};

class TrackedObject {
public:
    TrackedObject(int obj_id, std::shared_ptr<DynamicObstacleTracker> tracker, ros::Time last_update_time);
    ~TrackedObject() = default;

    TrackedObject(const TrackedObject& other) = default; // 복사 생성자
    TrackedObject& operator=(const TrackedObject& other) = default; // 복사 할당 연산자

    int obj_id;
    std::shared_ptr<DynamicObstacleTracker> tracker;
    ros::Time last_update_time;
private:

};

class MultiDynamicObstacleTracker {
public:
    MultiDynamicObstacleTracker(double dt = 0.1, double T = 1, double timeout = 1.0);
    ~MultiDynamicObstacleTracker() = default;

    void initialize(int obj_id, const Eigen::RowVectorXd& data);
    void addTracker(int obj_id);
    void update(int obj_id, const Eigen::RowVectorXd& data);
    void clean();
    void deleteObject(int obj_id);
    std::vector<int> getDeletedIds();
    std::unordered_map<int, Eigen::MatrixXd> pred();

    std::vector<std::shared_ptr<TrackedObject>> objects;
    std::unordered_map<int, int> obj_id_map; // obj_id를 인덱스로 매핑
    double dt;
    double T;
    double timeout; // 객체의 타임아웃 시간 (초)
    std::unordered_set<int> deleted_ids; // 삭제된 객체 ID를 추적하기 위한 집합 추가
private:

};

class DynamicObstacleTrackerNode {
public:
    DynamicObstacleTrackerNode();
    ~DynamicObstacleTrackerNode() = default;

    void run();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void objectInfoCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg);
    Eigen::RowVectorXd dataPreprocessing(const morai_msgs::ObjectStatus& obstacle);
    void publishObjectPose();
    void publishObjectPath();
    void publishDeletedIds();

    ros::NodeHandle nh;
    ros::Subscriber object_info_sub;
    ros::Subscriber odom_sub;
    ros::Publisher object_pose_pub;
    ros::Publisher object_path_pub;
    ros::Publisher deleted_id_pub;
    MultiDynamicObstacleTracker multi_tracker;

    ros::Rate rate;
    bool is_odom;
    bool is_object;
    nav_msgs::Odometry odom_data;
    morai_msgs::ObjectStatusList object_data;
private:

};

#endif // OBJECT_TRACKER_HPP
