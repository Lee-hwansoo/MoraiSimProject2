#include "state_models.hpp"
#include "state_filters.hpp"
#include "object_tracker.hpp"
#include <iostream>
#include <stdexcept>
#include <ros/ros.h>
#include <Eigen/Dense>

void testCA() {
    CA ca_model(0.1);

    Eigen::RowVectorXd x(5);
    x << 0, 0, 10, 1, 0.1; // Initial state: [x, y, v, a, theta]

    std::cout << "Initial state (CA): " << x << std::endl;

    for (int i = 0; i < 3; ++i) {
        x = ca_model.step(x);
        std::cout << "State after step " << i + 1 << " (CA): " << x << std::endl;

        Eigen::RowVectorXd z = ca_model.H(x);
        std::cout << "Measurement after step " << i + 1 << " (CA): " << z << std::endl;

        Eigen::MatrixXd JA = ca_model.JA(x);
        std::cout << "Jacobian of state transition after step " << i + 1 << " (CA): \n" << JA << std::endl;

        Eigen::MatrixXd JH = ca_model.JH(x);
        std::cout << "Jacobian of measurement function after step " << i + 1 << " (CA): \n" << JH << std::endl;
    }
}

void testCTRA() {
    CTRA ctra_model(0.1);

    Eigen::RowVectorXd x(6);
    x << 0, 0, 10, 1, 0.1, 0.01; // Initial state: [x, y, v, a, theta, theta_rate]

    std::cout << "Initial state (CTRA): " << x << std::endl;

    for (int i = 0; i < 3; ++i) {
        x = ctra_model.step(x);
        std::cout << "State after step " << i + 1 << " (CTRA): " << x << std::endl;

        Eigen::RowVectorXd z = ctra_model.H(x);
        std::cout << "Measurement after step " << i + 1 << " (CTRA): " << z << std::endl;

        Eigen::MatrixXd JA = ctra_model.JA(x);
        std::cout << "Jacobian of state transition after step " << i + 1 << " (CTRA): \n" << JA << std::endl;

        Eigen::MatrixXd JH = ctra_model.JH(x);
        std::cout << "Jacobian of measurement function after step " << i + 1 << " (CTRA): \n" << JH << std::endl;
    }
}

void testExtendedKalmanFilter() {
    CA ca_model(0.1);
    ExtendedKalmanFilter ekf(5, 4, std::make_shared<CA>(ca_model));

    Eigen::VectorXd Q(5);
    Q << 0.1, 0.1, 0.1, 0.1, 0.1;
    Eigen::VectorXd R(4);
    R << 0.05, 0.05, 0.05, 0.05;

    ekf.Q = Q.asDiagonal();
    ekf.R = R.asDiagonal();

    // Initial state
    Eigen::RowVectorXd x0(5);
    x0 << 0, 0, 10, 1, 0.1;
    ekf.x = x0;
    std::cout << "Initial state (EKF): " << ekf.x << std::endl;

    std::vector<Eigen::RowVectorXd> measurements = {
        (Eigen::RowVectorXd(4) << 0.999979, 0.100333, 10.1, 0.1).finished(),
        (Eigen::RowVectorXd(4) << 2.000123, 0.201234, 10.2, 0.1).finished(),
        (Eigen::RowVectorXd(4) << 3.001234, 0.302345, 10.3, 0.1).finished()
    };

    for (const auto& z : measurements) {
        ekf.prediction();
        std::cout << "State after prediction (EKF): " << ekf.x << std::endl;

        ekf.correction(z);
        std::cout << "State after correction (EKF): " << ekf.x << "\n" << std::endl;
    }

    double T = 1.0;
    Eigen::MatrixXd predicted_states = ekf.pred(T);
    std::cout << "\nPredicted states:\n" << predicted_states << std::endl;
}

void testIMMFilter() {
    CA ca_model(0.1);
    CTRA ctra_model(0.1);

    ExtendedKalmanFilter ekf_ca(5, 4, std::make_shared<CA>(ca_model));
    ExtendedKalmanFilter ekf_ctra(6, 4, std::make_shared<CTRA>(ctra_model));

    Eigen::VectorXd Q_ca(5);
    Q_ca << 0.1, 0.1, 0.1, 0.1, 0.1;
    Eigen::VectorXd R_ca(4);
    R_ca << 0.05, 0.05, 0.05, 0.05;
    Eigen::VectorXd Q_ctra(6);
    Q_ctra << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Eigen::VectorXd R_ctra(4);
    R_ctra << 0.05, 0.05, 0.05, 0.05;

    ekf_ca.Q = Q_ca.asDiagonal();
    ekf_ca.R = R_ca.asDiagonal();
    ekf_ctra.Q = Q_ctra.asDiagonal();
    ekf_ctra.R = R_ctra.asDiagonal();

    Eigen::RowVectorXd x0(5);
    x0 << 0, 0, 10, 1, 0.1;
    ekf_ca.x = x0;

    Eigen::RowVectorXd x1(6);
    x1 << 0, 0, 10, 1, 0.1, 0.01;
    ekf_ctra.x = x1;

    std::vector<std::shared_ptr<ExtendedKalmanFilter>> filters = {std::make_shared<ExtendedKalmanFilter>(ekf_ca), std::make_shared<ExtendedKalmanFilter>(ekf_ctra)};
    Eigen::RowVectorXd mu(2);
    mu << 0.5, 0.5;
    Eigen::MatrixXd M(2, 2);
    M << 0.97, 0.03,
        0.03, 0.97;
    IMMFilter imm(filters, mu, M);
    std::cout << "Initial state (IMM): " << imm.x << std::endl;

    std::vector<Eigen::RowVectorXd> measurements = {
        (Eigen::RowVectorXd(4) << 0.999979, 0.100333, 10.1, 0.1).finished(),
        (Eigen::RowVectorXd(4) << 2.000123, 0.201234, 10.2, 0.1).finished(),
        (Eigen::RowVectorXd(4) << 3.001234, 0.302345, 10.3, 0.1).finished()
    };

    for (const auto& z : measurements) {
        imm.prediction();
        std::cout << "State after prediction (IMM): " << imm.x << std::endl;

        imm.merging(z);
        std::cout << "State after merging (IMM): " << imm.x << "\n" << std::endl;
    }

    double T = 1.0;
    Eigen::MatrixXd predicted_states = imm.pred(T);
    std::cout << "\nPredicted states:\n" << predicted_states << std::endl;
}

void test_DynamicObstacleTracker() {
    DynamicObstacleTracker tracker(0.1, 1.0);

    // 초기 데이터 설정
    Eigen::RowVectorXd data(4);
    data << 1.0, 2.0, 0.5, 3.0; // [x, y, h, v]

    // initialize 테스트
    tracker.initialize(data);
    std::cout << "Initial state of filter 0: " << tracker.filters[0]->x << std::endl;
    std::cout << "Initial state of filter 1: " << tracker.filters[1]->x << std::endl;

    // update 테스트
    Eigen::RowVectorXd new_data(4);
    new_data << 1.1, 2.1, 0.55, 3.1; // [x, y, h, v]

    tracker.update(new_data);
    std::cout << "State of IMM after update: " << tracker.IMM->x << std::endl;
    std::cout << "Mode probabilities after update: " << tracker.IMM->mu << std::endl;

    // prediction 테스트
    Eigen::MatrixXd traj = tracker.pred();
    std::cout << "Predicted trajectory: \n" << traj << std::endl;
}

void test_TrackedObject() {
    // DynamicObstacleTracker 객체 생성
    auto tracker = std::make_shared<DynamicObstacleTracker>(0.1, 1.0);

    // 객체 ID와 마지막 업데이트 시간을 설정
    int obj_id = 1;
    ros::Time last_update_time = ros::Time::now();

    // 초기 데이터 설정
    Eigen::RowVectorXd data(4);
    data << 1.0, 2.0, 0.5, 3.0; // [x, y, h, v]
    tracker->initialize(data);

    // TrackedObject 객체 생성
    TrackedObject tracked_obj(obj_id, tracker, last_update_time);

    // TrackedObject 멤버 변수 출력
    std::cout << "TrackedObject ID: " << tracked_obj.obj_id << std::endl;
    std::cout << "Last update time: " << tracked_obj.last_update_time << std::endl;
    std::cout << "Initial state of filter 0: " << tracked_obj.tracker->filters[0]->x << std::endl;
    std::cout << "Initial state of filter 1: " << tracked_obj.tracker->filters[1]->x << std::endl;
}

void test_MultiDynamicObstacleTracker() {
    MultiDynamicObstacleTracker multi_tracker(0.1, 1.0, 2.0);

    // Object ID와 데이터 설정
    std::vector<int> obj_ids = {1, 2, 3};
    std::vector<Eigen::RowVectorXd> init_data = {
        (Eigen::RowVectorXd(4) << 1.0, 2.0, 0.5, 3.0).finished(), // [x, y, h, v]
        (Eigen::RowVectorXd(4) << 2.0, 3.0, 0.6, 2.5).finished(), // [x, y, h, v]
        (Eigen::RowVectorXd(4) << 3.0, 4.0, 0.7, 4.0).finished()  // [x, y, h, v]
    };

    // initialize 테스트
    for (size_t i = 0; i < obj_ids.size(); ++i) {
        multi_tracker.initialize(obj_ids[i], init_data[i]);
        std::cout << "Initialized object ID: " << obj_ids[i] << std::endl;
    }

    // update 테스트
    std::vector<Eigen::RowVectorXd> new_data = {
        (Eigen::RowVectorXd(4) << 1.1, 2.1, 0.55, 3.1).finished(), // [x, y, h, v]
        (Eigen::RowVectorXd(4) << 2.1, 3.1, 0.65, 2.6).finished(), // [x, y, h, v]
        (Eigen::RowVectorXd(4) << 3.1, 4.1, 0.75, 4.1).finished()  // [x, y, h, v]
    };

    for (size_t i = 0; i < obj_ids.size(); ++i) {
        multi_tracker.update(obj_ids[i], new_data[i]);
        std::cout << "Updated object ID: " << obj_ids[i] << std::endl;
    }

    // prediction 테스트
    auto trajs = multi_tracker.pred();
    for (int obj_id : obj_ids) {
        if (trajs.find(obj_id) != trajs.end()) {
            std::cout << "Predicted trajectory for object ID " << obj_id << ": \n" << trajs[obj_id] << std::endl;
        }
    }

    // clean 테스트
    std::cout << "Waiting for 3 seconds to test clean..." << std::endl;
    ros::Duration(3.0).sleep(); // 3초 대기

    multi_tracker.clean();
    auto deleted_ids = multi_tracker.getDeletedIds();
    for (const auto& id : deleted_ids) {
        std::cout << "Deleted object ID: " << id << std::endl;
    }
}

void test_MultiDynamicObstacleTrackerNode() {
    ROS_INFO("Start");

    DynamicObstacleTrackerNode tracker;
    tracker.run();
}


int main(int argc, char** argv) {
    // std::cout << "Testing CA Model..." << std::endl;
    // testCA();

    // std::cout << "\nTesting CTRA Model..." << std::endl;
    // testCTRA();

    // std::cout << "\nTesting Extended_KalmanFilter..." << std::endl;
    // testExtendedKalmanFilter();

    // std::cout << "\nTesting IMMFilter..." << std::endl;
    // testIMMFilter();

    // std::cout << "\nTesting DynamicObstacleTracker..." << std::endl;
    // test_DynamicObstacleTracker();

    ros::init(argc, argv, "test_object_tracker");
    ros::NodeHandle nh;

    // std::cout << "\nTesting TrackedObject..." << std::endl;
    // test_TrackedObject();

    // std::cout << "\nTesting MultiDynamicObstacleTracker..." << std::endl;
    // test_MultiDynamicObstacleTracker();

    std::cout << "\nTesting MultiDynamicObstacleTrackerNode..." << std::endl;
    test_MultiDynamicObstacleTrackerNode();
    return 0;
}
