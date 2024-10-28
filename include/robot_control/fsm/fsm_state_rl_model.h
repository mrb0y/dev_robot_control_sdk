/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include "robot_control/fsm/fsm_state.h"
#include <iostream>
#include <MNN/Interpreter.hpp>
#include <eigen3/Eigen/Dense>

// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

/*!
 * Take the quat rotate inverse
 */
template <typename T, typename T2>
Vec3<typename T2::Scalar> quat_rotate_inverse(const Eigen::MatrixBase<T> &q,
                                              const Eigen::MatrixBase<T2> &v)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                  "Must have 4x1 q");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                  "Must have 3x1 v");
    typename T::Scalar q_w = q[0];
    Vec3<typename T2::Scalar> q_vec(q[1], q[2], q[3]);
    Vec3<typename T2::Scalar> a = v * (2.0 * q_w * q_w - 1.0);
    Vec3<typename T2::Scalar> b = q_vec.cross(v) * q_w * 2.0;
    Vec3<typename T2::Scalar> c = q_vec * (q_vec.transpose() * v) * 2.0;
    return a - b + c;
}

/**
 * RLModel state
 */
class FSMStateRLModel : public FSMState
{
public:
    explicit FSMStateRLModel(RobotData *robot_data) : FSMState(robot_data)
    {
        state_name_ = FSMStateName::RL_MODEL;

        // Load config
        const YAML::Node &rl_model = robot_data_->config["rl_model"];

        input_dof_mapping_.resize(num_joints_, 0);
        output_dof_mapping_.resize(num_joints_, 0);
        action_.resize(num_joints_, 0.f);
        desired_pos_.resize(num_joints_, 0.f);
        last_desired_pos_.resize(num_joints_, 0.f);

        for (int j = 0; j < num_joints_; ++j)
        {
            target_pose_.push_back(rl_model["pose"][j].as<float>());
            kp_.push_back(rl_model["kp"][j].as<float>());
            kd_.push_back(rl_model["kd"][j].as<float>());
            output_dof_mapping_[j] = rl_model["output_dof_mapping"][j].as<int>();
            input_dof_mapping_[rl_model["output_dof_mapping"][j].as<int>()] = j;
        }
        
        decimation_ = rl_model["decimation"].as<int>();
        scale_.action = rl_model["scale"]["action"].as<float>();
        scale_.dof_pos = rl_model["scale"]["dof_pos"].as<float>();
        scale_.dof_vel = rl_model["scale"]["dof_vel"].as<float>();

        std::string model_path = rl_model["model_path"].as<std::string>();

        // Load model
        action_interpreter_ = std::unique_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(model_path.c_str()));

        MNN::ScheduleConfig config;
        config.numThread = 1;
        config.type = MNN_FORWARD_CPU;
        MNN::BackendConfig backend_config;
        backend_config.precision = MNN::BackendConfig::PrecisionMode::Precision_Low;
        config.backendConfig = &backend_config;
        action_session_ = action_interpreter_->createSession(config);
        action_input_ = action_interpreter_->getSessionInput(action_session_, nullptr);

        obs_history_.resize(obs_history_length_ * obs_step_size_, 0.f);
    }
    ~FSMStateRLModel()
    {
        action_interpreter_->releaseModel();
        action_interpreter_->releaseSession(action_session_);
    }

    // Enter this state
    void on_enter()
    {
        std::cout << "[FSMStateRLModel] enter" << std::endl;
        iter_ = 0;

        // Initialize desired joints position 
        for (int j = 0; j < num_joints_; ++j)
        {
            desired_pos_[j] = robot_data_->robot_state.joints[j].pos;
        }
    }

    // Update robot command in the current state
    void run()
    {
        // Do model inference every decimation
        if (iter_ % decimation_ == 0)
        {
            // Scroll observation history
            memmove(obs_history_.data() + obs_step_size_, obs_history_.data(), (obs_history_.size() - obs_step_size_) * sizeof(float));

            // Update observation data
            Quat<float> quat;
            quat << robot_data_->robot_state.base.quaternion_w,
                robot_data_->robot_state.base.quaternion_x,
                robot_data_->robot_state.base.quaternion_y,
                robot_data_->robot_state.base.quaternion_z;
            Vec3<float> projected_gravity = quat_rotate_inverse(quat, Vec3<float>(0., 0., -1));

            int i = 0;
            obs_history_[i++] = robot_data_->remote_command.vx;
            obs_history_[i++] = robot_data_->remote_command.vy;
            obs_history_[i++] = robot_data_->remote_command.wz;
            obs_history_[i++] = robot_data_->robot_state.base.gyro_x;
            obs_history_[i++] = robot_data_->robot_state.base.gyro_y;
            obs_history_[i++] = robot_data_->robot_state.base.gyro_z;
            obs_history_[i++] = quat[0];
            obs_history_[i++] = quat[1];
            obs_history_[i++] = quat[2];
            obs_history_[i++] = quat[3];
            obs_history_[i++] = projected_gravity[0];
            obs_history_[i++] = projected_gravity[1];
            obs_history_[i++] = projected_gravity[2];
            for (int j = 0; j < num_joints_; ++j)
            {
                obs_history_[i++] = (robot_data_->robot_state.joints[j].pos - target_pose_[j]) * scale_.dof_pos;
            }
            for (int j = 0; j < num_joints_; ++j)
            {
                obs_history_[i++] = robot_data_->robot_state.joints[j].vel * scale_.dof_vel;
            }
            for (int j = 0; j < num_joints_; ++j)
            {
                obs_history_[i++] = action_[j];
            }

            assert(obs_step_size_ == i);

            // memcpy to mnn input
            memcpy(action_input_->host<float>(), obs_history_.data(), obs_history_.size() * sizeof(float));

            // Inference
            action_interpreter_->runSession(action_session_);
            MNN::Tensor *action_space = action_interpreter_->getSessionOutput(action_session_, "actions");
            const float *action_host = action_space->host<float>();

            // Extract actions
            for (int j = 0; j < num_joints_; ++j)
            {
                last_desired_pos_[j] = desired_pos_[j];
                action_[j] = action_host[3 + j]; // The first 3 numbers are vel
                desired_pos_[j] = target_pose_[j] + scale_.action * action_[output_dof_mapping_[j]];
            }

            // Print velocity
            if (iter_ % 400 == 0)
            {
                printf("Velocity command: (%.3f %.3f %.3f), state: (%.3f %.3f %.3f)\n", 
                    robot_data_->remote_command.vx, 
                    robot_data_->remote_command.vy, 
                    robot_data_->remote_command.wz,
                    action_host[0], 
                    action_host[1], 
                    action_host[2]);
            }
        }

        // Smooth filter
        float filter_ratio = ((iter_ % decimation_) + 1.0f) / decimation_;

        // Apply action
        for (int j = 0; j < num_joints_; ++j)
        {
            robot_data_->robot_command.joints[j].kp = kp_[j];
            robot_data_->robot_command.joints[j].kd = kd_[j];
            robot_data_->robot_command.joints[j].pos = desired_pos_[j] * filter_ratio + last_desired_pos_[j] * (1 - filter_ratio);
            robot_data_->robot_command.joints[j].vel = 0;
            robot_data_->robot_command.joints[j].tau = 0;
        }

        iter_++;
    }

    // Check transition to desired state from user command
    FSMStateName check_transition()
    {
        FSMStateName next_name = state_name_;

        if (!check_safety())
        {
            return FSMStateName::SOFT_STOP;
        }

        switch (robot_data_->remote_command.mode[0])
        {
            case 0:
                next_name = FSMStateName::PASSIVE;
                break;
            case 1:
                next_name = FSMStateName::LIE_DOWN;
                break;
            case 2:
                next_name = FSMStateName::STAND_UP;
                break;
            case 3:
                next_name = FSMStateName::RL_MODEL;
                break;
            case 4:
                next_name = FSMStateName::SOFT_STOP;
                break;
            default:
                break;
        }

        return next_name;
    }

    // Check whether need stop for safety
    bool check_safety(bool silence = false)
    {
        // Up side down
        Quat<float> quat;
        quat << robot_data_->robot_state.base.quaternion_w,
            robot_data_->robot_state.base.quaternion_x,
            robot_data_->robot_state.base.quaternion_y,
            robot_data_->robot_state.base.quaternion_z;
        Vec3<float> projected_gravity = quat_rotate_inverse(quat, Vec3<float>(0., 0., -1));
        if (projected_gravity[2] >= -0.1f)
        {
            return false;
        }

        return FSMState::check_safety();
    }

    // Clean up
    void on_exit()
    {
        std::cout << "[FSMStateRLModel] exit" << std::endl;
    }

protected:
    // Config
    std::vector<float> target_pose_;
    std::vector<float> kp_;
    std::vector<float> kd_;
    std::vector<float> input_dof_mapping_;
    std::vector<float> output_dof_mapping_;
    int decimation_ = 4;

    struct Scale
    {
        float action = 1.0f;
        float dof_pos = 1.0f;
        float dof_vel = 1.0f;
    };
    Scale scale_;

    static const int obs_step_size_ = 49;
    static const int obs_history_length_ = 10;

    // Model interpreter
    std::unique_ptr<MNN::Interpreter> action_interpreter_;
    MNN::Session *action_session_ = nullptr;
    MNN::Tensor *action_input_ = nullptr;

    // Model input and output
    std::vector<float> obs_history_;
    std::vector<float> action_;
    std::vector<float> desired_pos_;
    std::vector<float> last_desired_pos_;

    // Iteration count
    int iter_ = 0;
};
