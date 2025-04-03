#ifndef FSMSTATE_RL_H
#define FSMSTATE_RL_H

#include <thread>
#include "FSMState.h"
#include "tensorrt_cuda/tensor_cuda_test.hpp"

#define DOF 6
/**
 *
 */

struct ModelParams
{
  float damping;
  float stiffness;
  float action_scale;
  float hip_scale_reduction;
  float num_of_dofs;
  float lin_vel_scale;
  float ang_vel_scale;
  float dof_pos_scale;
  float dof_vel_scale;
  float clip_obs;
  float clip_actions;
  float torque_limits[DOF];
  float d_gains[DOF];
  float p_gains[DOF];
  float commands_scale[3];
  float default_dof_pos[DOF];
};

struct Observations
{
  float lin_vel[3];
  float ang_vel[3];
  float gravity_vec[3];
  float forward_vec[3];
  float commands[3];
  float base_quat[4];
  float dof_pos[DOF];
  float dof_vel[DOF];
  float actions[DOF];
};

class FSMState_RL : public FSMState
{
public:
  FSMState_RL(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_RL() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSMStateName checkTransition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

private:
  // Keep track of the control iterations
  float wheel_init_pos_abs_[4];
  float x_vel_cmd_;
  float pitch_cmd_;

  // 从RL里面拿到的数据
  int n_prio = 27;
  int history_len = 10;
  int n_priv_latent = 30;
  int temp_history_len_all = 270 - 27;

private:
  ModelParams params_;
  Observations obs_;

  void _Forward();
  void _Run_Forward();

  std::shared_ptr<CudaTest> cuda_test_;

  std::thread forward_thread;
  bool threadRunning;
  // float desired_pos[DOF] = {0.75, -1.5, 0, 0.75, -1.5, 0};
  // float default_dof_pos[DOF] = {0.8, -1.5, 0, 0.8, -1.5, 0};
  // float desired_pos[DOF] = {-0.184481302, -1.194677873, 0.0, -0.184481302, -1.194677873, 0.0};
  // float default_dof_pos[DOF] = {-0.184481302, -1.194677873, 0.0, -0.184481302, -1.194677873, 0.0};
  float desired_pos[DOF] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float default_dof_pos[DOF] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // AttitudeData* attitude_ = nullptr;
  // LegData* legdata_ = nullptr;
  // LinearKFPositionVelocityEstimator<float>* posvelest_ = nullptr;

  std::shared_ptr<float[]> input_0;
  std::shared_ptr<float[]> input_1;
  std::shared_ptr<float[]> output;

  std::shared_ptr<float[]> output_last;
  std::shared_ptr<float[]> input_1_temp;

  int history_length = 10;

  void _GetObs();
  Vec3<double> a_l;

  float action[DOF];

  bool stop_update_ = false;
  bool thread_first_ = true;
};

#endif // FSMSTATE_RL_H
