/*============================= RL ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
    : FSMState(data, FSMStateName::RL, "rl"),
      input_0(new float[n_prio]),
      input_1(new float[n_prio * history_len]),
      output(new float[DOF]),
      output_last(new float[DOF]),
      input_1_temp(new float[temp_history_len_all])
{
  cuda_test_ = std::make_shared<CudaTest>("/home/hilabldc/tita_rl/logs/diablo_pluspro/exported/policies/model_gn.engine");
  std::cout << "cuda init :" << cuda_test_->get_cuda_init() << std::endl;
}

void FSMState_RL::enter()
{
  _data->state_command->firstRun = true;

  if (DOF / 2 == 4)
  {
    for (int i = 0; i < 2; i++)
    {
      wheel_init_pos_abs_[i] = _data->low_state->q[4 * i + 3];
      desired_pos[4 * i + 3] = 0;
      desired_pos[4 * i] = _data->low_state->q[4 * i];
      desired_pos[4 * i + 1] = _data->low_state->q[4 * i + 1];
      desired_pos[4 * i + 2] = _data->low_state->q[4 * i + 2];
    }
    for (int i = 0; i < 4; i++)
    {
      obs_.dof_pos[i + 4] = _data->low_state->q[i];
      obs_.dof_vel[i + 4] = _data->low_state->dq[i];
      obs_.dof_pos[i] = _data->low_state->q[i + 4];
      obs_.dof_vel[i] = _data->low_state->dq[i + 4];
    }
  }
  else if (DOF / 2 == 3)
  {
    for (int i = 0; i < 2; i++)
    {
      wheel_init_pos_abs_[i] = _data->low_state->q[3 * i + 2];
      desired_pos[3 * i + 2] = 0;
      desired_pos[3 * i] = _data->low_state->q[3 * i];
      desired_pos[3 * i + 1] = _data->low_state->q[3 * i + 1];
    }
    for (int i = 0; i < 3; i++)
    {
      obs_.dof_pos[i + 3] = _data->low_state->q[i];
      obs_.dof_vel[i + 3] = _data->low_state->dq[i];
      obs_.dof_pos[i] = _data->low_state->q[i + 3];
      obs_.dof_vel[i] = _data->low_state->dq[i + 3];
    }
  }
  else
  {
    std::cout << "dof not init!!!!ERROR!!!!! :" << std::endl;
  }

  params_.action_scale = 0.5;
  params_.action_scale_vel = 10.0;
  params_.hip_scale_reduction = 0.5;
  params_.num_of_dofs = DOF;
  params_.lin_vel_scale = 2.0;
  params_.ang_vel_scale = 0.25;
  params_.dof_pos_scale = 1.0;
  params_.dof_vel_scale = 0.05;

  params_.commands_scale[0] = params_.lin_vel_scale;
  params_.commands_scale[1] = params_.lin_vel_scale;
  params_.commands_scale[2] = params_.ang_vel_scale;

  for (int i = 0; i < DOF; i++)
  {
    params_.default_dof_pos[i] = default_dof_pos[i];
  }

  x_vel_cmd_ = 0.;
  yaw_vel_cmd_ = 0.;

  for (int i = 0; i < temp_history_len_all; i++)
    input_1.get()[i] = 0;
  for (int i = 0; i < DOF; i++)
    output_last.get()[i] = 0;

  obs_.forward_vec[0] = 1.0;
  obs_.forward_vec[1] = 0.0;
  obs_.forward_vec[2] = 0.0;

  for (int j = 0; j < DOF; j++)
  {
    action[j] = obs_.dof_pos[j];
  }
  a_l.setZero();

  for (int i = 0; i < history_len; i++)
  {
    // torch::Tensor obs_tensor = GetObs();
    // // append obs to obs buffer
    // obs_buf = torch::cat({obs_buf.index({Slice(1,None),Slice()}),obs_tensor},0);
    _GetObs();

    for (int i = 0; i < temp_history_len_all; i++)
      input_1_temp.get()[i] = input_1.get()[i + n_prio];

    for (int i = 0; i < temp_history_len_all; i++)
      input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < n_prio; i++)
      input_1.get()[i + temp_history_len_all] = input_0.get()[i];
  }
  std::cout << "init finised predict" << std::endl;

  for (int i = 0; i < history_len; i++)
  {
    _Forward();
  }

  threadRunning = true;
  if (thread_first_)
  {
    forward_thread = std::thread(&FSMState_RL::_Run_Forward, this); // 应该是这个线程一直在跑然后获得desired_pos
    thread_first_ = false;
  }
  stop_update_ = false;
}

void FSMState_RL::run()
{
  // _data->state_command->clear();
  // _data->low_cmd->zero();
  x_vel_cmd_ = _data->state_command->rc_data_->twist_linear[point::X];
  yaw_vel_cmd_ = _data->state_command->rc_data_->twist_angular[point::Z];
  // _data->state_command->rc_data_->twist_angular[point::Z]
  _data->low_cmd->qd.setZero();
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd.setZero();
  for (int i = 0; i < DOF; i++)
  {
    if (i % (DOF / 2) == (DOF / 2 - 1)) // 轮子
    {
      _data->low_cmd->tau_cmd[i] = 10 * desired_pos[i] - 0.5 * _data->low_state->dq[i];
    }
    else // 关节？
    {
      _data->low_cmd->tau_cmd[i] = 40 * (desired_pos[i] - _data->low_state->q[i]) + 1.0 * (0 - _data->low_state->dq[i]);
    }
  }
}

void FSMState_RL::exit()
{
  stop_update_ = true;
}

FSMStateName FSMState_RL::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RECOVERY_STAND:
    this->_nextStateName = FSMStateName::RECOVERY_STAND;
    break;

  case FSMStateName::RL: // normal c
    break;

  case FSMStateName::TRANSFORM_DOWN:
    this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
    break;

  case FSMStateName::PASSIVE: // normal c
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

void FSMState_RL::_GetObs()
{
  // omegawb = state_estimate_->omegaBody;
  // qwb = state_estimate_->orientation;
  // pwb = state_estimate_->position;
  // vwb = state_estimate_->vWorld;
  std::vector<float> obs_tmp;
  // compute gravity
  Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
  Mat3<double> _G2B_RotMat = this->_data->state_estimator->getResult().rBody.transpose();

  Vec3<double> base_ang_vel = a_l;
  a_l = 0.97 * this->_data->state_estimator->getResult().omegaBody + 0.03 * a_l;
  // a_l = 1.0 * this->_data->state_estimator->getResult().omegaBody;
  Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);
  Vec3<double> projected_forward = _G2B_RotMat * Vec3<double>(1.0, 0.0, 0.0);
  // gravity
  // _gxFilter->addValue(angvel(0,0));
  // _gyFilter->addValue(angvel(1,0));
  // _gzFilter->addValue(angvel(2,0));
  //
  obs_tmp.push_back(base_ang_vel(0) * params_.ang_vel_scale);
  obs_tmp.push_back(base_ang_vel(1) * params_.ang_vel_scale);
  obs_tmp.push_back(base_ang_vel(2) * params_.ang_vel_scale);

  for (int i = 0; i < 3; ++i)
  {
    obs_tmp.push_back(projected_gravity(i));
  }

  float command_x = x_vel_cmd_ * params_.lin_vel_scale;
  float command_yaw = yaw_vel_cmd_ * params_.ang_vel_scale;

  obs_tmp.push_back(command_x);
  obs_tmp.push_back(0.0);
  obs_tmp.push_back(command_yaw);
  // std::cout << "angle: " << command_yaw << " ang_vel_now " << base_ang_vel(2) << std::endl;

  // pos
  for (int i = 0; i < DOF; ++i)
  {
    float pos = (this->obs_.dof_pos[i] - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
    obs_tmp.push_back(pos);
  }
  // vel
  for (int i = 0; i < DOF; ++i)
  {
    float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
    obs_tmp.push_back(vel);
  }

  // last action
  // float index[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
  for (int i = 0; i < DOF; ++i)
  {
    obs_tmp.push_back(output_last.get()[i]);
  }

  for (int i = 0; i < n_prio; i++)
  {
    input_0.get()[i] = obs_tmp[i];
  }
}

void FSMState_RL::_Forward()
{
  _GetObs();
  cuda_test_->do_inference(input_0.get(), input_1.get(), output.get());

  for (int i = 0; i < temp_history_len_all; i++)
    input_1_temp.get()[i] = input_1.get()[i + n_prio];

  for (int i = 0; i < temp_history_len_all; i++)
    input_1.get()[i] = input_1_temp.get()[i];

  for (int i = 0; i < n_prio; i++)
    input_1.get()[i + temp_history_len_all] = input_0.get()[i];

  for (int i = 0; i < DOF; i++)
    output_last.get()[i] = output.get()[i];
}

void FSMState_RL::_Run_Forward()
{
  while (threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!stop_update_)
    {
      for (int i = 0; i < DOF / 2; i++)
      {
        obs_.dof_pos[i + DOF / 2] = _data->low_state->q[i];
        obs_.dof_vel[i + DOF / 2] = _data->low_state->dq[i];
        obs_.dof_pos[i] = _data->low_state->q[i + DOF / 2];
        obs_.dof_vel[i] = _data->low_state->dq[i + DOF / 2];
      }
      obs_.dof_pos[DOF / 2 - 1] = 0; // 把两个轮子的位置的观察量都置为0
      obs_.dof_pos[DOF - 1] = 0;

      _Forward();

      for (int j = 0; j < DOF; j++)
      {
        action[j] = output.get()[j] * params_.action_scale + params_.default_dof_pos[j];
      }
      action[0] *= params_.hip_scale_reduction;
      action[3] *= params_.hip_scale_reduction;
      // action[DOF / 2 - 1] = output.get()[DOF / 2 - 1] * params_.action_scale_vel;
      // action[DOF - 1] = output.get()[DOF - 1] * params_.action_scale_vel;
      // 换位？左腿换右腿(因为RL里面是反的)
      for (int i = 0; i < DOF / 2; i++)
      {
        desired_pos[i + (DOF / 2)] = action[i];
        desired_pos[i] = action[i + (DOF / 2)];
        // std::cerr << "desired_pos" << i << ":" << desired_pos[i] << std::endl;
      }
      // std::cerr << "des_pos0: " << desired_pos[0]
      //           << " des_pos1: " << desired_pos[1]
      //           << " des_pos2 " << desired_pos[2]
      //           << " des_pos3: " << desired_pos[3]
      //           << " des_pos4: " << desired_pos[4]
      //           << " des_pos5: " << desired_pos[5] << std::endl;
    }

    absoluteWait(_start_time, (long long)(0.01 * 1000000));
  }
  threadRunning = false;
}