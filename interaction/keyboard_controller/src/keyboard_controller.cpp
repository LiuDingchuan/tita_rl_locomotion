#include "keyboard_controller/keyboard_controller.hpp"

KeyboardControllerNode::KeyboardControllerNode(const rclcpp::NodeOptions &options) : Node("keyboard_controller", options)
{
  this->cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(tita_topic::manager_twist_command, 10);
  this->posestamped_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(tita_topic::manager_pose_command, 10);
  this->fsm_goal_publisher_ =
      this->create_publisher<std_msgs::msg::String>(tita_topic::manager_key_command, 10);
  realtime_cmd_vel_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
          cmd_vel_publisher_);
  realtime_posestamped_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
          posestamped_publisher_);
  realtime_fsm_goal_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(fsm_goal_publisher_);

  this->timer_ =
      this->create_wall_timer(10ms, std::bind(&KeyboardControllerNode::PubCmdVelCallBack, this));
  std::shared_ptr<std::thread> read_key_thread =
      std::make_shared<std::thread>(&KeyboardControllerNode::ReadKeyThread, this);
  (*read_key_thread).detach();
  pose_.pose.position.z = MIN_HEIGHT;
  for (size_t i = 0; i < 3; i++)
  {
    rpy_[i] = 0.0;
  }
  fsm_goal_.data = "idle";
  print_interface();
}
void KeyboardControllerNode::print_interface()
{
  system("clear");
  std::cout << R"(
  -------------------------------------------------------
                    State Machines
  0: passive 1: transform up 2 rl 3: transform down
  -------------------------------------------------------
  Moving Around    Postion Control    Orientation Control                                       
        w                 ↑                u i o                                               
      a s d             ←   →              j k l                                 
        x                 ↓                  ,                              
  -------------------------------------------------------
             Speed Scale        Pose Scale  
            + : increase       * : increase 
            - : decrease       / : decrease 
  -------------------------------------------------------
  )";

  std::cout << std::fixed << std::setprecision(2);
  std::cout << std::setw(32) << "state machine now: " << std::setw(6) << GREEN << fsm_goal_.data
            << RESET << std::endl;
  std::cout << std::setw(20) << "  speed scale:" << std::setw(6) << RED << speed_scale_ << RESET
            << "  pose scale:" << std::setw(6) << RED << pose_scale_ << RESET << std::endl
            << std::endl;
  std::cout << "  x vel:" << std::setw(6) << MAGENTA << twist_.linear.x << RESET
            << "  z vel:" << std::setw(6) << MAGENTA << twist_.angular.z << RESET << std::endl;

  std::cout << "  pitch:" << std::setw(6) << CYAN << rpy_[0] << RESET << "   roll:" << std::setw(6)
            << CYAN << rpy_[1] << RESET << "   yaw: " << std::setw(6) << CYAN << rpy_[2] << RESET
            << std::endl;

  std::cout << "  y:    " << std::setw(6) << YELLOW << pose_.pose.position.y << RESET
            << "   z:   " << std::setw(6) << YELLOW << pose_.pose.position.z << RESET
            << "   key_released :" << this->key_released << "     now_key:  " << this->now_key << std::endl;
}
/**
 * @description: 获取案件状态（非阻塞模式）
 * @param {int} &key_released
 * @return {*}
 */
int KeyboardControllerNode::get_key(bool &key_released)
{
  static int last_ch = -1;                                      // 缓存上一次的按键状态
  static auto last_time = std::chrono::steady_clock::now();     // 上一次检测时间
  static auto debounce_time = std::chrono::steady_clock::now(); // 消抖时间

  int ch = -1;
  struct termios oldt, newt;
  fd_set readfds;
  struct timeval timeout;

  // 获取终端设置
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // 设置非阻塞模式
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // 初始化文件描述符集合
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  // 设置超时时间为 0 秒（非阻塞）
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  // 检测是否有输入
  if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0)
  {
    // 检查是否满足消抖时间
    ch = getchar(); // 获取按键
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - debounce_time).count() > 300)
    {
      key_released = false; // 按键被按下
      last_ch = ch;         // 更新缓存的按键状态
      last_time = now;      // 更新检测时间
      debounce_time = now;  // 更新消抖时间
    }
    else
    {
      ch = -1; // 忽略按键（消抖）
    }
  }
  else
  {
    // 检测按键弹起的条件：当前没有输入，且距离上次按键按下超过一定时间
    auto now = std::chrono::steady_clock::now();
    if (last_ch != -1 && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() > 500)
    {
      key_released = true; // 按键被释放
      last_ch = -1;        // 重置缓存的按键状态
    }
    // else
    // {
    //   key_released = false; // 按键状态保持不变
    // }
    // std::cout << "now: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
    //           << " ms, last_time: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(last_time.time_since_epoch()).count()
    //           << " ms" << std::endl;
  }

  // 恢复终端设置
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  // std::cout << "key_released: " << key_released << " last_ch: " << last_ch << std::endl;
  return ch;
}

void KeyboardControllerNode::ReadKeyThread()
{
  while (rclcpp::ok())
  {
    this->now_key = get_key(this->key_released);

    if (!this->key_released)
    {
      switch (this->now_key)
      {
      case '0':
        fsm_goal_.data = "idle";
        break;
      case '1':
        fsm_goal_.data = "transform_up";
        break;
      case '2':
        fsm_goal_.data = "rl";
        break;
      case '3':
        fsm_goal_.data = "transform_down";
        break;
      case 'w':
        twist_.linear.x += STEP_ACCL_X * speed_scale_;
        break;
      case 's':
        twist_.linear.x -= STEP_ACCL_X * speed_scale_;
        break;
      case 'a':
        twist_.angular.z += STEP_ACCL_W * speed_scale_;
        break;
      case 'd':
        twist_.angular.z -= STEP_ACCL_W * speed_scale_;
        break;
      // case 's':
      //   twist_.linear.x = 0.0;
      //   twist_.angular.z = 0.0;
      //   break;
      case '+':
        speed_scale_ += 0.1;
        break;
      case '-':
        speed_scale_ -= 0.1;
        break;
      case '*':
        pose_scale_ += 0.1;
        break;
      case '/':
        pose_scale_ -= 0.1;
        break;
      case '\033':
        if (now_key == '[')
        {
          switch (now_key)
          {
          case 'A': // Up
            pose_.pose.position.z += STEP_HEIGHT * pose_scale_;
            break;
          case 'B': // Down
            pose_.pose.position.z -= STEP_HEIGHT * pose_scale_;
            break;
          case 'C': // Right
            pose_.pose.position.y -= STEP_POSITION * pose_scale_;
            break;
          case 'D': // Left
            pose_.pose.position.y += STEP_POSITION * pose_scale_;
            break;
          default:
            break;
          }
        }
        break;
      case 'i':
        rpy_[1] += STEP_ORIENTATION * pose_scale_;
        break;
      case ',':
        rpy_[1] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'j':
        rpy_[0] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'l':
        rpy_[0] += STEP_ORIENTATION * pose_scale_;
        break;
      case 'u':
        rpy_[2] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'o':
        rpy_[2] += STEP_ORIENTATION * pose_scale_;
        break;
      case 'k':
        pose_.pose.position.y = 0;
        rpy_[0] = rpy_[1] = rpy_[2] = 0;
        break;
      case '\x03': // Ctrl+C
        std::cout << "KEYBOARD WILL BE BACK..." << std::endl;
        rclcpp::shutdown();
        break;
      default:
        break;
      }
      this->last_key = this->now_key;
    }
    else
    {
      // switch (last_key)
      // {
      // case 'w':
      // case 'x':
      twist_.linear.x = 0.0;  // stop linear velocity
                              //   break;
                              // case 'a':
                              // case 'd':
      twist_.angular.z = 0.0; // stop angular velocity
      //   break;

      // default:
      //   break;
      // }
      // std::cout << "key released: " << key_released << " last_key: " << last_key << " now_key: " << key << std::endl;
      last_key = -1;
    }
    speed_scale_ = clamp(speed_scale_, 0.1, 4.0);
    pose_scale_ = clamp(pose_scale_, 0.1, 4.0);
    twist_.linear.x = clamp(twist_.linear.x, -MAX_VEL_X, MAX_VEL_X);
    twist_.angular.z = clamp(twist_.angular.z, -MAX_VEL_W, MAX_VEL_W);
    for (size_t i = 0; i < 3; i++)
    {
      rpy_[i] = clamp(rpy_[i], -MAX_ORIENTATION, MAX_ORIENTATION);
    }
    tf2::Quaternion q;
    q.setRPY(rpy_[0], rpy_[1], rpy_[2]);
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();
    pose_.pose.position.y = clamp(pose_.pose.position.y, -MAX_POSITION, MAX_POSITION);
    pose_.pose.position.z = clamp(pose_.pose.position.z, MIN_HEIGHT, MAX_HEIGHT);
    print_interface();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void KeyboardControllerNode::PubCmdVelCallBack()
{
  if (realtime_cmd_vel_publisher_ && realtime_cmd_vel_publisher_->trylock())
  {
    realtime_cmd_vel_publisher_->msg_ = twist_;
    realtime_cmd_vel_publisher_->unlockAndPublish();
  }
  if (realtime_posestamped_publisher_ && realtime_posestamped_publisher_->trylock())
  {
    realtime_posestamped_publisher_->msg_ = pose_;
    realtime_posestamped_publisher_->unlockAndPublish();
  }
  if (realtime_fsm_goal_publisher_ && realtime_fsm_goal_publisher_->trylock())
  {
    realtime_fsm_goal_publisher_->msg_ = fsm_goal_;
    realtime_fsm_goal_publisher_->unlockAndPublish();
  }
}
