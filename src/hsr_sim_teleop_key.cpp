#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class HSRSimTeleopKey : public rclcpp::Node {
private:
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings
    {
    {'i', {1, 0, 0, 0}},
    {'o', {1, 0, 0, -1}},
    {'j', {0, 0, 0, 1}},
    {'l', {0, 0, 0, -1}},
    {'u', {1, 0, 0, 1}},
    {',', {-1, 0, 0, 0}},
    {'.', {-1, 0, 0, 1}},
    {'m', {-1, 0, 0, -1}},
    {'O', {1, -1, 0, 0}},
    {'I', {1, 0, 0, 0}},
    {'J', {0, 1, 0, 0}},
    {'L', {0, -1, 0, 0}},
    {'U', {1, 1, 0, 0}},
    {'<', {-1, 0, 0, 0}},
    {'>', {-1, -1, 0, 0}},
    {'M', {-1, 1, 0, 0}},
    {'k', {0, 0, 0, 0}},
    {'K', {0, 0, 0, 0}}
    };

    // Map for speed keys
    std::map<char, std::vector<float>> speedBindings
    {
    {'q', {1.1, 1.1}},
    {'z', {0.9, 0.9}},
    {'w', {1.1, 1}},
    {'x', {0.9, 1}},
    {'e', {1, 1.1}},
    {'c', {1, 0.9}}
    };


    // Reminder message
    const char* msg = R"(
    Reading from the keyboard and Publishing to Twist!
    ---------------------------
    Moving around:
    u    i    o
    j    k    l
    m    ,    .
    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
    U    I    O
    J    K    L
    M    <    >
    ---------------------------
    Simple Teleoperation with arrow keys
    
            A
          D   C
            B

    ---------------------------
    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%
    a + arrow keys : move arm_lift_joint
    s + arrow keys : move arm_flex_joint & arm_roll_joint
    d + arrow keys : move wrist_flex_joint & wrist_roll_joint
    f + arrow keys : move head_pan_joint & head_tilt_joint
    y + arrow_keys : move linear_x & linear_y & angular_z
    g : toggle hand open/close
    h : Initial pose

    NOTE : Increasing or Decreasing will take affect live on the moving robot.
        Consider Stopping the robot before changing it.
    CTRL-C to quit)";

    float speed = 0.5; 
    float turn = 1.0;  
    char key = ' ';    
    float x, y, z, th;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

    const double ARM_LIFT_MIN = 0.0;
    const double ARM_LIFT_MAX = 0.69;

    const double ARM_FLEX_MIN = -2.617;
    const double ARM_FLEX_MAX = 0.0;

    const double ARM_ROLL_MIN = -1.919;
    const double ARM_ROLL_MAX = 3.665;

    const double WRIST_FLEX_MIN = -1.919;
    const double WRIST_FLEX_MAX = 1.221;

    const double WRIST_ROLL_MIN = -1.919;
    const double WRIST_ROLL_MAX = 3.665;

    const double HEAD_PAN_MIN = -3.839;
    const double HEAD_PAN_MAX = 1.745;

    const double HEAD_TILT_MIN = -1.570;
    const double HEAD_TILT_MAX = 0.523;

public:
    HSRSimTeleopKey();
    double clamp(double value, double min_value, double max_value);
    int getch();

    void moveBaseTwist(double linear_x, double linear_y, double angular_z);
    void operateHead(const double head_tilt_pos, const double head_pan_pos, const double duration_sec);
    void operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec);
    void operateHand(bool grasp);
    float vel_check(float curr, bool decrease);
    float Lvel(char key, float x);
    float Avel(char key, float th);

    void showHelp();
    int run(int argc, char **argv);
};

HSRSimTeleopKey::HSRSimTeleopKey()
: Node("hsr_sim_teleop_key") {
}

double HSRSimTeleopKey::clamp(double value, double min_value, double max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

int  HSRSimTeleopKey::getch() {
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

float HSRSimTeleopKey::vel_check(float curr, bool decrease) {
    if (decrease) curr = (curr >= -0.95) ? curr-0.05 : -1;
    else curr = (curr <= 0.95) ? curr+0.05 : 1;
    return curr;
}

float  HSRSimTeleopKey::Lvel(char key, float x) {
    if(key=='A') return vel_check(x,false);
    if(key=='B') return vel_check(x,true);
    return 0;
}

float  HSRSimTeleopKey::Avel(char key, float th) {
    if(key=='C') return vel_check(th,true);
    if(key=='D') return vel_check(th,false);
    return 0;
}

void HSRSimTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z) {
    geometry_msgs::msg::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    pub_base_twist_->publish(twist);
}

void HSRSimTeleopKey::operateHead(const double head_tilt_pos, const double head_pan_pos, const double duration_sec) {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("head_tilt_joint");
    joint_trajectory.joint_names.push_back("head_pan_joint");

    trajectory_msgs::msg::JointTrajectoryPoint head_joint_point;

    head_joint_point.positions = {head_tilt_pos, head_pan_pos};
    head_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    joint_trajectory.points.push_back(head_joint_point);
    pub_head_trajectory_->publish(joint_trajectory);
}

void HSRSimTeleopKey::operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos,const double wrist_roll_pos,  const double duration_sec) {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("arm_lift_joint");
    joint_trajectory.joint_names.push_back("arm_flex_joint");
    joint_trajectory.joint_names.push_back("arm_roll_joint");
    joint_trajectory.joint_names.push_back("wrist_flex_joint");
    joint_trajectory.joint_names.push_back("wrist_roll_joint");

    trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

    arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos};

    arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    joint_trajectory.points.push_back(arm_joint_point);
    pub_arm_trajectory_->publish(joint_trajectory);
}

void HSRSimTeleopKey::operateHand(bool is_hand_open) {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    std::vector<double> positions;

    if(is_hand_open) {
        RCLCPP_DEBUG(this->get_logger(), "Grasp");
        positions.push_back(-0.105);
    }
    else {
        RCLCPP_DEBUG(this->get_logger(), "Open hand");
        positions.push_back(+1.239);
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    pub_gripper_trajectory_->publish(joint_trajectory);
}

void HSRSimTeleopKey::showHelp() {
    puts(msg);
}

int HSRSimTeleopKey::run(int argc, char **argv) {
    rclcpp::Rate loop_rate(40);

    this->declare_parameter<std::string>("pub_base_twist_topic_name", "/hsrb/command_velocity");
    this->declare_parameter<std::string>("pub_head_trajectory_topic_name", "/hsrb/head_trajectory_controller/command");
    this->declare_parameter<std::string>("pub_arm_trajectory_topic_name", "/hsrb/arm_trajectory_controller/command");
    this->declare_parameter<std::string>("pub_gripper_trajectory_topic_name", "/hsrb/gripper_controller/command");

    std::string pub_base_twist_topic_name;
    std::string pub_head_trajectory_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;
    
    this->get_parameter("pub_base_twist_topic_name", pub_base_twist_topic_name);
    this->get_parameter("pub_head_trajectory_topic_name", pub_head_trajectory_topic_name);
    this->get_parameter("pub_arm_trajectory_topic_name", pub_arm_trajectory_topic_name);
    this->get_parameter("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name);

    pub_base_twist_= this->create_publisher<geometry_msgs::msg::Twist>(pub_base_twist_topic_name, 10);
    pub_head_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_head_trajectory_topic_name, 10);
    pub_arm_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    pub_gripper_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);

    bool is_hand_open = false;

    int Mode = 0;
    double arm_lift_pos = 0.0;
    double arm_flex_pos = 0.0;
    double arm_roll_pos = 0.0;
    double wrist_flex_pos = 0.0;
    double wrist_roll_pos = 0.0;
    double head_tilt_pos = 0.0;
    double head_pan_pos = 0.0;
    std::set<char> pressed_keys_;
    showHelp();
  
    while(rclcpp::ok()) {
        key = getch();

        if(Mode == 0) {
            if(key=='A'||key=='B') {
                x = Lvel(key, x);
                y = 0.0;
                z = 0.0;
            }

            else if(key=='C'||key=='D') {
                th = Avel(key,th);
                y = 0.0;
                z = 0.0;
            }
            moveBaseTwist(x * speed, y * speed, th * turn);
        }
        if(Mode == 1) {
            if(key=='A'||key=='B') {
                arm_lift_pos += (key == 'A' ? 0.05 : -0.05);
                arm_lift_pos = clamp(arm_lift_pos, ARM_LIFT_MIN, ARM_LIFT_MAX);
            }
            operateArm(arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos, 1.0);
        }
        if(Mode == 2) {
            if(key=='A'||key=='B') {
                arm_flex_pos += (key == 'A' ? 0.1 : -0.1);
                arm_flex_pos = clamp(arm_flex_pos, ARM_FLEX_MIN, ARM_FLEX_MAX);
            }
            if(key=='C'||key=='D') {
                arm_roll_pos += (key == 'C' ? 0.1 : -0.1);
                arm_roll_pos = clamp(arm_roll_pos, ARM_ROLL_MIN, ARM_ROLL_MAX);
            }
            operateArm(arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos, 1.0);

        }
        if(Mode == 3) {
            if(key=='A'||key=='B') {
                wrist_flex_pos += (key == 'A' ? 0.1 : -0.1);
                wrist_flex_pos = clamp(wrist_flex_pos, WRIST_FLEX_MIN, WRIST_FLEX_MAX);
            }
            if(key=='C'||key=='D') {
                wrist_roll_pos +=  (key == 'C' ? 0.1 : -0.1);
                wrist_roll_pos = clamp(wrist_roll_pos, WRIST_ROLL_MIN, WRIST_ROLL_MAX);
            }
            operateArm(arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos, 1.0);

        }
        if(Mode == 4) {
            if(key=='A'||key=='B') {
                head_tilt_pos +=  (key == 'A' ? 0.1 : -0.1);
                head_tilt_pos = clamp(head_tilt_pos, HEAD_TILT_MIN, HEAD_TILT_MAX);
            }
            if(key=='C'||key=='D') {
                head_pan_pos += (key == 'C' ? 0.1 : -0.1);
                head_pan_pos = clamp(head_pan_pos, HEAD_PAN_MIN, HEAD_PAN_MAX);
            }
            operateHead(head_tilt_pos, head_pan_pos, 1.0);
        }
        if(this->moveBindings.count(key) == 1) {
            x = this->moveBindings[key][0];
            y = this->moveBindings[key][1];
            z = this->moveBindings[key][2];
            th = this->moveBindings[key][3];

            moveBaseTwist(x * speed, y * speed, th * turn);
        } 
        else if(speedBindings.count(key) == 1) {
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            moveBaseTwist(x * speed, y * speed, th * turn);
        }
        else if(key == 'a') {
            Mode = 1;
        }
        else if(key == 's') {
            Mode = 2;
        }
        else if(key == 'd') {
            Mode = 3;
        }
        else if(key == 'f') {
            Mode = 4;
        }
        else if(key == 'g') {
            is_hand_open = !is_hand_open;
            operateHand(is_hand_open);
        }
        if(key=='y') {
            Mode = 0;
        }
        else if(key == 'h') {
            arm_lift_pos = 0.0;
            arm_flex_pos = 0.0;
            arm_roll_pos = 0.0;
            wrist_flex_pos = -1.57;
            wrist_roll_pos = 0.0;
            head_tilt_pos = 0.0;
            head_pan_pos = 0.0;
            operateArm(arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos, 1.0);
            operateHead(head_tilt_pos, head_pan_pos, 1.0);
        }
        else if(key == '\x03') break;

        rclcpp::spin_some(this->get_node_base_interface());  
    }

    return 0;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    HSRSimTeleopKey hsr_sim_teleop_key;
    return hsr_sim_teleop_key.run(argc, argv);
}