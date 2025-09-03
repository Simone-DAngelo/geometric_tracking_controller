#include "rclcpp/rclcpp.hpp"
#include "controller/lee_controller.h"

#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include <algorithm>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "actuator_msgs/msg/actuators.hpp"

#include "utils.h"

#include <cstdio>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

using std::placeholders::_1;

class CONTROLLER : public rclcpp::Node {
    public:
        CONTROLLER();
        void run();
        void ctrl_loop();
        void request_new_plan();
        void ffilter();

    private:
        void publish_motor_commands(const Eigen::VectorXd& motor_velocities);
        void timerCallback();
        void odom_callback(const nav_msgs::msg::Odometry& msg);

        rclcpp::TimerBase::SharedPtr timer_;

        // Subscriptions - using standard ROS2 messages
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

        // Publishers - using actuator messages for motor control
        rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _motor_command_publisher;

        bool _first_odom, _new_plan;
        bool _controller_initialized;
        bool _auto_hover_enabled;  // New flag to enable automatic hovering at initial position

        //---Parameters
        string _model_name;
        string _namespace;
        double _ctrl_rate;
        int _motor_num;
        Eigen::Matrix3d _inertia;
        Eigen::Vector3d _position_gain;
        Eigen::Vector3d _velocity_gain;
        Eigen::Vector3d _attitude_gain;
        Eigen::Vector3d _angular_rate_gain;
        Eigen::VectorXd _omega_motor;
        double _mass;
        double _gravity;
        vector<double> _rotor_angles;
        vector<double> _arm_length;
        double _motor_force_k;
        double _motor_moment_k;
        double _max_motor_velocity;
        double _rotor_drag_coefficient;
        double _time_constant_up;
        double _time_constant_down;
        vector<double> _motor_rotation_direction;

        double _ref_jerk_max;
        double _ref_acc_max;
        double _ref_omega;
        double _ref_zita;

        double _ref_o_jerk_max;
        double _ref_o_acc_max;
        double _ref_o_vel_max;
        double _ref_vel_max;

        //---
        Eigen::Vector3d _perror;
        Eigen::Vector3d _verror;

        Vector3d _ref_p;
        Vector3d _ref_dp;
        Vector3d _ref_ddp;
        double _ref_yaw;
        double _rate;
        Vector3d _cmd_p;
        Vector3d _cmd_dp;
        Vector3d _cmd_ddp;
        
        Vector4d _att_q;
        Vector3d _Eta;
        Vector3d _Eta_dot;
        double _yaw_cmd;
        double _ref_dyaw;
        double _ref_ddyaw;
        Eigen::Vector3d _mes_p;
        Eigen::Vector3d _mes_dp;
        Eigen::Vector3d _omega_mes;
        Eigen::Matrix3d _R_mes;

        float _max_thrust;
        bool _armed;
};

CONTROLLER::CONTROLLER() : Node("geometric_tracking_controller"), _first_odom(false), _new_plan(false), _controller_initialized(false), _auto_hover_enabled(false) {
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&CONTROLLER::timerCallback, this));

    declare_parameter("model_name","uav");
    declare_parameter("control_rate",100.0);
    declare_parameter("rate",100.0);
    declare_parameter("motor_num",4);
    declare_parameter("inertia",std::vector<double>(3, 1));
    declare_parameter("kp",std::vector<double>(3, 1));
    declare_parameter("kd",std::vector<double>(3, 1));
    declare_parameter("attitude_gain",std::vector<double>(3, 1));
    declare_parameter("angular_rate_gain",std::vector<double>(3, 1));
    declare_parameter("mass",0.0);
    declare_parameter("gravity",0.0);
    declare_parameter("motor_force_k",0.0);
    declare_parameter("motor_moment_k",0.0);
    declare_parameter("max_motor_velocity", 0.0);
    declare_parameter("rotor_drag_coefficient", 0.0);
    declare_parameter("time_constant_up", 0.0);
    declare_parameter("time_constant_down", 0.0);
    declare_parameter("rotor_angles",std::vector<double>(4, 1.0));
    declare_parameter("arm_length",std::vector<double>(4, 1.0));
    declare_parameter("motor_rotation_direction",std::vector<double>(4, 1.0));
    declare_parameter("ref_jerk_max",0.0);
    declare_parameter("ref_acc_max",0.0);
    declare_parameter("ref_vel_max",0.0);
    declare_parameter("ref_omega",0.0);
    declare_parameter("ref_zita",0.0);
    declare_parameter("ref_o_jerk_max",0.0);
    declare_parameter("ref_o_acc_max",0.0);
    declare_parameter("ref_o_vel_max",0.0);

    auto mod_name = get_parameter("model_name").as_string();
    auto rate_c = get_parameter("control_rate").as_double();
    auto rate2_c = get_parameter("rate").as_double();
    auto mot_num = get_parameter("motor_num").as_int();
    auto iner = get_parameter("inertia").as_double_array();
    auto k_p = get_parameter("kp").as_double_array();
    auto k_d = get_parameter("kd").as_double_array();
    auto att_gain = get_parameter("attitude_gain").as_double_array();
    auto ang_rate_gain = get_parameter("angular_rate_gain").as_double_array();
    auto m = get_parameter("mass").as_double();
    auto grav = get_parameter("gravity").as_double();
    auto mot_force_k = get_parameter("motor_force_k").as_double();
    auto mot_moment_k = get_parameter("motor_moment_k").as_double();
    auto max_mot_vel = get_parameter("max_motor_velocity").as_double();
    auto rotor_drag_coeff = get_parameter("rotor_drag_coefficient").as_double();
    auto time_const_up = get_parameter("time_constant_up").as_double();
    auto time_const_down = get_parameter("time_constant_down").as_double();
    auto rot_angles = get_parameter("rotor_angles").as_double_array();
    auto arm_l = get_parameter("arm_length").as_double_array();
    auto m_rot_dir = get_parameter("motor_rotation_direction").as_double_array();
    auto refjerk_max = get_parameter("ref_jerk_max").as_double();
    auto refacc_max = get_parameter("ref_acc_max").as_double();
    auto refvel_max = get_parameter("ref_vel_max").as_double();
    auto refomega = get_parameter("ref_omega").as_double();
    auto refzita = get_parameter("ref_zita").as_double();
    auto refo_jerk_max = get_parameter("ref_o_jerk_max").as_double();
    auto refo_acc_max = get_parameter("ref_o_acc_max").as_double();
    auto refo_vel_max = get_parameter("ref_o_vel_max").as_double();

    _model_name = mod_name;
    _ctrl_rate = rate_c;
    _rate = rate2_c;
    _motor_num = mot_num;
    _inertia = Eigen::Matrix3d( Eigen::Vector3d( iner[0], iner[1], iner[2] ).asDiagonal() );
    _position_gain = Eigen::Vector3d( k_p[0], k_p[1], k_p[2] );
    _velocity_gain = Eigen::Vector3d( k_d[0], k_d[1], k_d[2] );
    _attitude_gain = Eigen::Vector3d(att_gain[0], att_gain[1], att_gain[2]);
    _angular_rate_gain = Eigen::Vector3d(ang_rate_gain[0],ang_rate_gain[1],ang_rate_gain[2]);
    _mass = m;
    _gravity = grav;
    _rotor_angles.resize( _motor_num );
    _arm_length.resize( _motor_num );
    _motor_rotation_direction.resize( _motor_num );
    for(int i=0;i<_motor_num;i++){
        _rotor_angles[i] = rot_angles[i];
        _arm_length[i] = arm_l[i];
        _motor_rotation_direction[i] = m_rot_dir[i];
    }
    _motor_force_k = mot_force_k;
    _motor_moment_k = mot_moment_k;
    _max_motor_velocity = max_mot_vel;
    _rotor_drag_coefficient = rotor_drag_coeff;
    _time_constant_up = time_const_up;
    _time_constant_down = time_const_down;
    _ref_jerk_max = refjerk_max;
    _ref_acc_max = refacc_max;
    _ref_omega = refomega;
    _ref_zita = refzita;

    _ref_o_jerk_max = refo_jerk_max;
    _ref_o_acc_max = refo_acc_max;
    _ref_o_vel_max = refo_vel_max;
    _ref_vel_max = refvel_max;

    // ROS2 pub/sub with namespace support
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    // Get namespace parameter for topic mapping
    declare_parameter("namespace", "quadrotor");  // Default to match SDF robotNamespace
    _namespace = this->get_parameter("namespace").as_string();
    
    // Ground-truth odometry from frame converter (already in FRD frame)
    std::string odom_topic = "/quadrotor/odometry_frd";
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, qos, 
        std::bind(&CONTROLLER::odom_callback, this, std::placeholders::_1));

    std::string motor_topic = "/" + _namespace + "/gazebo/command/motor_speed";
    _motor_command_publisher = this->create_publisher<actuator_msgs::msg::Actuators>(motor_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Controller initialized: %s → %s", odom_topic.c_str(), motor_topic.c_str());

    _cmd_p << 0.0, 0.0, 0.0;      // Start with zero command - wait for user input
    _cmd_dp << 0.0, 0.0, 0.0;
    _cmd_ddp << 0.0, 0.0, 0.0;
    _ref_yaw = 0.0;
    _Eta.resize(3);
    _omega_motor.resize( _motor_num );
    for(int i=0; i<_motor_num; i++ )
      _omega_motor[i] = 0.0; 

    _armed = false;
    _new_plan = false;  // Initialize with no active plan

    _mes_p.setZero();
    _mes_dp.setZero();
    _omega_mes.setZero();
    _R_mes.setIdentity();
    _Eta.setZero();
    _max_thrust = 2 * _mass * 9.81;  // Max thrust is mass * gravity
}

void CONTROLLER::odom_callback(const nav_msgs::msg::Odometry& msg) {

    _mes_p << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
    _mes_dp << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
    _omega_mes << msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;
    
    _att_q << msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
              msg.pose.pose.orientation.y, msg.pose.pose.orientation.z;
    
    _R_mes = utilities::QuatToMat(_att_q);
    _Eta = utilities::MatToRpy(_R_mes);
    
    // AUTO-HOVER initialization
    if(!_first_odom && !_auto_hover_enabled) {
        _cmd_p = _mes_p;
        _ref_yaw = _Eta(2);
        _yaw_cmd = _Eta(2);
        _auto_hover_enabled = true;
        RCLCPP_INFO(this->get_logger(), "Auto-hover enabled at [%.3f, %.3f, %.3f]", 
                   _cmd_p[0], _cmd_p[1], _cmd_p[2]);
    }
    
    _first_odom = true;
}

void CONTROLLER::timerCallback() {
    if(_first_odom && _controller_initialized){
        if(_auto_hover_enabled && !_armed) {
            _armed = true;
            RCLCPP_INFO(this->get_logger(), "Controller AUTO-ARMED");
        }
        
        if(_new_plan && _cmd_p[2] <= -0.5 && !_armed){
            _armed = true;
            RCLCPP_INFO(this->get_logger(), "Controller armed - explicit command");
        }
        
        if(_armed && (_auto_hover_enabled || _new_plan)){            
            publish_motor_commands(_omega_motor);
        }
        else {
            Eigen::VectorXd zero_commands = Eigen::VectorXd::Zero(_motor_num);
            publish_motor_commands(zero_commands);
        }
    }
    else{
        Eigen::VectorXd zero_commands = Eigen::VectorXd::Zero(_motor_num);
        publish_motor_commands(zero_commands);
    }
}

/**
 * @brief Publish motor commands using actuator_msgs
 * @param motor_velocities   Motor velocity commands in [rad/s]
 */
void CONTROLLER::publish_motor_commands(const Eigen::VectorXd& motor_velocities) {
    actuator_msgs::msg::Actuators msg;
    
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = _model_name;
    
    // Use velocity field for motor speed control in [rad/s]
    msg.velocity.resize(_motor_num);
    
    for(int i = 0; i < _motor_num; i++) {
        double raw_velocity = motor_velocities[i];
        
        // Safety checks for numerical stability
        if(std::isnan(raw_velocity) || std::isinf(raw_velocity)) {
            RCLCPP_WARN(this->get_logger(), "NaN/Inf detected in motor %d command, setting to 0", i);
            raw_velocity = 0.0;
        }
        
        // Conservative clamping based on updated SDF maxRotVelocity
        double safe_max = _max_motor_velocity;  // Now 1885 rad/s from SDF
        double clamped_velocity = std::max(0.0, std::min(raw_velocity, safe_max));
        
        msg.velocity[i] = clamped_velocity;
    }    
    _motor_command_publisher->publish(msg);
}

void CONTROLLER::request_new_plan() {
    float set_x, set_y, set_z, set_yaw;
    _new_plan = false;  // Start with no active plan
    
    while(rclcpp::ok()) {
        cout << "Insert new coordinates x (front), y (right), z (downword), yaw (clowise)" <<endl;
        scanf("%f %f %f %f", &set_x, &set_y, &set_z, &set_yaw);

        _cmd_p << set_x, set_y, set_z;
        _yaw_cmd = set_yaw;
        _new_plan = true;
        
        if(_auto_hover_enabled) {
            _auto_hover_enabled = false;
            RCLCPP_INFO(this->get_logger(), "Auto-hover DISABLED - switching to explicit command mode");
        }
        
        RCLCPP_INFO(this->get_logger(), "New plan: cmd_p=[%.2f, %.2f, %.2f], yaw=%.2f°", 
                   _cmd_p[0], _cmd_p[1], _cmd_p[2], _yaw_cmd * 180.0 / M_PI);
    }
}  

bool generate_allocation_matrix(Eigen::MatrixXd & allocation_M, 
                                    int motor_size,
                                    vector<double> rotor_angle,
                                    vector<double> arm_length, 
                                    double force_k,
                                    double moment_k,
                                    vector<double> direction ) {  // Changed from vector<int> to vector<double>

    if(motor_size <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("allocation"), "Invalid motor_size: %d", motor_size);
        return false;
    }
    
    if(rotor_angle.size() != motor_size || arm_length.size() != motor_size || direction.size() != motor_size) {
        RCLCPP_ERROR(rclcpp::get_logger("allocation"), "Vector size mismatch: motor_size=%d", motor_size);
        return false;
    }

    allocation_M.resize(4, motor_size );

    for(int i=0; i<motor_size; i++ ) {
        allocation_M(0, i) = sin( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(1, i) = cos( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(2, i) = direction[i] * force_k * moment_k;
        allocation_M(3, i) = -force_k;
    }
    
    Eigen::FullPivLU<Eigen::MatrixXd> lu( allocation_M);
    if ( lu.rank() < 4 ) {
        RCLCPP_ERROR(rclcpp::get_logger("allocation"), "Allocation matrix rank < 4: system not fully controllable");
        return false;
    }

    return true;
}

void CONTROLLER::ffilter(){
  
    //Params
    double ref_jerk_max;
    double ref_acc_max;
    double ref_omega;
    double ref_zita;
    double ref_o_jerk_max;
    double ref_o_acc_max;
    double ref_o_vel_max;

    ref_jerk_max = _ref_jerk_max;
    ref_acc_max = _ref_acc_max;
    _ref_vel_max = _ref_vel_max;
    ref_omega = _ref_omega;
    ref_zita = _ref_zita;

    ref_o_jerk_max = _ref_o_jerk_max;
    ref_o_acc_max = _ref_o_acc_max;
    ref_o_vel_max = _ref_o_vel_max;


    while( !_first_odom ) usleep(0.1*1e6);

    rclcpp::Rate r(_rate);
    double ref_T = 1.0/(double)_rate;

    _ref_p = _mes_p;
    
    Vector3d ddp;
    ddp << 0.0, 0.0, 0.0;
    Vector3d dp;  
    dp << 0.0, 0.0, 0.0;
    _ref_dp << 0.0, 0.0, 0.0;  
    _ref_ddp << 0.0, 0.0, 0.0;

    _ref_yaw = _Eta(2);
    _yaw_cmd = _Eta(2);

    _ref_dyaw = 0;
    _ref_ddyaw = 0;
    double ddyaw = 0.0;
    double dyaw = 0.0;

    Vector3d ep;
    ep << 0.0, 0.0, 0.0; 
    Vector3d jerk;
    jerk << 0.0, 0.0, 0.0;
            
    while( rclcpp::ok() ) {
        // Process commands based on mode: explicit new plan OR auto-hover
        if(_new_plan || _auto_hover_enabled) {
            // Either explicit user command OR auto-hover mode - track command position
            ep = _cmd_p - _ref_p;
        } else {
            // No active mode - keep reference stationary
            ep << 0.0, 0.0, 0.0;
        }

        double eyaw = _yaw_cmd - _ref_yaw;

        if(fabs(eyaw) > M_PI)
        eyaw = eyaw - 2*M_PI* ((eyaw>0)?1:-1);

        for(int i=0; i<3; i++ ) {
        ddp(i) = ref_omega*ref_omega * ep(i) - 2.0 * ref_zita*ref_omega*_ref_dp(i);

        jerk(i) = (ddp(i) - _ref_ddp(i))/ref_T;
        if( fabs( jerk(i) > ref_jerk_max) ) {
            if( jerk(i) > 0.0 ) jerk(i) = ref_jerk_max;
            else jerk(i) = -ref_jerk_max;
        } 

        ddp(i) = _ref_ddp(i) + jerk(i)*ref_T;
        if( fabs( ddp(i)) > ref_acc_max   ) {
            if( ddp(i) > 0.0 )
            _ref_ddp(i) = ref_acc_max;
            else 
            _ref_ddp(i) = -ref_acc_max;
        }
        else {
            _ref_ddp(i) = ddp(i);
        }

        dp(i) = _ref_dp(i) + _ref_ddp(i) * ref_T;
        if( fabs( dp(i) ) > _ref_vel_max )  {
            if( dp(i) > 0.0 ) _ref_dp(i) = _ref_vel_max;
            else _ref_dp(i) = -_ref_vel_max;
        }
        else 
            _ref_dp(i) = dp(i);

        _ref_p(i) += _ref_dp(i)*ref_T;

        }

        ddyaw = ref_omega*ref_omega * eyaw - 2.0 * ref_zita*ref_omega*_ref_dyaw;
        double o_jerk = (ddyaw - _ref_ddyaw)/ref_T;
        if ( fabs ( o_jerk ) > ref_o_jerk_max ) {
        if( o_jerk > 0.0 ) o_jerk = ref_o_jerk_max;
        else o_jerk = -ref_o_jerk_max;
        }

        ddyaw = _ref_ddyaw + o_jerk*ref_T;
        if( fabs( ddyaw ) > ref_o_acc_max ) {
        if ( ddyaw > 0.0 ) _ref_ddyaw = ref_o_acc_max;
        else if( ddyaw < 0.0 ) _ref_ddyaw = -ref_o_acc_max;
        }
        else 
        _ref_ddyaw = ddyaw;

        dyaw = _ref_dyaw + _ref_ddyaw*ref_T;
        if( fabs( dyaw ) > ref_o_vel_max ) {
        if( dyaw > 0.0 ) dyaw = ref_o_vel_max;
        else dyaw = -ref_o_vel_max;
        }
        else 
        _ref_dyaw = dyaw;

        _ref_yaw += _ref_dyaw*ref_T;

        r.sleep();
    }
}

void CONTROLLER::ctrl_loop() {

    rclcpp::Rate r(_ctrl_rate);

    Eigen::MatrixXd allocation_M;
    Eigen::MatrixXd wd2rpm;
    
    while( !_first_odom ) usleep(0.1*1e6);
    
    if(!generate_allocation_matrix( allocation_M, _motor_num, _rotor_angles, _arm_length, _motor_force_k, _motor_moment_k, _motor_rotation_direction ) ) {     
        RCLCPP_ERROR(this->get_logger(), "Failed to generate allocation matrix");
        exit(0);
    }

    boost::thread input_t( &CONTROLLER::request_new_plan, this);
    boost::thread ffilter_t(&CONTROLLER::ffilter, this);

    wd2rpm.resize( _motor_num, 4 );
    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = _inertia;
    I(3, 3) = 1;
    
    LEE_CONTROLLER lc;
    lc.set_uav_dynamics( _motor_num, _mass, _gravity, I);
    lc.set_controller_gains( _position_gain, _velocity_gain, _attitude_gain, _angular_rate_gain );
    lc.set_allocation_matrix( allocation_M );
    
    _controller_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Controller fully initialized - allocation matrix ready");
     
    Eigen::VectorXd ref_rotor_velocities;
    Eigen::Vector4d ft;
    Eigen::Vector3d att_err;

    while( rclcpp::ok() ) {
    
        // Only run controller when armed AND (we have an active plan OR auto-hover is enabled)
        if(_armed && (_new_plan || _auto_hover_enabled)) {
                        
            lc.controller(_mes_p, _ref_p, _R_mes, _mes_dp, _ref_dp, _ref_ddp, _ref_yaw, _ref_dyaw, _omega_mes, &ref_rotor_velocities, &ft, &_perror, &_verror, &att_err);   
                        
            // Safety checks for controller output
            for(int i = 0; i < 4; i++) {
                if(std::isnan(ft[i]) || std::isinf(ft[i])) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "NaN/Inf detected in force/torque %d, emergency stop", i);
                    ft << 0, 0, 0, 0;  // Emergency stop
                    break;
                }
            }
            
            if(ft[3]<-_max_thrust){
                ft[3]=-_max_thrust;
            }
            
            Eigen::VectorXd raw_motor_vels = ref_rotor_velocities;

            for(int i=0; i<_motor_num; i++ ) {
                // Safety check for motor velocities
                double motor_vel = ref_rotor_velocities[i];
                if(std::isnan(motor_vel) || std::isinf(motor_vel)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "NaN/Inf in motor velocity %d, resetting to 0", i);
                    motor_vel = 0.0;
                }
                
                // Conservative clamping based on SDF maxRotVelocity (1885 rad/s)
                double safe_max = _max_motor_velocity;  // Now 1885 rad/s from updated SDF
                motor_vel = std::max(0.0, std::min(motor_vel, safe_max));
                
                // CONSERVATIVE rate limiting: small changes per iteration for stability
                double max_change = 50.0;  // Very conservative - only 50 rad/s change per cycle
                double current_vel = _omega_motor[i];
                double vel_diff = motor_vel - current_vel;
                if(fabs(vel_diff) > max_change) {
                    motor_vel = current_vel + (vel_diff > 0 ? max_change : -max_change);
                }
                
                // Strong low-pass filter (alpha = 0.8 for very smooth transitions)
                double alpha = 0.8;  // Increased from 0.3 to 0.8 for more filtering
                _omega_motor[i] = alpha * _omega_motor[i] + (1.0 - alpha) * motor_vel;
            }
            
        } else {
            // When not armed or no active plan, gradually reduce motor speeds to zero
            for(int i=0; i<_motor_num; i++ ) {
                _omega_motor[i] *= 0.95;  // Gradually decay to zero
                if(_omega_motor[i] < 1.0) _omega_motor[i] = 0.0;  // Complete stop when very low
            }
            
            // Reset force/torque outputs
            ft << 0, 0, 0, 0;
        }
        
        r.sleep();
    }   
}

void CONTROLLER::run() {
    boost::thread ctrl_loop_t( &CONTROLLER::ctrl_loop, this );  
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CONTROLLER>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}