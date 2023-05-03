#include "rclcpp/rclcpp.hpp"
#include "controller/lee_controller.h"
#include "nav_msgs/msg/odometry.hpp"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include "utils.h"
#include <fstream>      // std::fstream
#include <cstdio>
using std::placeholders::_1;

using namespace std;
using namespace Eigen;

class CONTROLLER : public rclcpp::Node {
    public:
        CONTROLLER();
        void LinPosVelCallback(const px4_msgs::msg::VehicleLocalPosition pos_msg);
        void AttCallback(const px4_msgs::msg::VehicleAttitude att_msg);
        void AngVelCallback(const px4_msgs::msg::VehicleAngularVelocity angvel_msg);
        void run();
        void ctrl_loop();
        void request_new_plan();
        bool get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size );
        void ffilter();
        void arm();
        void disarm();

    private:
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
        void publish_thrust_setpoint(float thrust);
        void publish_torque_setpoint(Eigen::Vector3d torque);
        void timerCallback();
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_posvel_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _local_att_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr _local_angvel_sub;
        
        // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  _cmd_vel_motor_pub;  
        rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr _vehicle_thrust_sp_publisher; 
        rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr _vehicle_torque_sp_publisher; 
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr	_vehicle_command_publisher;


        bool _first_odom, _new_plan;
        //---Parameters
        string _model_name;
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
        vector<int> _motor_rotation_direction;
        
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

        px4_msgs::msg::VehicleLocalPosition _lin_posvel;
        px4_msgs::msg::VehicleAttitude _att;
        px4_msgs::msg::VehicleAngularVelocity _ang_vel;

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

        float _thrust_normalized;
        float _max_thrust;
        Eigen::Vector3d _torque_normalized;
        Eigen::Vector4d _max_wrench, _max_rot_speed;
        bool _armed;
};

CONTROLLER::CONTROLLER() : Node("lee_controller"), _first_odom(false), _new_plan(false) {
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(7),
        std::bind(&CONTROLLER::timerCallback, this));

    // Get param from file yaml --------------------------------------- 
    declare_parameter("model_name","uav");
    declare_parameter("control_rate",0.0);
    declare_parameter("rate",0.0);
    declare_parameter("motor_num",0);
    declare_parameter("inertia",std::vector<double>(3, 1));
    declare_parameter("kp",std::vector<double>(3, 1));
    declare_parameter("kd",std::vector<double>(3, 1));
    declare_parameter("attitude_gain",std::vector<double>(3, 1));
    declare_parameter("angular_rate_gain",std::vector<double>(3, 1));
    declare_parameter("mass",0.0);
    declare_parameter("gravity",0.0);
    declare_parameter("motor_force_k",0.0);
    declare_parameter("motor_moment_k",0.0);
    declare_parameter("rotor_angles",std::vector<double>(4, 1));
    declare_parameter("arm_length",std::vector<double>(4, 1));
    declare_parameter("motor_rotation_direction",std::vector<double>(4, 1));
    declare_parameter("ref_jerk_max",0.0);
    declare_parameter("ref_acc_max",0.0);
    declare_parameter("ref_vel_max",0.0);
    declare_parameter("ref_omega",0.0);
    declare_parameter("ref_zita",0.0);
    declare_parameter("ref_o_jerk_max",0.0);
    declare_parameter("ref_o_acc_max",0.0);
    declare_parameter("ref_o_vel_max",0.0);

    auto _mod_name = get_parameter("model_name").as_string();
    auto _rate_c = get_parameter("control_rate").as_double();
    auto _rate2_c = get_parameter("rate").as_double();
    auto _mot_num = get_parameter("motor_num").as_int();
    auto _iner = get_parameter("inertia").as_double_array();
    auto _k_p = get_parameter("kp").as_double_array();
    auto _k_d = get_parameter("kd").as_double_array();
    auto _att_gain = get_parameter("attitude_gain").as_double_array();
    auto _ang_rate_gain = get_parameter("angular_rate_gain").as_double_array();
    auto _m = get_parameter("mass").as_double();
    auto _grav = get_parameter("gravity").as_double();
    auto _mot_force_k = get_parameter("motor_force_k").as_double();
    auto _mot_moment_k = get_parameter("motor_moment_k").as_double();
    auto _rot_angles = get_parameter("rotor_angles").as_double_array();
    auto _arm_l = get_parameter("arm_length").as_double_array();
    auto _m_rot_dir = get_parameter("motor_rotation_direction").as_double_array();
    auto _refjerk_max = get_parameter("ref_jerk_max").as_double();
    auto _refacc_max = get_parameter("ref_acc_max").as_double();
    auto _refvel_max = get_parameter("ref_vel_max").as_double();
    auto _refomega = get_parameter("ref_omega").as_double();
    auto _refzita = get_parameter("ref_zita").as_double();
    auto _refo_jerk_max = get_parameter("ref_o_jerk_max").as_double();
    auto _refo_acc_max = get_parameter("ref_o_acc_max").as_double();
    auto _refo_vel_max = get_parameter("ref_o_vel_max").as_double();

    _model_name = _mod_name;
    _ctrl_rate = _rate_c;
    _rate = _rate2_c;
    _motor_num = _mot_num;
    _inertia = Eigen::Matrix3d( Eigen::Vector3d( _iner[0], _iner[1], _iner[2] ).asDiagonal() );
    _position_gain = Eigen::Vector3d( _k_p[0], _k_p[1], _k_p[2] );
    _velocity_gain = Eigen::Vector3d( _k_d[0], _k_d[1], _k_d[2] );
    _attitude_gain = Eigen::Vector3d(_att_gain[0], _att_gain[1], _att_gain[2]);
    _angular_rate_gain = Eigen::Vector3d(_ang_rate_gain[0],_ang_rate_gain[1],_ang_rate_gain[2]);
    _mass = _m;
    _gravity = _grav;
    _rotor_angles.resize( _motor_num );
    _arm_length.resize( _motor_num );
    _motor_rotation_direction.resize( _motor_num );
    for(int i=0;i<_motor_num;i++){
        _rotor_angles[i] =_rot_angles[i];
        _arm_length[i] = _arm_l[i];
        _motor_rotation_direction[i] = _m_rot_dir[i];
    }
    _motor_force_k = _mot_force_k;
    _motor_moment_k = _mot_moment_k;
    _ref_jerk_max = _refjerk_max;
    _ref_acc_max = _refacc_max;
    _ref_omega = _refomega;
    _ref_zita = _refzita;

    _ref_o_jerk_max = _refo_jerk_max;
    _ref_o_acc_max = _refo_acc_max;
    _ref_o_vel_max = _refo_vel_max;
    _ref_vel_max = _refvel_max;

    // cout<<"-----------------DEBUG PARAMS: -------------\n"<<endl;
    // cout<<"Model name: "<<_model_name<<endl;
    // cout<<"ctrl_rate: "<<_ctrl_rate<<endl;
    // cout<<"rate: "<<_rate<<endl;
    // cout<<"Motor num: "<<_motor_num<<endl;
    // cout<<"I: "<<_inertia<<endl;
    // cout<<"K_p: "<<_position_gain.transpose()<<endl;
    // cout<<"K_d: "<<_velocity_gain.transpose()<<endl;
    // cout<<"K_r: "<<_attitude_gain.transpose()<<endl;
    // cout<<"L_w: "<<_angular_rate_gain.transpose()<<endl;
    // cout<<"mass: "<<_mass<<endl;
    // cout<<"grav: "<<_gravity<<endl;
    // cout<<"rotor angles: "<<endl;
    // for(int i=0;i<4;i++){cout<<_rotor_angles[i]<<", ";}
    // cout<<endl;
    // cout<<"arm lenght: "<<endl;
    // for(int i=0;i<4;i++){cout<<_arm_length[i]<<", ";}
    // cout<<endl;
    // cout<<"rot direction: "<<endl;
    // for(int i=0;i<4;i++){cout<<_motor_rotation_direction[i]<<", ";}
    // cout<<endl;
    // cout<<"K_f: "<<_motor_force_k<<endl;
    // cout<<"K_m_ "<<_motor_moment_k<<endl;
    // cout<<"_ref_jerk_max: "<<_ref_jerk_max<<endl;
    // cout<<"_ref_acc_max: "<<_ref_acc_max<<endl;
    // cout<<"_ref_omega: "<<_ref_omega<<endl;
    // cout<<"_ref_zita: "<<_ref_zita<<endl;
    // cout<<"_ref_o_jerk_max: "<<_ref_o_jerk_max<<endl;
    // cout<<"_ref_o_acc_max: "<<_ref_o_acc_max<<endl;
    // cout<<"_ref_o_vel_max: "<<_ref_o_vel_max<<endl;
    // cout<<"_ref_vel_max: "<<_ref_vel_max<<endl;
    // ------------------------------------------------------------------

    // ROS2 pub/sub
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    _local_posvel_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/vehicle_local_position/out", qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr pos_msg) {
        _mes_p << pos_msg->x, pos_msg->y, pos_msg->z;
        _mes_dp << pos_msg->vx, pos_msg->vy, pos_msg->vz;
        _first_odom = true;
    });
    _local_att_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/vehicle_attitude/out", qos, [this](const px4_msgs::msg::VehicleAttitude::UniquePtr att_msg) {
        _att_q = Eigen::Vector4d( att_msg->q[0],att_msg->q[1],att_msg->q[2],att_msg->q[3] );
        _Eta << utilities::quatToRpy (Eigen::Vector4d(att_msg->q[0],att_msg->q[1],att_msg->q[2],att_msg->q[3]));
    });
    _local_angvel_sub = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
    "/fmu/vehicle_angular_velocity/out", qos, [this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr angvel_msg) {
            _omega_mes << angvel_msg->xyz[0], angvel_msg->xyz[1], angvel_msg->xyz[2];
    });

    // _cmd_vel_motor_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(_model_name + "/cmd/motor_vel", 0);
    _vehicle_thrust_sp_publisher = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/vehicle_thrust_setpoint/in", 0);
    _vehicle_torque_sp_publisher = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("/fmu/vehicle_torque_setpoint/in", 0);
    _vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/vehicle_command/in", 0);
    //----------------
    _cmd_p << 0.0, 0.0, 0.0;
    _cmd_dp << 0.0, 0.0, 0.0;
    _cmd_ddp << 0.0, 0.0, 0.0;
    _ref_yaw = 0.0;
    _Eta.resize(3);
    _omega_motor.resize( _motor_num );
    for(int i=0; i<_motor_num; i++ )
      _omega_motor[i] = 0.0; 

    _thrust_normalized = 0.0;
    _max_thrust = 2*_mass*_gravity;
    _torque_normalized << 0,0,0;  
    _max_rot_speed << pow(1100,2), pow(1100,2), pow(1100,2), pow(1100,2);
    _armed = false;
}

// void CONTROLLER::LinPosVelCallback(const px4_msgs::msg::VehicleLocalPosition pos_msg){
//     _lin_posvel = pos_msg;
//     _mes_p << _lin_posvel.x, _lin_posvel.y, _lin_posvel.z;
//     _mes_dp << _lin_posvel.vx, _lin_posvel.vy, _lin_posvel.vz;
//     _first_odom = true;
// }

// void CONTROLLER::AttCallback(const px4_msgs::msg::VehicleAttitude att_msg){
//     _att = att_msg;
//     _Eta << utilities::quatToRpy (Eigen::Vector4d(_att.q[0],_att.q[1],_att.q[2],_att.q[3]));
// }

// void CONTROLLER::AngVelCallback(const px4_msgs::msg::VehicleAngularVelocity angvel_msg){
//     _ang_vel = angvel_msg;
//     _omega_mes << _ang_vel.xyz[0], _ang_vel.xyz[1], _ang_vel.xyz[2];
// }

// void CONTROLLER::TrajSetCallback(const px4_msgs::msg::TrajectorySetpoint trajset_msg){
//     _traj_set = trajset_msg;
// }

void CONTROLLER::timerCallback() {
    if(_first_odom){
        // RCLCPP_INFO(this->get_logger(), "DEBUGGER");
        // cout<<"\nLinear Pos: "<<_mes_p.transpose()<<"\n"<<endl;
        // cout<<"\nLinear Vel: "<<_mes_dp.transpose()<<"\n"<<endl;
        // cout<<"\nAttitude: "<<_Eta.transpose()<<"\n"<<endl;
        // cout<<"\nQuaternion: "<<Eigen::Vector4d(_att.q[0],_att.q[1],_att.q[2],_att.q[3]).transpose()<<endl;
        // cout<<"\nAng Vel: "<<_omega_mes.transpose()<<"\n"<<endl;
        // RCLCPP_INFO(this->get_logger(), "--- END ---");
        // cout<<endl;
        std_msgs::msg::Float32MultiArray motor_vel;
        motor_vel.data.resize( _motor_num );
    
        if(_cmd_p[2]<=-0.2 && !_armed){
            // publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
            _armed = true;
        }
        else if(_cmd_p[2]>-0.2 && _ref_p[2]>-0.2 && _armed && _new_plan){
            disarm();
            _armed = false;
        }
        if(_new_plan){
            // for(int i=0; i<_motor_num; i++ ) {
            //     motor_vel.data[i] =  _omega_motor(i);  
            // }  
            publish_thrust_setpoint(_thrust_normalized); 
            publish_torque_setpoint(_torque_normalized);
        }
        else{
            // for(int i=0; i<_motor_num; i++ ) {
            //     motor_vel.data[i] =  0.0;  
            // }   
            publish_thrust_setpoint(0.0); 
            publish_torque_setpoint(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        // _cmd_vel_motor_pub->publish( motor_vel );
        
    }
    else{
        RCLCPP_INFO(this->get_logger(), "WAITING FOR ODOM...");
    }
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void CONTROLLER::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_command_publisher->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void CONTROLLER::arm(){
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void CONTROLLER::disarm(){
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void CONTROLLER::publish_thrust_setpoint(float thrust){
	px4_msgs::msg::VehicleThrustSetpoint msg{};
    
	msg.xyz[0] = 0;
    msg.xyz[1] = 0;
    msg.xyz[2] = thrust;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_thrust_sp_publisher->publish(msg);
}

void CONTROLLER::publish_torque_setpoint(Eigen::Vector3d torque)
{
	px4_msgs::msg::VehicleTorqueSetpoint msg{};
    
	msg.xyz[0] = torque[0];
    msg.xyz[1] = torque[1];
    msg.xyz[2] = torque[2];
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_torque_sp_publisher->publish(msg);
}

void CONTROLLER::request_new_plan() {
    float set_x, set_y, set_z, set_yaw;
    while(rclcpp::ok()) {
        cout << "Insert new coordinates x (front), y (left), z (upword), yaw (c-clowise)" <<endl;
        scanf("%f %f %f %f", &set_x, &set_y, &set_z, &set_yaw);
        cout << "Request new plan for: [" << set_x << ", " << set_y << ", " << set_z << " - " << set_yaw << "]" << endl;

        //---ENU -> NED
        _cmd_p << set_x, -set_y, -set_z;
        _yaw_cmd = -set_yaw;
        //---
        _new_plan = true;
    }
    _new_plan = false;
}  

bool generate_allocation_matrix(Eigen::MatrixXd & allocation_M, 
                                    int motor_size,
                                    vector<double> rotor_angle,
                                    vector<double> arm_length, 
                                    double force_k,
                                    double moment_k,
                                    vector<int> direction ) {

    allocation_M.resize(4, motor_size );

    for(int i=0; i<motor_size; i++ ) {
        allocation_M(0, i) = sin( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(1, i) = cos( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(2, i) = direction[i] * force_k * moment_k;
        allocation_M(3, i) = -force_k;
    }
    
    Eigen::FullPivLU<Eigen::Matrix4Xd> lu( allocation_M);
    if ( lu.rank() < 4 ) {
        cout<<"The allocation matrix rank is lower than 4. This matrix specifies a not fully controllable system, check your configuration"<<endl;
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

    _cmd_p << _lin_posvel.x, _lin_posvel.y, _lin_posvel.z;
    _ref_p = _cmd_p;

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
        ep = _cmd_p - _ref_p;

        //cout << "_cmd_p: " << _cmd_p.transpose() << endl;
        //cout << "_ref_f: " << _ref_p.transpose() << endl;

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

    //---Input
    Eigen::Vector3d des_p;              
    Eigen::Vector3d des_dp; 
    Eigen::Vector3d des_ddp; 
    des_dp << 0.0, 0.0, 0.0;
    des_ddp << 0.0, 0.0, 0.0;

    Eigen::Vector4d mes_q;
    Eigen::Vector3d mes_dp;    
    Eigen::Vector3d mes_w;

    //---

    Eigen::MatrixXd allocation_M;
    Eigen::MatrixXd wd2rpm;
    
    while( !_first_odom ) usleep(0.1*1e6);
    // _mes_p << _lin_posvel.x, _lin_posvel.y, _lin_posvel.z;
    // _mes_dp << _lin_posvel.vx, _lin_posvel.vy, _lin_posvel.vz;
    
    if(!generate_allocation_matrix( allocation_M, _motor_num, _rotor_angles, _arm_length, _motor_force_k, _motor_moment_k, _motor_rotation_direction ) ) {     
        cout << "Wrong allocation matrix" << endl;
        exit(0);
    }

    // ----- max thrust and torque computation
        _max_wrench[0] =  sin( _rotor_angles[0] ) * _arm_length[0] * _motor_force_k * _max_rot_speed[0] + sin( _rotor_angles[2] ) * _arm_length[2] * _motor_force_k * _max_rot_speed[2];
        _max_wrench[1] =  cos( _rotor_angles[1] ) * _arm_length[1] * _motor_force_k * _max_rot_speed[2] + cos( _rotor_angles[2] ) * _arm_length[2] * _motor_force_k * _max_rot_speed[2];
        _max_wrench[2] =  2 * _motor_force_k * _motor_moment_k * _max_rot_speed[0];
        _max_wrench[3] = -4 * _motor_force_k * _max_rot_speed[0];
        cout<<"Vector of maximum command wrench: "<<_max_wrench.transpose()<<endl;
    // ---------------------------------------

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
     
    Eigen::VectorXd ref_rotor_velocities;
    Eigen::Vector4d ft;
    
    Vector3d att_err;
    float alfa=0.0;

    while( rclcpp::ok() ) {
        //Measured    
        // _mes_p << _lin_posvel.x, _lin_posvel.y, _lin_posvel.z;
        // _mes_dp << _lin_posvel.vx, _lin_posvel.vy, _lin_posvel.vz;
        mes_q = _att_q;
        mes_w = _omega_mes;  

        
        Eigen::Matrix3d mes_R = utilities::QuatToMat( mes_q );
        // mes_w = mes_R.transpose()*mes_w;
    
        lc.controller(_mes_p, _ref_p, mes_R, _mes_dp, _ref_dp, _ref_ddp, _ref_yaw, _ref_dyaw, mes_w, &ref_rotor_velocities, &ft, &_perror, &_verror, &att_err);   
        if(ft[3]<-_max_thrust){
            ft[3]=-_max_thrust;
        }
        if(_ref_p(2)>-0.05 && _cmd_p(2)>-0.3){
            ft<< 0,0,0,0;
        }
        else{
        alfa = alfa+0.002;
        alfa = min(1.0f,alfa);
        }
        ft(3) = alfa*ft(3);

        // RCLCPP_INFO(this->get_logger(),"Thrust Command: %lf",ft[3]);
        _thrust_normalized = 0.0-((abs(ft[3])-0.0)/(_max_thrust-0)); //zi = (xi – min(x)) / (max(x) – min(x)) normalizzazione tra 0 e 1 (tra -1 e 0 fai il contrario)
        // _torque_normalized << ft[0], ft[1], ft[2];
        if(ft[0]<0.0){
            _torque_normalized[0] = 0.0-(abs(ft[0])-0.0)/(abs(_max_wrench[0])-0.0);
        }
        else{
            _torque_normalized[0] = (abs(ft[0])-0.0)/(abs(_max_wrench[0])-0.0);
        }
        if(ft[1]<0.0){
            _torque_normalized[1] = 0.0-(abs(ft[1])-0.0)/(abs(_max_wrench[1])-0.0);
        }
        else{
            _torque_normalized[1] = (abs(ft[1])-0.0)/(abs(_max_wrench[1])-0.0);
        }
        if(ft[2]<0.0){
            _torque_normalized[2] = 0.0-(abs(ft[2])-0.0)/(abs(_max_wrench[2])-0.0);
        }
        else{
            _torque_normalized[2] = (abs(ft[2])-0.0)/(abs(_max_wrench[2])-0.0);
        }

        for(int i=0; i<_motor_num; i++ ) {
            _omega_motor[i] = ref_rotor_velocities[i]; 
        }
        r.sleep();
    }   
}

void CONTROLLER::run() {
    boost::thread ctrl_loop_t( &CONTROLLER::ctrl_loop, this );  
    // auto node = std::make_shared<CONTROLLER>();
    // rclcpp::spin(node);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CONTROLLER>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}