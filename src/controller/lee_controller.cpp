#include "lee_controller.h"

using namespace std;

LEE_CONTROLLER::LEE_CONTROLLER() {}

void LEE_CONTROLLER::set_allocation_matrix(  Eigen::MatrixXd allocation_M ) {
  // Store allocation matrix dimensions for validation
  int n_motors = allocation_M.cols();
  int n_controls = allocation_M.rows();  // Should be 4 (roll, pitch, yaw, thrust)
  
  // Log matrix characteristics
  std::cout << "🔧 Allocation matrix: " << n_controls << "x" << n_motors 
            << " (controls x motors)" << std::endl;
  
  if (n_controls != 4) {
    std::cerr << "❌ ERROR: Allocation matrix must have 4 rows (roll, pitch, yaw, thrust)" << std::endl;
    return;
  }
  
  // For square matrices (4x4), use direct inverse for optimal performance
  if (n_motors == 4 && allocation_M.determinant() != 0.0) {
    _wd2rpm = allocation_M.inverse();
    std::cout << "✅ Using direct inverse for 4x4 quadrotor allocation" << std::endl;
  }
  // For over-actuated systems (n_motors > 4), use pseudo-inverse (Moore-Penrose)
  else if (n_motors > 4) {
    // Compute pseudo-inverse: A⁺ = Aᵀ(AAᵀ)⁻¹
    Eigen::MatrixXd AA_t = allocation_M * allocation_M.transpose();
    
    // Check if AA_t is invertible
    Eigen::FullPivLU<Eigen::MatrixXd> lu(AA_t);
    if (lu.rank() < 4) {
      std::cerr << "❌ ERROR: Allocation matrix rank deficient (rank=" 
                << lu.rank() << " < 4), system not fully controllable" << std::endl;
      return;
    }
    
    _wd2rpm = allocation_M.transpose() * AA_t.inverse();
    std::cout << "✅ Using pseudo-inverse for " << n_motors 
              << "-motor over-actuated system" << std::endl;
  }
  // For under-actuated systems (n_motors < 4) - not physically realizable for quadrotor control
  else {
    std::cerr << "❌ ERROR: Cannot control quadrotor with " << n_motors 
              << " motors (need at least 4 for full control)" << std::endl;
    return;
  }
  
  // Validate the computed pseudo-inverse by checking reconstruction
  Eigen::MatrixXd reconstruction_error = allocation_M * _wd2rpm - Eigen::Matrix4d::Identity();
  double max_error = reconstruction_error.array().abs().maxCoeff();
  
  if (max_error > 1e-6) {
    std::cout << "⚠️  WARNING: Pseudo-inverse reconstruction error = " << max_error 
              << " (should be < 1e-6)" << std::endl;
  } else {
    std::cout << "✅ Pseudo-inverse validation successful (error = " << max_error << ")" << std::endl;
  }
}

void LEE_CONTROLLER::set_uav_dynamics (int motor_num, double mass, double gravity, Eigen::Matrix4d I) {
  _mass = mass;
  _gravity = gravity;
  _I = I;
  _motor_num = motor_num;
}

void LEE_CONTROLLER::set_controller_gains(Eigen::Vector3d kp, Eigen::Vector3d kd, Eigen::Vector3d attitude_gain, Eigen::Vector3d angular_rate_gain ) {
  _kp = kp;
  _kd = kd;
  _attitude_gain = attitude_gain;
  _angular_rate_gain = angular_rate_gain;
}

void LEE_CONTROLLER::controller(    Eigen::Vector3d mes_p, 
                                    Eigen::Vector3d des_p,  
                                    Eigen::Matrix3d mes_R,
                                    Eigen::Vector3d mes_dp, 
                                    Eigen::Vector3d des_dp,    
                                    Eigen::Vector3d des_ddp,
                                    double des_yaw,
                                    double des_dyaw,
                                    Eigen::Vector3d mes_w,
                                    Eigen::VectorXd* rotor_velocities,
                                    Eigen::Vector4d* ft,
                                    Eigen::Vector3d* p_error,
                                    Eigen::Vector3d* v_error,
                                    Eigen::Vector3d* att_error ) {

                                      
    Eigen::Vector3d normalized_attitude_gain;
    Eigen::Vector3d normalized_angular_rate_gain;
    normalized_attitude_gain = _attitude_gain.transpose() * _I.block(0,0,3,3).inverse();
    normalized_angular_rate_gain = _angular_rate_gain.transpose() * _I.block(0,0,3,3).inverse();


    rotor_velocities->resize(_motor_num);
    Eigen::Matrix3d I_mat = _I.block(0,0,3,3);
    Eigen::Matrix3d _K_p, _K_d, _K_r, _K_w;
    Eigen::Matrix3d w_mes_skew;
    _K_p = Eigen::Matrix3d( Eigen::Vector3d(_kp[0],_kp[1], _kp[2] ).asDiagonal() );
    _K_d = Eigen::Matrix3d( Eigen::Vector3d( _kd[0], _kd[1], _kd[2] ).asDiagonal() );
    _K_r = Eigen::Matrix3d( Eigen::Vector3d( _attitude_gain[0], _attitude_gain[1], _attitude_gain[2] ).asDiagonal() );
    _K_w = Eigen::Matrix3d( Eigen::Vector3d( _angular_rate_gain[0], _angular_rate_gain[1], _angular_rate_gain[2] ).asDiagonal() );
    w_mes_skew << 0, -mes_w(2), mes_w(1),
                  mes_w(2), 0, -mes_w(0),
                 -mes_w(1), mes_w(0), 0;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  
    Eigen::Vector3d acceleration;
    Eigen::Vector3d position_error;
    position_error = mes_p - des_p;

    Eigen::Vector3d velocity_error;
    velocity_error = mes_dp - des_dp;

    acceleration = -_K_p*position_error - _K_d*velocity_error - _mass*_gravity*e_3 + _mass*des_ddp;
  
    Eigen::Vector3d angular_acceleration;
    // Eigen::Matrix3d mes_R = mes_q.toRotationMatrix();
    double yaw = des_yaw;
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = des_dyaw;
    
    Eigen::Vector3d b1_des, b2_des, b2_des_t, b3_des;
    Eigen::Matrix3d R_des;

    b1_des << cos(yaw), sin(yaw), 0;
    b3_des = -acceleration / acceleration.norm();
    b2_des_t = b3_des.cross(b1_des);
    b2_des = b2_des_t / b2_des_t.norm();

    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Angle error according to lee et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * mes_R - mes_R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    angle_error << angle_error_matrix(2, 1), angle_error_matrix(0,2), angle_error_matrix(1, 0);
    
    *att_error = angle_error;
    
    Eigen::Vector3d angular_rate_error = mes_w - mes_R.transpose() * R_des * angular_rate_des;

    angular_acceleration = -_K_r*angle_error -_K_w*angular_rate_error +mes_w.cross(I_mat*mes_w) -I_mat*((w_mes_skew*mes_R.transpose()*R_des*angular_rate_des));

    double thrust = - acceleration.dot( mes_R*e_3 );

    Eigen::Vector4d angular_acceleration_thrust;
    angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
    angular_acceleration_thrust(3) = - thrust;

    *rotor_velocities = _wd2rpm * angular_acceleration_thrust;
    *ft = angular_acceleration_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();

    *p_error = position_error;
    *v_error = velocity_error;
      
}
