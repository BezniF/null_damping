#ifndef NULL_DAMPING_H_
#define NULL_DAMPING_H_
#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>

// CVXgen
extern "C" {
    #include "solver.h"
}

class NullDamping
{
    public:
        NullDamping();
        ~NullDamping();
        void spin();

    private:
        // Callbacks
        void jointPosSimCb(const sensor_msgs::JointStateConstPtr& msg);
		void jointPosCb(const sensor_msgs::JointStateConstPtr& msg);
		void forceCallback(const geometry_msgs::WrenchStamped& msg);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
		// void obsCallback(const geometry_msgs::PoseStampedConstPtr& msg);
		// void urBaseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        // Functions
        void init();
        void computeAdmittance();
        void kinematicDecomposition();
        Eigen::Vector3d computeOptimization();
        void updateTank();
        void publishVelocity();
        void saturateDynamics();
        void writeToFiles();

        // Node Handle
        ros::NodeHandle nh_;

        // Subscribers
		ros::Subscriber joints_pos_sub_;
		ros::Subscriber joints_pos_sim_sub_;
		ros::Subscriber wrench_sub_;
        ros::Subscriber joy_sub_;

        // Publishers
        ros::Publisher arm_pose_pub_;
        ros::Publisher arm_pose_sim_pub_;

        // MoveIt!
        robot_model_loader::RobotModelLoader* robot_model_loader;

        // Admittance params
        Eigen::Matrix<double, 6, 6> M_a_, D_a_;
        Eigen::Matrix<double, 6, 6> dot_M_;
        Eigen::Matrix<double, 6, 1> initial_twist_;
        Eigen::VectorXd des_twist_adm;
        Eigen::Vector3d x_1;

        // Jacobians
        Eigen::MatrixXd jacobian_arm;
        Eigen::Matrix<double, 3, 6> jacobian_adm_reduced_; // J1
        Eigen::Matrix<double, 6, 6> jacobian_augmented_; // Je
        Eigen::Matrix<double, 6, 6> jacobian_augmented_inverse_; // Je
        Eigen::Matrix<double, 6, 3> J_W_pinv_;

        // Decoupling matrices
        Eigen::Matrix<double, 6, 6> W_; // weight matrix
        Eigen::Matrix<double, 3, 6> Z_; // full row rank null space base matrix
        Eigen::Matrix<double, 3, 6> N_; // null space projector
        Eigen::Matrix<double, 3, 3> D_null_; // null space damping matrix
        Eigen::Matrix<double, 3, 3> dot_D_null_; // variation of the null space damping

        // Velocity variables
        Eigen::Vector3d dotx_2_;
        Eigen::Vector3d dotx_2_prev_;
        Eigen::Vector3d ddotx_2_;
        Eigen::Matrix<double, 6, 1> dot_q_;
        Eigen::Matrix<double, 6, 1> dot_q_prev_;
        Eigen::Matrix<double, 6, 1> dotx_e_print_;

        // Force variables
        Eigen::Matrix<double, 6, 1> F_e_;
        Eigen::Matrix<double, 6, 1> F_e_prev_;
        Eigen::Matrix<double, 6, 1> F_e_prev_filter_;
        Eigen::Matrix<double, 3, 1> F_d_;
        std::vector<std::vector<double>> F_e_filter_;
        double FILTERING_WINDOW_ = 3000.0;

        // Joint state variables
        double initial_pos_array[6];
        double initial_vel_array[6];
        double roll_ee, pitch_ee, yaw_ee;
        double roll_ee_init, pitch_ee_init, yaw_ee_init; 
        Eigen::Vector3d ee_pose_linear;
        Eigen::Matrix3d ee_pose_angular;

        // Control variables
        bool REAL_ROBOT_;
        bool DAMPING_INJECTION_;
        bool first_cycle_;
        bool activate_null_;
        bool increase_inertia_;
        bool upper_bound_;
        bool lower_bound_;

        // Tank variables
    	float tank_state_;
    	float tank_energy_;
    	double xt_dot_;
		const float TANK_INITIAL_VALUE = 27;
    	const float TANK_MAX_VALUE = 500;
        const float TANK_DES_VALUE = 25;
    	const float TANK_MIN_VALUE = 5;
        double P_np_; // power loss due to non-passive behaviors

        // To-File variables
        double current_time;
        double cycle_time;
        double start_time_;
        double damp_time_;
        std::ofstream rank_file_;
        std::ofstream tank_file_;
        std::ofstream dotq_file_;
        std::ofstream dotx_e_file_;
        std::ofstream force_file_;
        std::ofstream null_pow_file_;

};

#endif /* REDUNDANCY_SOLVER_H */