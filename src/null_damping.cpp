#include "null_damping.h"
#include "solver.h"
#include "solver.c"
#include "ldl.c"
#include "matrix_support.c"
#include "util.c"

NullDamping::NullDamping(){

    cycle_time = 0.002; // was 0.002
    
    joints_pos_sub_= nh_.subscribe("/joint_states", 1, &NullDamping::jointPosCb, this);
    joints_pos_sim_sub_= nh_.subscribe("joints_value", 1, &NullDamping::jointPosSimCb, this);
    wrench_sub_ = nh_.subscribe("wrench", 1, &NullDamping::forceCallback, this);
    joy_sub_ = nh_.subscribe("/joy", 1, &NullDamping::joyCallback, this);

    arm_pose_sim_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_traj_controller/command", 1);
    arm_pose_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    std::vector<double> M_a;
    std::vector<double> D_a;
    std::vector<double> D_n;
    std::vector<double> des_twist_init (6, 0.0);

    // MoveIt structs
    robot_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");

    if (!nh_.getParam("mass_arm", M_a)) {
        ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    }

    if (!nh_.getParam("damping_arm", D_a)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    }

    if (!nh_.getParam("damping_null", D_n)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the arm in the null space.");
    }

    if (!nh_.getParam("DAMPING_INJECTION", DAMPING_INJECTION_)) {
        ROS_ERROR("Couldn't retrieve the desired damping behaviour.");
    }

    if (!nh_.getParam("REAL_ROBOT", REAL_ROBOT_)) {
        ROS_ERROR("Couldn't retrieve which robot to use.");
    }

    M_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_a.data());
    D_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_a.data());
    D_null_ = Eigen::Map<Eigen::Matrix<double, 3, 3> >(D_n.data());

    F_e_filter_.resize(int(FILTERING_WINDOW_));
    
    for(int i = 0; i < F_e_filter_.size(); i++)
        F_e_filter_[i].resize(6); 

    dot_M_.setZero();
    dot_D_null_.setZero();
    F_e_.setZero();
    F_e_prev_.setZero();
    F_e_prev_filter_.setZero();
    dot_q_prev_.setZero();

    tank_energy_ = TANK_INITIAL_VALUE;
    tank_state_ = sqrt(2 * tank_energy_);

    // Time cycle variables
    start_time_ = ros::Time::now().toSec();

    // Output to file stream
    std::stringstream file_path;
    file_path << "/home/federico/MATLAB_ws/IROS21-Null/src/rank.txt";
    rank_file_.open(file_path.str());

    std::stringstream file_path2;
    file_path2 << "/home/federico/MATLAB_ws/IROS21-Null/src/tank.txt";
    tank_file_.open(file_path2.str());

    std::stringstream file_path3;
    file_path3 << "/home/federico/MATLAB_ws/IROS21-Null/src/dotx_e.txt";
    dotx_e_file_.open(file_path3.str());

    std::stringstream file_path4;
    file_path4 << "/home/federico/MATLAB_ws/IROS21-Null/src/force.txt";
    force_file_.open(file_path4.str());

    //! CONTROL FLAGS
    // REAL_ROBOT_ = false;
    // DAMPING_INJECTION_ = true;

    //! CYCLE VARIABLES
    first_cycle_ = true;
    activate_null_ = false;
    increase_inertia_ = false;
    lower_bound_ = false;
    upper_bound_ = false;

}

NullDamping::~NullDamping(){
    rank_file_.close();
}

void NullDamping::jointPosCb(const sensor_msgs::JointStateConstPtr& msg){
    if(REAL_ROBOT_){
        sensor_msgs::JointState initial_pos_joints_ =  *msg;

        for(int i=0; i < 6; i++){
            initial_pos_array[i] = initial_pos_joints_.position[i];
            initial_vel_array[i] = initial_pos_joints_.velocity[i];
        }
        double tmp_pos = initial_pos_array[2];
        initial_pos_array[2] = initial_pos_array[0];
        initial_pos_array[0] = tmp_pos;

        double tmp_vel = initial_vel_array[2];
        initial_vel_array[2] = initial_vel_array[0];
        initial_vel_array[0] = tmp_vel;
    }
}

void NullDamping::jointPosSimCb(const sensor_msgs::JointStateConstPtr& msg){
    if(!REAL_ROBOT_){
        sensor_msgs::JointState  array_msg = *msg;

        for(int i=0; i < 6; i++){
            initial_pos_array[i] = array_msg.position[i];
        }
    }
}

void NullDamping::forceCallback(const geometry_msgs::WrenchStamped& msg){
    if(REAL_ROBOT_){
        //Storing the force msg coming from the F/T sensor
        F_e_[0] = msg.wrench.force.x;
        F_e_[1] = msg.wrench.force.y;
        F_e_[2] = msg.wrench.force.z;
        F_e_[3] = msg.wrench.torque.x;
        F_e_[4] = msg.wrench.torque.y;
        F_e_[5] = msg.wrench.torque.z;

        // Satirating the sensor's uncertainties
        double F_sat = 4.0;
        double T_sat = 10.0;

        for(int i = 0; i < 3; i++){
            if((F_e_[i] > 0.0) && (F_e_[i] < F_sat))
                F_e_[i] = 0.0;

            else if((F_e_[i] < 0.0) && (F_e_[i] > - F_sat))
                F_e_[i] = 0.0;
        }

        for(int i = 3; i < F_e_.size(); i++){
            if((F_e_[i] > 0.0) && (F_e_[i] < T_sat))
                F_e_[i] = 0.0;

            else if((F_e_[i] < 0.0) && (F_e_[i] > - T_sat))
                F_e_[i] = 0.0;
        }

        // Filtering the input
        // std::vector<double> f_e_in;
        // f_e_in.resize(6);
        // for(int i = 0; i < 6; i++)
        //     f_e_in[i] = F_e_[i];
        
        // F_e_filter_.push_back(f_e_in);
        // F_e_filter_.erase(F_e_filter_.begin());

        // double F_x = 0.0;
        // double F_y = 0.0;
        // double F_z = 0.0;

        // for(int i = 0; i < F_e_filter_.size(); i++){
        //     F_x +=  F_e_filter_[i][0]/ FILTERING_WINDOW_;
        //     F_y +=  F_e_filter_[i][1]/ FILTERING_WINDOW_;
        //     F_z +=  F_e_filter_[i][2]/ FILTERING_WINDOW_;
        // }

        // F_e_[0] = F_x;
        // F_e_[1] = F_y;
        // F_e_[2] = F_z;

        // double filter_threshold = 2.0;
        // if((F_e_[2] > F_e_prev_filter_[2] + filter_threshold) || (F_e_[2] < F_e_prev_filter_[2] - filter_threshold))
        //     F_e_[2] = F_e_prev_filter_[2];

        // F_e_prev_filter_ = F_e_;

    }
}

//* Callback for debugging stuff using the joystick
void NullDamping::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    if ((bool)msg->buttons[4])
        P_np_ = 5.0;
    else
        P_np_ = 0.0;

    if(!REAL_ROBOT_){
        double force_gain = 50;
        F_e_[0] = force_gain * msg->axes[0];
        F_e_[1] = force_gain * msg->axes[1];
        F_e_[2] = force_gain * msg->axes[4];
    
        double F_sat = 4.0;

        for(int i = 0; i < 3; i++){
            if((F_e_[i] > 0.0) && (F_e_[i] < F_sat))
                F_e_[i] = 0.0;

            else if((F_e_[i] < 0.0) && (F_e_[i] > - F_sat))
                F_e_[i] = 0.0;
        }
    }

}


void NullDamping::init(){
    current_time = ros::Time::now().toSec();

    // Instantiating the robot model
    robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel();

    // Creating the robot kinematic state and setting it to the current joints positions
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setVariablePositions(initial_pos_array);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Pasting the kinematic model to the Joint Groups in MoveIt!
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    jacobian_arm;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position,
                                jacobian_arm);

    // Correct the forces using the reference frame of the tool
    const Eigen::Affine3d& end_effector_state_tool = kinematic_state->getGlobalLinkTransform("tool0");

    Eigen::Matrix3d ee_pose_angular_tool = end_effector_state_tool.rotation();

    Eigen::MatrixXd force_mapping(6,6);
    Eigen::MatrixXd force_mapping_inverse(6,6);
    force_mapping << ee_pose_angular_tool,
                    Eigen::Matrix3d::Zero(3,3),
                    Eigen::Matrix3d::Zero(3,3),
                    ee_pose_angular_tool;

    Eigen::Matrix<double,6,1> mapped_force;
    mapped_force = force_mapping * F_e_;
    F_e_ = mapped_force;

    force_mapping_inverse << ee_pose_angular_tool.inverse(),
                    Eigen::Matrix3d::Zero(3,3),
                    Eigen::Matrix3d::Zero(3,3),
                    ee_pose_angular_tool.inverse();

    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");

    // Compute the pose of the end-effector with respect to the /base frame //? are we sure?
    ee_pose_linear = end_effector_state.translation();
    ee_pose_angular = end_effector_state.rotation();

    // ROS_INFO_STREAM("EE Z: " << ee_pose_linear[2]);

    // Compute RPY by converting through tf
    geometry_msgs::TransformStamped rot_msg = tf2::eigenToTransform(end_effector_state);

    tf2::Quaternion quat;
    tf2::convert(rot_msg.transform.rotation, quat);
    tf2::Matrix3x3(quat).getRPY(roll_ee, pitch_ee, yaw_ee);

    // Correct the Jacobian for a pose control application
    double R,P,Y;

    R = roll_ee;
    P = pitch_ee;
    Y = yaw_ee;

    Eigen::Matrix3d T_rpy;
    T_rpy << cos(Y)*cos(P),  -sin(Y), 0, 
             sin(Y)*cos(P),   cos(Y), 0,
                   -sin(P),        0, 1;

    Eigen::MatrixXd transform_RPY_arm(6,6);
    transform_RPY_arm <<Eigen::Matrix3d::Identity(3,3),
                    Eigen::Matrix3d::Zero(3,3),
                    Eigen::Matrix3d::Zero(3,3),
                    T_rpy.inverse();

    jacobian_arm = transform_RPY_arm * jacobian_arm;

    if(first_cycle_){
        roll_ee_init = roll_ee;
        pitch_ee_init = pitch_ee_init;
        yaw_ee_init = yaw_ee;
    }

    // Apply a sinusaidal virtual force in simulation 
    if(!REAL_ROBOT_){
        double K_force = -2.0;

        // for(int i = 0; i < 6; i++){
        //     if(i!=2)
        //         F_e_[i] = 0.0;
        // }
        
        F_e_ = F_e_prev_;

        if(first_cycle_)
        {
            F_e_[2] = K_force;
            first_cycle_ = false;
        }

        //F_e_[2] = K_force * sin( 1.0 * (ros::Time::now().toSec() - start_time_));
        if((ee_pose_linear[2] > 0.8) && (F_e_[2] > 0))
            F_e_[2] = -F_e_[2];
        else if(ee_pose_linear[2] < 0.75 && F_e_[2] < 0)
            F_e_[2] = -F_e_[2];

        double dot_m = 3;

        // Change the inertia matrix 
        if((F_e_prev_[2] <= 0.0) && (F_e_[2] > 0.0)){
            Eigen::Matrix<double, 6,1> dot_m_vec;
            dot_m_vec << -dot_m, -dot_m, -dot_m, -dot_m, -dot_m, -dot_m;
            dot_M_ = dot_m_vec.array().matrix().asDiagonal();
            // ROS_INFO_STREAM("Dot_M \n" << dot_M_);
        }
        else if((F_e_prev_[2] >= 0.0) && (F_e_[2] < 0.0)){
        // else if(increase_inertia_){
            Eigen::Matrix<double, 6,1> dot_m_vec;
            dot_m_vec << dot_m, dot_m, dot_m, dot_m, dot_m, dot_m;
            dot_M_ = dot_m_vec.array().matrix().asDiagonal();
            increase_inertia_ = false;
            // ROS_INFO_STREAM("Dot_M \n" << dot_M_);
        }
        else{
            dot_M_.setZero();
            // ROS_INFO_STREAM("Dot_M \n" << dot_M_);
        }
        
        // Increase the mass matrix accordingly
        M_a_ = M_a_ + dot_M_;

        // ROS_INFO_STREAM("Mass matrix: \n" << M_a_);
        // ROS_INFO_STREAM("F ext: \n" << F_e_);

    }
    if(REAL_ROBOT_){
        // Change the inertia matrix
        double dot_m = 10;

        // for(int i=0; i < 6; i++){
        //     if(i == 2){

        //     }
        //     else
        //         F_e_[i] = 0.0;
        // }
        // ROS_INFO_STREAM("F ext: \n" << F_e_);

        
        //if((F_e_prev_[2] < 0.0) && (F_e_[2] > 0.0)){
            if((ee_pose_linear[2] < 0.69) && (!upper_bound_)){
            Eigen::Matrix<double, 6,1> dot_m_vec;
            dot_m_vec << -dot_m, -dot_m, -dot_m, -dot_m, -dot_m, -dot_m;
            dot_M_ = dot_m_vec.array().matrix().asDiagonal();
            std::cout << "*******************************" << std::endl;
            ROS_WARN_STREAM("DROP INERTIA");
            //ROS_INFO_STREAM("F_e_ \n" << F_e_);
            //ROS_INFO_STREAM("F_e prev \n " << F_e_prev_);
            std::cout << "*******************************" << std::endl;
            upper_bound_ = true;
            lower_bound_ = false;
        }
        else if((ee_pose_linear[2] > 0.81) && (!lower_bound_)){
            Eigen::Matrix<double, 6,1> dot_m_vec;
            dot_m_vec << dot_m, dot_m, dot_m, dot_m, dot_m, dot_m;
            dot_M_ = dot_m_vec.array().matrix().asDiagonal();
            std::cout << "*******************************" << std::endl;
            increase_inertia_ = false;
            ROS_ERROR_STREAM("GROW INERTIA");
            // ROS_INFO_STREAM("F_e_ \n" << F_e_);
            // ROS_INFO_STREAM("F_e prev \n" << F_e_prev_);
            std::cout << "*******************************" << std::endl;
            upper_bound_= false;
            lower_bound_ = true;
        }
        else{
            dot_M_.setZero();
            // ROS_INFO_STREAM("Dot_M \n" << dot_M_);
        }
        
        // Increase the mass matrix accordingly
        M_a_ = M_a_ + dot_M_;   

    }

}

void NullDamping::computeAdmittance(){
    Eigen::VectorXd adm_desired_accelaration(6);
    
    initial_twist_ = jacobian_arm * dot_q_prev_;
    Eigen::MatrixXd D_inj(6,6);

    D_inj = Eigen::MatrixXd::Zero(6,6); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! EXP IN LAB
    if(DAMPING_INJECTION_){
        if(tank_energy_ >= TANK_DES_VALUE)
            D_inj = Eigen::MatrixXd::Zero(6,6);
        else{
            for(int i = 0; i < 6; i++){
                if(des_twist_adm[i] != 0.0)
                    // D_inj(i,i) = fabs(F_e_[i] / des_twist_adm[i]);
                    D_inj(i,i) = 4.0;
            }
        }
    }
    adm_desired_accelaration = (M_a_).inverse() * ( - (D_a_ + D_inj) * initial_twist_ + F_e_);

    des_twist_adm = initial_twist_ + adm_desired_accelaration * cycle_time;

    if((des_twist_adm[2] < 0.0) && (initial_twist_[2] >= 0.0)){
        increase_inertia_ = true;
    }

    for(int i = 0; i < adm_desired_accelaration.size(); i++){
        if(F_e_[i] == 0.0)
            adm_desired_accelaration[i] = 0.0;
    }
    for(int i = 0; i < F_e_.size(); i++){
        if(F_e_[i] == 0.0)
            des_twist_adm[i] = 0.0;
    }

    // ROS_INFO_STREAM("Des twist adm: \n" << des_twist_adm);

}

void NullDamping::kinematicDecomposition(){
    // Admittance only for translational parts
    jacobian_adm_reduced_ = jacobian_arm.block(0,0,3,6);

    // SVD from Eigen -> compute a basis for the null of Jadm_reduced
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_adm_reduced_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd2(jacobian_adm_reduced_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd null_space_base = svd.matrixV().block<6,3>(0,3);

    // std::cout << "Its FULL right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;
    // std::cout << "Its THIN right singular vectors are the columns of the thin V matrix:" << std::endl << svd2.matrixV() << std::endl;
    // std::cout << "RANK: " << svd.rank() << std::endl;
    // std::cout << "Its null space base is:" << std::endl << null_space_base << std::endl;

    rank_file_ << svd.rank() << std::endl;

    // Transpose because Z is full row rank
    Z_ = null_space_base.transpose();

    // Assign the admittance mass matrix as weight //TODO CHECK IF THIS MAKES SENSE OR CAN BE IMPROVED
    // W_ = M_a_;
    W_ = Eigen::MatrixXd::Identity(6,6);

    // Compute the null space projector
    N_ = (Z_ * W_ * Z_.transpose()).inverse() * Z_ * W_;

    // Weighted pseudoinverse of J1
    J_W_pinv_ = W_.inverse() * jacobian_adm_reduced_.transpose() * (jacobian_adm_reduced_ * W_.inverse() * jacobian_adm_reduced_.transpose()).inverse();

    x_1 << des_twist_adm[0], des_twist_adm[1], des_twist_adm[2];

    Eigen::MatrixXd checK_null = N_ * J_W_pinv_* x_1;
    Eigen::MatrixXd checK_null2 = jacobian_adm_reduced_ * Z_.transpose() * x_1;

    double K_null;
    if (tank_energy_ >= TANK_DES_VALUE){
        K_null = 0.0;
        activate_null_ = false;
    } 
    else{
        double D_null_gain = ((TANK_DES_VALUE - tank_energy_) * 0.5);
        if(D_null_gain <= 0.0)
            D_null_gain = 0.0;
        //for(int i = 0; i < 3; i++)
            //dot_D_null_(i,i) = D_null_gain;
        if(!activate_null_){
            damp_time_ = ros::Time::now().toSec();

            activate_null_ = true;
        }

        K_null = 0.5 * (1 + D_null_gain);
    }

    // Generate the null motion
    double sin_rpy = K_null * sin(1.0 * (ros::Time::now().toSec() - start_time_));

    Eigen::Vector3d F_null;

    if(!DAMPING_INJECTION_){
        if(!REAL_ROBOT_)
            F_null = {0.0, sin_rpy, -sin_rpy};
        if(REAL_ROBOT_)
            F_null = {0.0, 0.0, -sin_rpy};
    }
    else
        F_null = {0.0, 0.0, 0.0};

    // Eigen::MatrixXd check_null2 = jacobian_adm_reduced_ * Z_.transpose() * dotx_2_;

    // Dynamic computations in the null
    ddotx_2_ = - (D_null_ + dot_D_null_) * dotx_2_prev_ + F_null;
    dotx_2_ = dotx_2_prev_ + cycle_time * ddotx_2_;

    // Augment Jacobian
    jacobian_augmented_.topRows(3) = jacobian_adm_reduced_;
    jacobian_augmented_.bottomRows(3) = N_;

    jacobian_augmented_inverse_.leftCols(3) = J_W_pinv_;
    jacobian_augmented_inverse_.rightCols(3) = Z_.transpose();

    // ROS_INFO_STREAM("J W PINV: \n" << J_W_pinv_);
    // ROS_INFO_STREAM("Z T: \n" << Z_.transpose());
    // ROS_INFO_STREAM("Jacobian augmented inverse: \n" << jacobian_augmented_inverse_);

    Eigen::VectorXd dotx_e(6);
    dotx_e << x_1, dotx_2_;

    // dot_q_ = jacobian_augmented_.inverse() * dotx_e;
    dot_q_ = jacobian_augmented_inverse_ * dotx_e;

    Eigen::VectorXd dotx_debug(6);
    dotx_debug = jacobian_augmented_ * dot_q_;

    // ROS_INFO_STREAM("dotx debug \n" << dotx_debug);
    dotx_e_print_ = dotx_e;

}

Eigen::Vector3d NullDamping::computeOptimization(){
    // Optimization problem for a single N ports tank
    set_defaults();  // Set basic algorithm parameters.
    setup_indexing();

    for(int i=0; i < 3; i++)
         params.Fd[i] = F_d_[i];

    params.tau[0] = cycle_time;
    params.Td[0] = TANK_DES_VALUE;
    params.T0[0] = tank_energy_;
    
    settings.verbose = 0;
    settings.max_iters = 50;
    long num_iters = solve();

    Eigen::Vector3d dotx2;

    for(int i= 0; i < dotx2.size(); i++)
        dotx2[i] = vars.dotx_2[i];

    return dotx2;

}

void NullDamping::saturateDynamics(){
    double dotq_max = 0.5;
    double ddotq_max = 0.25 * cycle_time; 

    //! By saturating, the motion isn't in the null space anymore
    // for(int i = 0; i < 6; i++){
    //     if(dot_q_[i] >= dotq_max)
    //         dot_q_[i] = dotq_max;
    //     else if(dot_q_[i] <= -dotq_max)
    //         dot_q_[i] = -dotq_max;
    // }

    // for(int i = 0; i < 6; i++){
    //     if((dot_q_[i] - dot_q_prev_[i]) >= ddotq_max)
    //         dot_q_[i] = dot_q_prev_[i] + ddotq_max;
    //     else if((dot_q_[i] - dot_q_prev_[i]) <= -ddotq_max)
    //         dot_q_[i] = dot_q_prev_[i] - ddotq_max;
    // }
}

void NullDamping::updateTank(){
    // Power injection due to null space refill
    double pow_null_damp = dotx_2_.transpose() * (D_null_+ dot_D_null_) * dotx_2_;
    // Power loss due to inertia variation
    double pow_inertia;
    if(dot_M_(1,1) > 0.0)
        pow_inertia = des_twist_adm.transpose()* dot_M_ * des_twist_adm;
    // Damping power injection
    double pow_damp_inj = des_twist_adm.transpose() * D_a_ * des_twist_adm;
    // Update tank
    tank_energy_ -= pow_inertia; //!!!!!!!!!!!!!!!!!
    tank_energy_ += cycle_time * pow_null_damp;
    // tank_energy_ -= cycle_time * P_np_;
    if(DAMPING_INJECTION_)
        tank_energy_ += cycle_time * pow_damp_inj;        

    ROS_INFO_STREAM("TANK ENERGY: " << tank_energy_);

}

void NullDamping::writeToFiles(){

    tank_file_ << ros::Time::now().toSec() - start_time_;
    dotq_file_ << ros::Time::now().toSec() - start_time_;
    dotx_e_file_ << ros::Time::now().toSec() - start_time_;
    force_file_ << ros::Time::now().toSec() - start_time_;
    null_pow_file_ << ros::Time::now().toSec() - start_time_;

    tank_file_ << " " << tank_energy_;
    tank_file_ << std::endl;

    for(int i = 0; i < 6; i ++){
        dotq_file_ << " " << dot_q_[i];
        dotx_e_file_ << " " << dotx_e_print_[i];
        force_file_ << " " << F_e_[i];
    }

    dotq_file_ << std::endl;
    dotx_e_file_ << std::endl;
    force_file_ << std::endl;

    double pow_null_damp = dotx_2_.transpose() * (D_null_ + dot_D_null_) * dotx_2_;

    null_pow_file_ << " " << pow_null_damp;

    null_pow_file_ << std::endl;

}

void NullDamping::publishVelocity(){
    // Save the speed values for the next cycle
    dot_q_prev_ = dot_q_;

    // ROS_INFO_STREAM("DotQ: \n" << dot_q_);

    // Create a dedicated array and send the velocity message
    std::vector<double> dotq_vec;
    dotq_vec.resize(6);

    std_msgs::Float64MultiArray msg;

    for(int i=0; i< dotq_vec.size(); i++)
        dotq_vec[i] = dot_q_[i];
        
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = dotq_vec.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "vel";

    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), dotq_vec.begin(), dotq_vec.end());

    arm_pose_pub_.publish(msg);

    // Update the force and null speed as the last thing before the next cycle
    F_e_prev_ = F_e_;
    dotx_2_prev_ = dotx_2_;
}

void NullDamping::spin(){
    init();
    computeAdmittance();
    kinematicDecomposition();
    saturateDynamics();
    updateTank();
    writeToFiles();
    publishVelocity();

}
