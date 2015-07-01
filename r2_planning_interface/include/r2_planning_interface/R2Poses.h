#ifndef R2_PLANNING_R2_POSES_H_
#define R2_PLANNING_R2_POSES_H_

// default joint values for all joints, in radians
static void defaultPose(std::map<std::string, double>& joints)
{
    joints["r2/left_arm/joint0"] = 0;
    joints["r2/left_arm/joint1"] = 0;
    joints["r2/left_arm/joint2"] = 0;
    joints["r2/left_arm/joint3"] = 0;
    joints["r2/left_arm/joint4"] = 0;

    joints["r2/right_arm/joint0"] = 0;
    joints["r2/right_arm/joint1"] = 0;
    joints["r2/right_arm/joint2"] = 0;
    joints["r2/right_arm/joint3"] = 0;
    joints["r2/right_arm/joint4"] = 0;

    joints["r2/left_leg/joint0"] = 0;
    joints["r2/left_leg/joint1"] = 0;
    joints["r2/left_leg/joint2"] = 0;
    joints["r2/left_leg/joint3"] = 0;
    joints["r2/left_leg/joint4"] = 0;
    joints["r2/left_leg/joint5"] = 0;
    joints["r2/left_leg/joint6"] = 0;
    joints["r2/left_leg/gripper/joint0"] = 0;
    joints["r2/left_leg/gripper/jawLeft"] = 0;
    joints["r2/left_leg/gripper/jawRight"] = 0;

    joints["r2/right_leg/joint0"] = 0;
    joints["r2/right_leg/joint1"] = 0;
    joints["r2/right_leg/joint2"] = 0;
    joints["r2/right_leg/joint3"] = 0;
    joints["r2/right_leg/joint4"] = 0;
    joints["r2/right_leg/joint5"] = 0;
    joints["r2/right_leg/joint6"] = 0;
    joints["r2/right_leg/gripper/joint0"] = 0;
    joints["r2/right_leg/gripper/jawLeft"] = 0;
    joints["r2/right_leg/gripper/jawRight"] = 0;

    joints["r2/waist/joint0"] = 0;

    joints["r2/neck/joint0"] = 0;
    joints["r2/neck/joint1"] = 0;
    joints["r2/neck/joint2"] = 0;

    joints["r2/left_arm/hand/index/yaw"] = 0;
    joints["r2/left_arm/hand/index/proximal"] = 0;
    joints["r2/left_arm/hand/index/medial"] = 0;
    joints["r2/left_arm/hand/index/distal"] = 0;
    joints["r2/left_arm/hand/middle/yaw"] = 0;
    joints["r2/left_arm/hand/middle/proximal"] = 0;
    joints["r2/left_arm/hand/middle/medial"] = 0;
    joints["r2/left_arm/hand/middle/distal"] = 0;
    joints["r2/left_arm/hand/ringlittle/little"] = 0;
    joints["r2/left_arm/hand/ringlittle/ring"] = 0;
    joints["r2/left_arm/hand/ringlittle/ringMedial"] = 0;
    joints["r2/left_arm/hand/ringlittle/ringDistal"] = 0;
    joints["r2/left_arm/hand/ringlittle/littleMedial"] = 0;
    joints["r2/left_arm/hand/ringlittle/littleDistal"] = 0;
    joints["r2/left_arm/hand/thumb/roll"] = 0;
    joints["r2/left_arm/hand/thumb/proximal"] = 0;
    joints["r2/left_arm/hand/thumb/medial"] = 0;
    joints["r2/left_arm/hand/thumb/distal"] = 0;

    joints["r2/left_arm/wrist/pitch"] = 0;
    joints["r2/left_arm/wrist/yaw"] = 0;

    joints["r2/right_arm/hand/index/yaw"] = 0;
    joints["r2/right_arm/hand/index/proximal"] = 0;
    joints["r2/right_arm/hand/index/medial"] = 0;
    joints["r2/right_arm/hand/index/distal"] = 0;
    joints["r2/right_arm/hand/middle/yaw"] = 0;
    joints["r2/right_arm/hand/middle/proximal"] = 0;
    joints["r2/right_arm/hand/middle/medial"] = 0;
    joints["r2/right_arm/hand/middle/distal"] = 0;
    joints["r2/right_arm/hand/ringlittle/little"] = 0;
    joints["r2/right_arm/hand/ringlittle/ring"] = 0;
    joints["r2/right_arm/hand/ringlittle/ringMedial"] = 0;
    joints["r2/right_arm/hand/ringlittle/ringDistal"] = 0;
    joints["r2/right_arm/hand/ringlittle/littleMedial"] = 0;
    joints["r2/right_arm/hand/ringlittle/littleDistal"] = 0;
    joints["r2/right_arm/hand/thumb/roll"] = 0;
    joints["r2/right_arm/hand/thumb/proximal"] = 0;
    joints["r2/right_arm/hand/thumb/medial"] = 0;
    joints["r2/right_arm/hand/thumb/distal"] = 0;

    joints["r2/right_arm/wrist/pitch"] = 0;
    joints["r2/right_arm/wrist/yaw"] = 0;
}

// default joint values for all joints, in radians
static void crouchingLegsPose(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -1.1648;
    joints["r2/left_leg/joint1"] = 0.0680;
    joints["r2/left_leg/joint2"] = -2.9297;
    joints["r2/left_leg/joint3"] = 2.3740;
    joints["r2/left_leg/joint4"] = 0.0;
    joints["r2/left_leg/joint5"] = 1.2759;
    joints["r2/left_leg/joint6"] = 0.0;


    joints["r2/right_leg/joint0"] = 1.1648;
    joints["r2/right_leg/joint1"] = 0.0680;
    joints["r2/right_leg/joint2"] = 2.9297;
    joints["r2/right_leg/joint3"] = 2.3740;
    joints["r2/right_leg/joint4"] = 0.0;
    joints["r2/right_leg/joint5"] = 1.2759;
    joints["r2/right_leg/joint6"] = 0.0;
}

static void readyArmPose(std::map<std::string, double>& jointVals)
{
    double TORAD = 3.14159265/180.0;

    jointVals["r2/left_arm/joint0"] = 50 * TORAD;
    jointVals["r2/left_arm/joint1"] = -80 * TORAD;
    jointVals["r2/left_arm/joint2"] = -105 * TORAD;
    jointVals["r2/left_arm/joint3"] = -140 * TORAD;
    jointVals["r2/left_arm/joint4"] = 80 * TORAD;
    jointVals["r2/left_arm/wrist/pitch"] = 0;
    jointVals["r2/left_arm/wrist/yaw"] = 0;

    jointVals["r2/right_arm/joint0"] = -50 * TORAD;
    jointVals["r2/right_arm/joint1"] = -80 * TORAD;
    jointVals["r2/right_arm/joint2"] = 105 * TORAD;
    jointVals["r2/right_arm/joint3"] = -140 * TORAD;
    jointVals["r2/right_arm/joint4"] = -80 * TORAD;
    jointVals["r2/right_arm/wrist/pitch"] = 0;
    jointVals["r2/right_arm/wrist/yaw"] = 0;
}

static void issLegsPose(std::map<std::string, double>& jointVals)
{
    double TORAD = 3.14159265/180.0;

    jointVals["r2/left_leg/joint0"] = -66.73 * TORAD;
    jointVals["r2/left_leg/joint1"] = 3.89 * TORAD;
    jointVals["r2/left_leg/joint2"] = -165.83 * TORAD;
    jointVals["r2/left_leg/joint3"] = 136.02 * TORAD;
    jointVals["r2/left_leg/joint4"] = 4.04 * TORAD;
    jointVals["r2/left_leg/joint5"] = 101.86 * TORAD;

    jointVals["r2/right_leg/joint0"] = 66.73 * TORAD;
    jointVals["r2/right_leg/joint1"] = 3.89 * TORAD;
    jointVals["r2/right_leg/joint2"] = 162.30 * TORAD;
    jointVals["r2/right_leg/joint3"] = 136.02 * TORAD;
    jointVals["r2/right_leg/joint4"] = -4.04 * TORAD;
    jointVals["r2/right_leg/joint5"] = 101.86 * TORAD;
}

static void issLegsUnstowPose(std::map<std::string, double>& jointVals)
{
    // R2 pirouette
    // jointVals["r2/left_leg/joint0"] = 0.0;
    // jointVals["r2/left_leg/joint1"] = -0.9502;
    // jointVals["r2/left_leg/joint2"] = 1.5531;
    // jointVals["r2/left_leg/joint3"] = 2.2328;
    // jointVals["r2/left_leg/joint4"] = 0.0;
    // jointVals["r2/left_leg/joint5"] = 1.3229;
    // jointVals["r2/left_leg/joint6"] = 0.0;

    // jointVals["r2/right_leg/joint0"] = 0.0;
    // jointVals["r2/right_leg/joint1"] = -0.9502;
    // jointVals["r2/right_leg/joint2"] = -1.5531;
    // jointVals["r2/right_leg/joint3"] = 2.2328;
    // jointVals["r2/right_leg/joint4"] = 0.0;
    // jointVals["r2/right_leg/joint5"] = 1.3229;
    // jointVals["r2/right_leg/joint6"] = 0.0;

    jointVals["r2/left_leg/joint0"] = 0.0;
    jointVals["r2/left_leg/joint1"] = 0.0;
    jointVals["r2/left_leg/joint2"] = -2.7885;
    jointVals["r2/left_leg/joint3"] = 2.3897;
    jointVals["r2/left_leg/joint4"] = 0.0;
    jointVals["r2/left_leg/joint5"] = 2.4368;
    jointVals["r2/left_leg/joint6"] = -1.9061;

    jointVals["r2/right_leg/joint0"] = 0.0;
    jointVals["r2/right_leg/joint1"] = 0.0;
    jointVals["r2/right_leg/joint2"] = 2.7885;
    jointVals["r2/right_leg/joint3"] = 2.3897;
    jointVals["r2/right_leg/joint4"] = 0.0;
    jointVals["r2/right_leg/joint5"] = 2.4368;
    jointVals["r2/right_leg/joint6"] = 1.9061;
}

// World joint transform for ISS Legs pose
// static void issLegsWorldTransform(Eigen::Affine3d& transform)
// {
//     transform.translation()(0) = -1.6;
//     transform.translation()(1) = 0.185;
//     transform.translation()(2) = 0.855;

//     transform = Eigen::Translation3d(transform.translation()) *
//                (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
//                 Eigen::AngleAxisd(0.52359, Eigen::Vector3d::UnitY()) *
//                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
// }

// World joint transform for ISS Legs pose
static void issLegsWorldTransform(Eigen::Affine3d& transform)
{
    transform = Eigen::Translation3d(0,0,0) *
              (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(1.57079, Eigen::Vector3d::UnitZ()));

    // R2 pirouette in front of closet
    // transform.translation()(0) = -2.65;
    // transform.translation()(1) = -0.835;
    // transform.translation()(2) = 0.0225;

    // R2 crouch forward in front of closet
    transform.translation()(0) = -2.70;
    transform.translation()(1) = -0.45;
    transform.translation()(2) = -0.22;
}

#endif