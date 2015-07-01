#ifndef ARGOS_POSES_H_
#define ARGOS_POSES_H_

#include <map>
#include <boost/function.hpp>
#include <Eigen/Dense>

static const double TORAD = 3.14159265/180.0;


static void readyArmPose(std::map<std::string, double>& jointVals)
{
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

static void stowArmPose(std::map<std::string, double>& jointVals)
{
    jointVals["r2/left_arm/joint0"] = -15.0 * TORAD;
    jointVals["r2/left_arm/joint1"] = -85.0 * TORAD;
    jointVals["r2/left_arm/joint2"] = -65.0 * TORAD;
    jointVals["r2/left_arm/joint3"] = -155.0 * TORAD;
    jointVals["r2/left_arm/joint4"] = 35.0 * TORAD;
    jointVals["r2/left_arm/wrist/pitch"] = 0;
    jointVals["r2/left_arm/wrist/yaw"] = 0;

    jointVals["r2/right_arm/joint0"] = 15.0 * TORAD;
    jointVals["r2/right_arm/joint1"] = -85.0 * TORAD;
    jointVals["r2/right_arm/joint2"] = 65.0 * TORAD;
    jointVals["r2/right_arm/joint3"] = -155.0 * TORAD;
    jointVals["r2/right_arm/joint4"] = -35.0 * TORAD;
    jointVals["r2/right_arm/wrist/pitch"] = 0;
    jointVals["r2/right_arm/wrist/yaw"] = 0;
}

// static void issLegsStowPose(std::map<std::string, double>& jointVals)
// {
//     jointVals["r2/left_leg/joint0"] = 12.72 * TORAD;
//     jointVals["r2/left_leg/joint1"] = -1.22 * TORAD;
//     jointVals["r2/left_leg/joint2"] = -169.63 * TORAD;
//     jointVals["r2/left_leg/joint3"] = 123.31 * TORAD;
//     jointVals["r2/left_leg/joint4"] = -2.02 * TORAD;
//     jointVals["r2/left_leg/joint5"] = 127.04 * TORAD;
//     jointVals["r2/left_leg/joint6"] = -1.9061 * TORAD;

//     jointVals["r2/right_leg/joint0"] = -16.42 * TORAD;
//     jointVals["r2/right_leg/joint1"] = -5.33 * TORAD;
//     jointVals["r2/right_leg/joint2"] = -171.97 * TORAD;
//     jointVals["r2/right_leg/joint3"] = 117.81 * TORAD;
//     jointVals["r2/right_leg/joint4"] = -8.6 * TORAD;
//     jointVals["r2/right_leg/joint5"] = 123.65 * TORAD;
//     jointVals["r2/right_leg/joint6"] = -95.77 * TORAD;
//     //jointVals["r2/right_leg/joint6"] = 84.23 * TORAD;

//     //jointVals["r2/right_leg/joint6"] = -95.77 * TORAD;
// }

static void curlLeftLeg(std::map<std::string, double>& jointVals)
{
    jointVals["r2/left_leg/joint1"] = -10.0 * TORAD;
    jointVals["r2/left_leg/joint3"] = 155.0 * TORAD;
    jointVals["r2/left_leg/joint5"] = 155.04 * TORAD;
}

// Initial configuration after unstow and left leg curl
static void issLegsUnstowPose(std::map<std::string, double>& jointVals)
{
    jointVals["r2/left_leg/joint0"] = 15.79 * TORAD;
    jointVals["r2/left_leg/joint1"] = -30.01 * TORAD;
    jointVals["r2/left_leg/joint2"] = -0.01 * TORAD;
    jointVals["r2/left_leg/joint3"] = 153.38 * TORAD;
    jointVals["r2/left_leg/joint4"] = -1.36 * TORAD;
    jointVals["r2/left_leg/joint5"] = 153.76 * TORAD;
    jointVals["r2/left_leg/joint6"] = -98.46 * TORAD;

    jointVals["r2/right_leg/joint0"] = -7.4 * TORAD;
    jointVals["r2/right_leg/joint1"] = -26.72 * TORAD;
    jointVals["r2/right_leg/joint2"] = -88.84 * TORAD;
    jointVals["r2/right_leg/joint3"] = 145.95 * TORAD;
    jointVals["r2/right_leg/joint4"] = -6.46 * TORAD;
    jointVals["r2/right_leg/joint5"] = 119.85 * TORAD;
    jointVals["r2/right_leg/joint6"] = -94.8 * TORAD;
}

static void issLegsStowPose(std::map<std::string, double>& jointVals)
{
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

// default joint values for all joints, in radians
static void defaultPose(std::map<std::string, double>& joints)
{
    // zero for all finger joints
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

    // waist and head are all zero
    joints["r2/waist/joint0"] = 0;
    joints["r2/neck/joint0"] = 0;
    joints["r2/neck/joint1"] = 0;
    joints["r2/neck/joint2"] = 0;

    readyArmPose(joints);
    issLegsUnstowPose(joints);
}

// Step 1: goal
static void leftOverHR1RightAttachedHR0(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -40.11 * TORAD;
    joints["r2/left_leg/joint1"] = -5.31  * TORAD;
    joints["r2/left_leg/joint2"] = -24.07 * TORAD;
    joints["r2/left_leg/joint3"] = 96.94 * TORAD;
    joints["r2/left_leg/joint4"] = 26.72 * TORAD;
    joints["r2/left_leg/joint5"] = 130.77 * TORAD;
    joints["r2/left_leg/joint6"] = -35.63 * TORAD;

    joints["r2/right_leg/joint0"] = -21.33 * TORAD;
    joints["r2/right_leg/joint1"] = -27.21 * TORAD;
    joints["r2/right_leg/joint2"] = -115.58 * TORAD;
    joints["r2/right_leg/joint3"] = 130.87 * TORAD;
    joints["r2/right_leg/joint4"] = -32.64 * TORAD;
    joints["r2/right_leg/joint5"] = 108.28 * TORAD;
    joints["r2/right_leg/joint6"] = -73.53 * TORAD;
}

// After step 1, move left leg to grab first handrail on right
static void leftAttachedHR1RightAttachedHR0(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -38.05 * TORAD;
    joints["r2/left_leg/joint1"] = -8.7  * TORAD;
    joints["r2/left_leg/joint2"] = -45.38 * TORAD;
    joints["r2/left_leg/joint3"] = 81.9 * TORAD;
    joints["r2/left_leg/joint4"] = 19.57 * TORAD;
    joints["r2/left_leg/joint5"] = 105.92 * TORAD;
    joints["r2/left_leg/joint6"] = -26.9 * TORAD;

    // right leg joints should be very close to leftOverHR1RightAttachedHR0
    joints["r2/right_leg/joint0"] = -21.24 * TORAD;
    joints["r2/right_leg/joint1"] = -27.05 * TORAD;
    joints["r2/right_leg/joint2"] = -115.48 * TORAD;
    joints["r2/right_leg/joint3"] = 130.96 * TORAD;
    joints["r2/right_leg/joint4"] = -32.67 * TORAD;
    joints["r2/right_leg/joint5"] = 108.26 * TORAD;
    joints["r2/right_leg/joint6"] = -73.54 * TORAD;
}

// Step 2: start
static void leftAttachedHR1RightUpHR0(std::map<std::string, double>& joints)
{
    // left leg joints should be very close to leftAttachedHR1RightAttachedHR0
    joints["r2/left_leg/joint0"] = -38.06 * TORAD;
    joints["r2/left_leg/joint1"] = -8.68  * TORAD;
    joints["r2/left_leg/joint2"] = -45.4 * TORAD;
    joints["r2/left_leg/joint3"] = 81.88 * TORAD;
    joints["r2/left_leg/joint4"] = 19.57 * TORAD;
    joints["r2/left_leg/joint5"] = 105.99 * TORAD;
    joints["r2/left_leg/joint6"] = -26.8 * TORAD;

    joints["r2/right_leg/joint0"] = -29.04 * TORAD;
    joints["r2/right_leg/joint1"] = -21.31 * TORAD;
    joints["r2/right_leg/joint2"] = -125.65 * TORAD;
    joints["r2/right_leg/joint3"] = 144.04 * TORAD;
    joints["r2/right_leg/joint4"] = -49.72 * TORAD;
    joints["r2/right_leg/joint5"] = 129.82 * TORAD;
    joints["r2/right_leg/joint6"] = -82.02 * TORAD;
}

// Step 2: goal
static void leftAttachedHR1RightOverHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -32.78 * TORAD;
    joints["r2/left_leg/joint1"] = -2.85  * TORAD;
    joints["r2/left_leg/joint2"] = -40.19 * TORAD;
    joints["r2/left_leg/joint3"] = 87.75 * TORAD;
    joints["r2/left_leg/joint4"] = 16.63 * TORAD;
    joints["r2/left_leg/joint5"] = 111.33 * TORAD;
    joints["r2/left_leg/joint6"] = 68.91 * TORAD;

    joints["r2/right_leg/joint0"] = 48.99 * TORAD;
    joints["r2/right_leg/joint1"] = -60.08 * TORAD;
    joints["r2/right_leg/joint2"] = -53.09 * TORAD;
    joints["r2/right_leg/joint3"] = 98.8 * TORAD;
    joints["r2/right_leg/joint4"] = 62.49 * TORAD;
    joints["r2/right_leg/joint5"] = 92.01 * TORAD;
    joints["r2/right_leg/joint6"] = 35.64 * TORAD;
}

// After step 2, attach right to HR2
static void leftAttachedHR1RightAttachedHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -32.79 * TORAD;
    joints["r2/left_leg/joint1"] = -2.84  * TORAD;
    joints["r2/left_leg/joint2"] = -40.19 * TORAD;
    joints["r2/left_leg/joint3"] = 87.21 * TORAD;
    joints["r2/left_leg/joint4"] = 16.63 * TORAD;
    joints["r2/left_leg/joint5"] = 110.78 * TORAD;
    joints["r2/left_leg/joint6"] = 69.08 * TORAD;

    joints["r2/right_leg/joint0"] = 46.77 * TORAD;
    joints["r2/right_leg/joint1"] = -55.78 * TORAD;
    joints["r2/right_leg/joint2"] = -62.09 * TORAD;
    joints["r2/right_leg/joint3"] = 76.24 * TORAD;
    joints["r2/right_leg/joint4"] = 65.34 * TORAD;
    joints["r2/right_leg/joint5"] = 71.87 * TORAD;
    joints["r2/right_leg/joint6"] = 20.89 * TORAD;
}

// Step 3: start
static void leftUpHR1RightAttachedHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -36.19 * TORAD;
    joints["r2/left_leg/joint1"] = -4.36  * TORAD;
    joints["r2/left_leg/joint2"] = -36.15 * TORAD;
    joints["r2/left_leg/joint3"] = 96.84 * TORAD;
    joints["r2/left_leg/joint4"] = 18.07 * TORAD;
    joints["r2/left_leg/joint5"] = 124.11 * TORAD;
    joints["r2/left_leg/joint6"] = 70.3 * TORAD;

    joints["r2/right_leg/joint0"] = 44.2 * TORAD;
    joints["r2/right_leg/joint1"] = -55.87 * TORAD;
    joints["r2/right_leg/joint2"] = -58.13 * TORAD;
    joints["r2/right_leg/joint3"] = 75.29 * TORAD;
    joints["r2/right_leg/joint4"] = 65.38 * TORAD;
    joints["r2/right_leg/joint5"] = 72.02 * TORAD;
    joints["r2/right_leg/joint6"] = 21.74 * TORAD;
}

// Step 3: goal  (wall)
static void leftOverHR3RightAttachedHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -47.93 * TORAD;
    joints["r2/left_leg/joint1"] = 17.1  * TORAD;
    joints["r2/left_leg/joint2"] = -98.77 * TORAD;
    joints["r2/left_leg/joint3"] = 112.89 * TORAD;
    joints["r2/left_leg/joint4"] = -44.71 * TORAD;
    joints["r2/left_leg/joint5"] = 109.49 * TORAD;
    joints["r2/left_leg/joint6"] = 96.31 * TORAD;

    joints["r2/right_leg/joint0"] = -35.26 * TORAD;
    joints["r2/right_leg/joint1"] = -0.05 * TORAD;
    joints["r2/right_leg/joint2"] = 49.01 * TORAD;
    joints["r2/right_leg/joint3"] = 111.73 * TORAD;
    joints["r2/right_leg/joint4"] = 23.60 * TORAD;
    joints["r2/right_leg/joint5"] = 86.9 * TORAD;
    joints["r2/right_leg/joint6"] = -44.14 * TORAD;
}

// After step 3, left leg grab handrail
// TODO: This is exactly the same as above.  One of them is wrong
static void leftAttachedHR3RightAttachedHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -47.93 * TORAD;
    joints["r2/left_leg/joint1"] = 17.1  * TORAD;
    joints["r2/left_leg/joint2"] = -98.77 * TORAD;
    joints["r2/left_leg/joint3"] = 112.89 * TORAD;
    joints["r2/left_leg/joint4"] = -44.71 * TORAD;
    joints["r2/left_leg/joint5"] = 109.49 * TORAD;
    joints["r2/left_leg/joint6"] = 96.31 * TORAD;

    joints["r2/right_leg/joint0"] = -35.26 * TORAD;
    joints["r2/right_leg/joint1"] = -0.05 * TORAD;
    joints["r2/right_leg/joint2"] = 49.01 * TORAD;
    joints["r2/right_leg/joint3"] = 111.73 * TORAD;
    joints["r2/right_leg/joint4"] = 23.60 * TORAD;
    joints["r2/right_leg/joint5"] = 86.9 * TORAD;
    joints["r2/right_leg/joint6"] = -44.14 * TORAD;
}

// After step 4: left leg grab second handrail on wall
static void leftAttachedHR4RightAttachedHR2(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -39.43 * TORAD;
    joints["r2/left_leg/joint1"] = -5.14  * TORAD;
    joints["r2/left_leg/joint2"] = 77.44 * TORAD;
    joints["r2/left_leg/joint3"] = 36.39 * TORAD;
    joints["r2/left_leg/joint4"] = 53.68 * TORAD;
    joints["r2/left_leg/joint5"] = 75.02 * TORAD;
    joints["r2/left_leg/joint6"] = -124.33 * TORAD;

    joints["r2/right_leg/joint0"] = -40.22 * TORAD;
    joints["r2/right_leg/joint1"] = 25.4 * TORAD;
    joints["r2/right_leg/joint2"] = 1.14 * TORAD;
    joints["r2/right_leg/joint3"] = 125.73 * TORAD;
    joints["r2/right_leg/joint4"] = -20.74 * TORAD;
    joints["r2/right_leg/joint5"] = 83.69 * TORAD;
    joints["r2/right_leg/joint6"] = -43.41 * TORAD;
}

// Step 5: Goal
static void leftAttachedHR4RightOverHR5(std::map<std::string, double>& joints)
{
    joints["r2/left_leg/joint0"] = -97.6 * TORAD;
    joints["r2/left_leg/joint1"] = -70.36  * TORAD;
    joints["r2/left_leg/joint2"] = 141.08 * TORAD;
    joints["r2/left_leg/joint3"] = 99.23 * TORAD;
    joints["r2/left_leg/joint4"] = 46.25 * TORAD;
    joints["r2/left_leg/joint5"] = 68.7 * TORAD;
    joints["r2/left_leg/joint6"] = -133.82 * TORAD;

    joints["r2/right_leg/joint0"] = 68.33 * TORAD;
    joints["r2/right_leg/joint1"] = -14.4 * TORAD;
    joints["r2/right_leg/joint2"] = 13.48 * TORAD;
    joints["r2/right_leg/joint3"] = 56.83 * TORAD;
    joints["r2/right_leg/joint4"] = -16.69 * TORAD;
    joints["r2/right_leg/joint5"] = 116.75 * TORAD;
    joints["r2/right_leg/joint6"] = -79.17 * TORAD;
}

// World joint transform for ISS Legs pose after unstow routine
static void issLegsUnstowWorldTransform(Eigen::Affine3d& transform)
{
    // Unstow the right way...
    // transform = Eigen::Translation3d(0,0,0) *
    //           (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    //            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    //            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

    transform = Eigen::Affine3d::Identity();

    transform.translation()(0) = -2.87;
    transform.translation()(1) = -0.90;
    transform.translation()(2) = -0.15;
}

// The stow position for R2.  Facing the closet, but upright since no roll/pitch in ARGOS
static void issLegsStowWorldTransformARGOS(Eigen::Affine3d& transform)
{
    // Real stow has a roll that cannot be achieved in ARGOS
    // transform = Eigen::Translation3d(0,0,0) *
    //           (Eigen::AngleAxisd(-10.0 * TORAD, Eigen::Vector3d::UnitX()) *
    //            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    //            Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));

    // // R2 crouch forward in front of closet
    // transform.translation()(0) = -2.80;
    // transform.translation()(1) = -1.275;
    // transform.translation()(2) = -0.15;


    // Unstow the right way...
    transform = Eigen::Translation3d(0,0,0) *
              (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));

    // R2 crouch forward in front of closet
    transform.translation()(0) = -2.90;
    transform.translation()(1) = -1.275;
    transform.translation()(2) = -0.15;
}

#endif