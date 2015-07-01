/**
 * @file MobileTreeIk.h
 * @brief Defines the MobileTreeIk class.
 * @author Ross Taylor
 */
#ifndef R2_TREE_IK_H
#define R2_TREE_IK_H

#include "nasa_robodyn_controllers_core/KdlTreeIk.h"

class R2TreeIk : public KdlTreeIk
{
public:
    static const int BASE;

    R2TreeIk();
    ~R2TreeIk();

    void setBases(const std::vector<std::string>& bases_in);

    void resetMobileJoints();

    /**
     * @brief getJoints
     * @param joints_in input joints
     * @param nodeNames input frame names
     * @param nodeFrames input frames
     * @param joints_out output joints
     * @param nodePriorities input node priorities (6 * nodeNames.size())
     */
    virtual void getJointPositions(const KDL::JntArray& joints_in, const std::vector<std::string>& nodeNames,
                   const std::vector<KDL::Frame>& nodeFrames, KDL::JntArray& joints_out,
                   const std::vector<NodePriority>& nodePriorities);

    inline virtual std::string getBaseName() const {return robotModelBase;}
    virtual void getJointNames(std::vector<std::string>& jointNames) const;
    inline virtual unsigned int getJointCount() const { return tree.getNrOfJoints() - 6; }
//    void setJointInertias(const KDL::JntArray &inertia_in);
    virtual void getFrames (const KDL::JntArray& joints_in, std::map<std::string, KDL::Frame>& frameMap);
     void setPriorityTol(const std::map<int, std::pair<double, double> >& priority_tol)
     {
         KdlTreeIk::setPriorityTol(priority_tol);
         priorityTolMap[R2TreeIk::BASE] = std::make_pair(eps, eps);
     }

     // Added by Ryan Luna
     void getMobileJoints(std::vector<double>& mobileJointValues) const
     {
        mobileJointValues.resize(6);
        for (size_t i = 0; i < 6; ++i)
            mobileJointValues[i] = mobileJoints(i);
     }

     // Added by Ryan Luna
     void setMobileJoints(const std::vector<double>& mobileJointValues)
     {
        if (mobileJointValues.size() != 6)
        {
            std::string err("Expected vector of size 6 for setMobileJoints");
            //RCS::Logger::log("gov.nasa.controllers.R2TreeIk", log4cpp::Priority::ERROR, err);
            throw std::runtime_error(err);
        }

        for (size_t i = 0; i < mobileJointValues.size(); ++i)
            mobileJoints(i) = mobileJointValues[i];
     }

protected:
    virtual void initialize();

private:
    KDL::JntArray mobileJoints;
    std::vector<std::string> bases;
    std::string robotModelBase;

};

#endif
