#include <queue>
#include <ompl/util/Exception.h>
#include "moveit_ompl_components/LinearMotionValidator.h"

using namespace ompl_interface;

LinearMotionValidator::LinearMotionValidator(ompl::base::SpaceInformation *si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

LinearMotionValidator::LinearMotionValidator(const ompl::base::SpaceInformationPtr& si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

LinearMotionValidator::~LinearMotionValidator()
{
}

bool LinearMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    // assume motion starts in a valid configuration (s1 is valid)
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    bool result = true;
    int nd = ss_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ompl::base::State *test = si_->allocState();

        for (int j = 1; j < nd && result; ++j)
        {
            ss_->interpolate(s1, s2, (double)j / (double)nd, test);
            result = si_->isValid(test);
        }
        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool LinearMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                        std::pair<ompl::base::State*, double> &lastValid) const
{
    // assume motion starts in a valid configuration (s1 is valid)
    bool result = true;
    int nd = ss_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        // temporary storage for the checked state
        ompl::base::State *test = si_->allocState();

        for (int j = 1 ; j < nd ; ++j)
        {
            ss_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first)
                    ss_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first)
                ss_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

void LinearMotionValidator::defaultSettings()
{
    ss_ = si_->getStateSpace().get();
    if (!ss_)
        throw ompl::Exception("No state space for motion validator");
}