#include <queue>
#include <ompl/util/Exception.h>
#include "moveit_ompl_components/SubdivisionMotionValidator.h"

using namespace ompl_interface;

SubdivisionMotionValidator::SubdivisionMotionValidator(ompl::base::SpaceInformation *si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

SubdivisionMotionValidator::SubdivisionMotionValidator(const ompl::base::SpaceInformationPtr& si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

SubdivisionMotionValidator::~SubdivisionMotionValidator()
{
}

bool SubdivisionMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
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
        double eps = ss_->getLongestValidSegmentLength();
        std::queue<std::pair<ompl::base::State*, ompl::base::State*> > pos;

        pos.push(std::make_pair(si_->cloneState(s1), si_->cloneState(s2)));

        ompl::base::State *test = si_->allocState();
        do
        {
            std::pair<ompl::base::State*, ompl::base::State*> x = pos.front();
            pos.pop();

            if (ss_->distance(x.first, x.second) > eps)
            //if (ss_->validSegmentCount(x.first, x.second) > 1)
            {
                ss_->interpolate(x.first, x.second, 0.5, test);

                if (ss_->distance(x.first, test) >= ss_->distance(x.first, x.second) ||
                    ss_->distance(test, x.second) >= ss_->distance(x.first, x.second))
                {
                    OMPL_ERROR("Divergence detected");
                    OMPL_ERROR("Starting dist: %f   First seg: %f   Second seg: %f", ss_->distance(x.first, x.second), ss_->distance(x.first, test), ss_->distance(test, x.second));
                    return false;
                }

                result = si_->isValid(test);
                //result = true;
                if (result)
                {
                    pos.push(std::make_pair(si_->cloneState(x.first), si_->cloneState(test)));
                    pos.push(std::make_pair(si_->cloneState(test)   , si_->cloneState(x.second)));
                }
            }

            si_->freeState(x.first);
            si_->freeState(x.second);
        } while (result && !pos.empty());

        // cleanup
        si_->freeState(test);
        while (!pos.empty())
        {
            std::pair<ompl::base::State*, ompl::base::State*> x = pos.front();
            pos.pop();
            si_->freeState(x.first);
            si_->freeState(x.second);
        }
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;

    /*// assume motion starts in a valid configuration so s1 is valid
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    bool result = true;
    int nd = ss_->validSegmentCount(s1, s2);

    // initialize the queue of test positions
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

        // temporary storage for the checked state
        ompl::base::State *test = si_->allocState();

        // repeatedly subdivide the path segment in the middle (and check the middle)
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            ss_->interpolate(s1, s2, (double)mid / (double)nd, test);

            //if (!si_->isValid(test))
            //{
            //    result = false;
            //    break;
            //}

            pos.pop();

            if (x.first < mid)
                pos.push(std::make_pair(x.first, mid - 1));
            if (x.second > mid)
                pos.push(std::make_pair(mid + 1, x.second));
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;*/
}

bool SubdivisionMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                             std::pair<ompl::base::State*, double> &lastValid) const
{
    OMPL_ERROR("checkMotion");
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

void SubdivisionMotionValidator::defaultSettings()
{
    ss_ = si_->getStateSpace().get();
    if (!ss_)
        throw ompl::Exception("No state space for motion validator");
}