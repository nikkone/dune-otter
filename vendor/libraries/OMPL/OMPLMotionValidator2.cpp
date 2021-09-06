#include "OMPLMotionValidator2.hpp"
#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;


void MotionPlanners::OMPL::ChartsDBMotionValidator2::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr)
        throw ompl::Exception("No state space for motion validator");
}

bool MotionPlanners::OMPL::ChartsDBMotionValidator2::checkMotion(const ob::State *s1, const ob::State *s2,
                                                      std::pair<ob::State *, double> &lastValid) const
{
    
    // If lastValid.first is nullPtr, then the exact state is dontcare? OMPL discreteMotionValidator also does this.
    // assume motion starts in a valid configuration so s1 is valid 
    //std::cout << "1";
    if (lastValid.first == nullptr) {
        return checkMotion(s1,s2);
    }
    bool result = false;
    if(!transectSafety(s1,s2)) {
        bool partresult = false; // temporary storage for the checked state
        ob::State *test = si_->allocState();
        double scale = 0.5;
        for(unsigned j=2;j<binDepth;j++) {
            stateSpace_->interpolate(s1, s2, scale, test);
            if(transectSafety(s1,test)) { // Could be optimized with checking from previous valid instead of S1
                lastValid.second = scale;
                stateSpace_->copyState(lastValid.first, test);
                partresult = true;
                scale += 1.0/pow(2.0,j);
            } else {
                scale -= 1.0/pow(2.0,j);
            }

        }
        si_->freeState(test);
        if(!partresult) { // No valid point on the line between s1 to s2 could be found
            lastValid.second = 0.0;
            stateSpace_->copyState(lastValid.first, s1);
        }

    } else {
        result=true;
    }


    if (result)
        valid_++;
    else
        invalid_++;
    return result;
}

bool MotionPlanners::OMPL::ChartsDBMotionValidator2::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    // TODO: check if s2 check speeds up?.
    /* assume motion starts in a valid configuration so s1 is valid */
    /*if (!si_->isValid(s2))
    */
       //std::cout << "0"<< std::endl;
    if(transectSafety(s1, s2)) {
        valid_++;
        return true;
    } else {
        invalid_++;
        return false;
    }
}

bool MotionPlanners::OMPL::ChartsDBMotionValidator2::transectSafety(const ob::State *s1, const ob::State *s2) const {

    const auto *state1 = static_cast<const ob::RealVectorStateSpace::StateType *>(s1);
    const auto *state2 = static_cast<const ob::RealVectorStateSpace::StateType *>(s2);
    return dbcon->run(state1->values[1], state1->values[0], state2->values[1], state2->values[0]) == 0;
}