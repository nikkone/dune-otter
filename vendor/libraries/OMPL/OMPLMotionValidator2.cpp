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
    bool result = false;
    if(!transectSafety(s1,s2)) {
        bool partresult = false; // temporary storage for the checked state
        ob::State *test = si_->allocState();
        double scale = 0.5;
        //unsigned maxBin=6; // "Binary search" on line TODO: This variable should be made accessible through class interface
        for(unsigned j=2;j<binDepth;j++) {
            stateSpace_->interpolate(s1, s2, scale, test);
            if(transectSafety(s1,test)) {
                lastValid.second = scale;
                if (lastValid.first != nullptr) {
                    stateSpace_->copyState(lastValid.first, test);
                }
                partresult = true;
                scale += 1.0/pow(2.0,j);
                //break;
            } else {
                scale -= 1.0/pow(2.0,j);
            }

        }
        si_->freeState(test);
        if(!partresult) {
            lastValid.second = 0.0;
            if (lastValid.first != nullptr) {
                stateSpace_->copyState(lastValid.first, s1);
            }
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

//Works, but monotonically decreasing bin search only
/*bool MotionPlanners::OMPL::ChartsDBMotionValidator2::checkMotion(const ob::State *s1, const ob::State *s2,
                                                      std::pair<ob::State *, double> &lastValid) const
{
    // If lastValid.first is nullPtr, then the exact state is dontcare? OMPL discreteMotionValidator also does this.
    // assume motion starts in a valid configuration so s1 is valid 
    bool result = false;
    if(!transectSafety(s1,s2)) {
        bool partresult = false; // temporary storage for the checked state
        ob::State *test = si_->allocState();
        //unsigned maxBin=5; // "Binary search" on line TODO: This variable should be made accessible through class interface
        for(unsigned j=1;j<binDepth;j++) {
            stateSpace_->interpolate(s1, s2, 1.0/pow(2.0,j), test);
            if(transectSafety(s1,test)) {
                lastValid.second = 1.0/pow(2.0,j);
                if (lastValid.first != nullptr) {
                    stateSpace_->copyState(lastValid.first, test);
                }
                partresult = true;
                break;
            }
        }
        si_->freeState(test);
        if(!partresult) {
            lastValid.second = 0.0;
            if (lastValid.first != nullptr) {
                stateSpace_->copyState(lastValid.first, s1);
            }
        }

    } else {
        result=true;
    }


    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}*/


/*
// Possible reason for this not working great:
https://locationtech.github.io/jts/javadoc/org/locationtech/jts/algorithm/package-summary.html
https://locationtech.github.io/jts/javadoc/org/locationtech/jts/algorithm/Intersection.html
" In general it is not possible to accurately compute the intersection point of two lines, due to numerical roundoff. This is particularly true when the input lines are nearly parallel. This routine uses numerical conditioning on the input values to ensure that the computed value should be very close to the correct value."
bool MotionPlanners::OMPL::ChartsDBMotionValidator2::checkMotion(const ob::State *s1, const ob::State *s2,
                                                      std::pair<ob::State *, double> &lastValid) const
{
    const auto *state1 = static_cast<const ob::RealVectorStateSpace::StateType *>(s1);
    const auto *state2 = static_cast<const ob::RealVectorStateSpace::StateType *>(s2);
    if(dbcon->checkTransectLanding(state1->values[1], state1->values[0], state2->values[1], state2->values[0]) == 0) {
        valid_++;
        return true;
    }
    // assume motion starts in a valid configuration so s1 is valid
    double lat=0.0,lon=0.0,time=0.0;

    dbcon->findFirstIntersectingPoint(state1->values[1], state1->values[0], state2->values[1], state2->values[0], lat, lon, time);
    lastValid.second = time;
    if (lastValid.first != nullptr) {
        if(lastValid.second > 0.0) {
            //std::cout << state1->values[1] << "," << state1->values[0] << "," << state2->values[1] << "," << state2->values[0] << "," << lat << "," << lon << "," << time << std::endl;
            auto *rstate = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(lastValid.first);
            rstate->values[1]=lat;
            rstate->values[0]=lon;
        } else {
            stateSpace_->copyState(lastValid.first, s1);
            lastValid.second = 0.0;
        }
    }
    invalid_++;
    return false;
}*/


bool MotionPlanners::OMPL::ChartsDBMotionValidator2::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    // TODO: check if s2 check speeds up?.
    /* assume motion starts in a valid configuration so s1 is valid */
    /*if (!si_->isValid(s2))
    */
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