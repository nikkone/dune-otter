#ifndef DUNE_CHARTSDATABASE_OMPL_MOTION_VALIDATOR2_
#define DUNE_CHARTSDATABASE_OMPL_MOTION_VALIDATOR2_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
//#include <USER/ChartsDatabase/Connection.hpp>
#include <ENCGIS/DBconnection.hpp>

 namespace ob = ompl::base;


namespace MotionPlanners
{
  //! @author Nikolai Lauvås
  namespace OMPL
  {
        /** \brief A motion validator that ... */
        class ChartsDBMotionValidator2 : public ob::MotionValidator
        {
        public:
            /** \brief Constructor */
            ChartsDBMotionValidator2(ob::SpaceInformation *si, ENCGIS::lineIntersectLayerStatement* dbConIn, unsigned binDepthIn) : ob::MotionValidator(si), dbcon(dbConIn), binDepth(binDepthIn)
            {
                defaultSettings();
            }

            /** \brief Constructor */
            ChartsDBMotionValidator2(const ob::SpaceInformationPtr &si, ENCGIS::lineIntersectLayerStatement* dbConIn, unsigned binDepthIn) : ob::MotionValidator(si), dbcon(dbConIn), binDepth(binDepthIn)
            {
                defaultSettings();
            }

            /** \brief Destructor */
            ~ChartsDBMotionValidator2() override = default;

            //! \brief Check if the path between two states (from \e s1 to \e s2) is valid. This function assumes \e s1
            //! is valid.
            //! @param [in] s1 start state of the motion to be checked (assumed to be valid)
            //! @param [in] s2 final state of the motion to be checked
            //! \note This function updates the number of valid and invalid segments. 
            bool checkMotion(const ob::State *s1, const ob::State *s2) const override;



            //! Check if the path between two states is valid. Also compute the last state that was
            //! valid and the time of that state. The time is used to parametrize the motion from \e s1 to \e s2, \e s1
            //! being at t =
            //!  0 and \e s2 being at t = 1. This function assumes \e s1 is valid.
            //! @param [in] s1 start state of the motion to be checked (assumed to be valid)
            //! @param [in] s2 final state of the motion to be checked
            //! @param [inout] lastValid first: storage for the last valid state (may be nullptr, if the user does not care
            //! about the exact state); this need not be different from \e s1 or \e s2. second: the time (between 0 and
            //! 1) of the last valid state, on the motion from \e s1 to \e s2. If the function returns false, \e
            //! lastValid.first must be set to a valid state, even if that implies copying \e s1 to \e lastValid.first
            //! (in case \e lastValid.second = 0). If the function returns true, \e lastValid.first and \e
            //! lastValid.second should \b not be modified.
            //!
            //! \note This function updates the number of valid and invalid segments. 
            //! \note Currently works by using a binary search method, first checking the entire distance, then half, then one fourth. Continues to binDepth depth
            bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;

        private:
            //! The OMPL representation of the stateSpace
            ob::StateSpace *stateSpace_;
            //! The database containing the layer to check for intersections
            //ENCGIS::DBconnection* dbcon;
            ENCGIS::lineIntersectLayerStatement *dbcon;//2("lndaretable", m_con->db);
            //! Defines how many halvings of the line should be done before being content.
            unsigned binDepth;
            //! Helperfunction used by constructors to check if there is a state space and set pointer variable
            void defaultSettings();
            //! Helperfunction checkMotion functions checking for intersection between two points. Does not update valid segments.
            bool transectSafety(const ob::State *s1, const ob::State *s2) const;
        };
    }
}

#endif // DUNE_CHARTSDATABASE_OMPL_MOTION_VALIDATOR_