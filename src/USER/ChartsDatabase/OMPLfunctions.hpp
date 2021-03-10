#ifndef DUNE_CHARTSDATABASE_OMPL_FUNCTIONS_
#define DUNE_CHARTSDATABASE_OMPL_FUNCTIONS_

// OMPL headers
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/State.h>
#include "ompl/util/Console.h"

// DUNE headers.
#include <USER/ChartsDatabase/Connection.hpp>
#include <USER/ChartsDatabase/DBTree.hpp>
#include <DUNE/Tasks/Task.hpp>


namespace og = ompl::geometric;

namespace DUNE
{
    namespace ChartsDatabase
    {

        //! Convenience function for printin a path to cout
        //! TODO: Take ostream as parameter to support others, like filestream
        //! @param[in] states The geometric path to be printed
        void printPath(og::PathGeometric states);

        //! Writes a path to a database. The main purpose is to easily visualize it in a GIS at a later point.
        //! TODO: Create table if not exists.
        //! @param[in] states The geometric path to be add to tree.
        //! @param[in] treeName The table to store the tree in. Needs to have been created before this function is called.
        //! @param[in] tree Pointer to tree database object.
        void pathToTree(og::PathGeometric states, std::string treeName, ChartsDatabase::DBTree* tree);

        //! This (utility) method generates a PlanSpecification
        //! consisting in the given maneuver sequence.
        //! This function is sourced from the plan.generator task.
        //! @param[in] plan_id The name of the plan to be generated
        //! @param[in] maneuvers A vector with maneuvers (order of the resulting plan will correspond to the order of this vector).
        //! @param[out] result The resulting PlanSpecification will be stored here.
        void sequentialPlan(std::string plan_id, const IMC::MessageList<IMC::Maneuver>* maneuvers, IMC::PlanSpecification& result) ;


        //! Function to transform a ompl::geometric::PathGeometric to a DUNE::IMC::PlanDB entry. All transects are turned into GoTo
        //! maneuvers with a fixed speed.
        //! @param[in] paths The geometric path to be add to create a plan from.
        //! @param[in] plan_id A unique ID/name for the plan. To activate the task. To activate the path, a IMC::PlanControl message using this name needs to be sent
        //! @param[in] speed The speed to set in the meneuvers.
        IMC::PlanDB createPlanDBEntry(og::PathGeometric paths, std::string plan_id, fp32_t speed);


        //! Function to be used as a state validator in OMPL. 
        //! TODO: Ensure given minimum depth 
        //! @param[in] state The geometric state to check validity of.
        //! @param[in] dbCon The database containing the queried tables.        
        bool isStateValid(const ompl::base::State *state, ChartsDatabase::ChartsDBConnection* dbCon);


        //! This class redirects OMPL outputs to use output methods of the DUNE system.  
        class OutputHandlerDUNEConsole : public ompl::msg::OutputHandler {
            public:
                //! Constructor.
                //! @param[in] inTask Pointer to the DUNE task used to output OMPL messages.
                OutputHandlerDUNEConsole(DUNE::Tasks::Task *inTask) : task(inTask) {};

                //! The function called by OMPL for outputing data to the user.
                //! @param[in] text
                //! @param[in] level
                //! @param[in] filename
                //! @param[in] line
                void log(const std::string &text, ompl::msg::LogLevel level, const char *filename, int line) override;
            private:
                //! Stores pointer to task used for output
                DUNE::Tasks::Task *task;
        };
    }
}

#endif // DUNE_CHARTSDATABASE_OMPL_FUNCTIONS_
