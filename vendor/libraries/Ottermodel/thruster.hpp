#include <Eigen/Core>

#ifndef OTTERMODEL_THRUSTER
#define OTTERMODEL_THRUSTER
namespace Ottermodel
{
  namespace Thruster
  {
    float forceToThrust(float force);
    float thrustToForce(float thrust);
  }
}
#endif