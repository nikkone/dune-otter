//***************************************************************************
// Copyright 2007-2021 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <algorithm>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace ASV
  {
    namespace RemoteOperationOtter
    {
      using DUNE_NAMESPACES;
      typedef enum RCmode
      {
          RCM_NORMAL   = 0,
          RCM_FORCE    = 1,
          RCM_SKEWED   = 2
      } RCmode_t;
      //! Task arguments.
      struct Arguments
      {
        //! Thrust scaling.
        double scale;
        //! Factor to reduce forward thrust when differential thrusting
        float skewed_motor_force_factor;
        u_int8_t rcmode;
      };

      struct Task: public DUNE::Control::BasicRemoteOperation
      {
        //! Motor commands.
        IMC::SetThrusterActuation m_thrust[2];
        //! Task arguments.
        Arguments m_args;

        double m_speed;
        double m_heading;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::BasicRemoteOperation(name, ctx)
        {
          param("Thrust Scale", m_args.scale)
          .defaultValue("1.0");

          param("Skewed Motor Force Factor", m_args.skewed_motor_force_factor)
          .defaultValue("0.0")
          .description("Factor to reduce forward thrust when differential thrusting.");

          param("RC mode", m_args.rcmode)
          .defaultValue("0")
          .description("");

          // Add remote actions.
          addActionAxis("Port Motor");
          addActionAxis("Starboard Motor");
          addActionButton("Accelerate");
          addActionButton("Decelerate");
          addActionAxis("Heading");
          addActionAxis("Throttle");
          addActionButton("Stop");

          // Initialize SetThrusterActuation messages.
          m_thrust[0].id = 0;
          m_thrust[1].id = 1;
          m_speed = m_heading = 0;
        }

        void
        onActivation(void)
        {
          m_thrust[0].value = 0;
          m_thrust[1].value = 0;
          m_speed = m_heading = 0;
          actuate();
        }

        void
        onDeactivation(void)
        {
          m_thrust[0].value = 0;
          m_thrust[1].value = 0;
          m_speed = m_heading = 0;
          actuate();
        }

        void
        onConnectionTimeout(void)
        {
          m_thrust[0].value = 0;
          m_thrust[1].value = 0;
          m_speed = m_heading = 0;
          actuate();
        }

        double
        applyScale(int value)
        {
          return Math::trimValue((value / 127.0) * m_args.scale, -1.0, 1.0);
        }

        //! Model converting force to thrust actuation level (Untrimmed)
        //! @param[in] force value of force currently in the motor
        //! @return thrust actuation.
        float forceToThrust(float force){
          if(force > 0) {
            float weights[] = {0.01137,-7.549e-05,1.86e-07};
            return weights[0]*force+weights[1]*force*force+weights[2]*force*force*force; // ax+bx^2+cx^3
          } else if(force < 0) {
            float weights[] = {0.01912, 0.0002268, 1.012e-06};
            return weights[0]*force+weights[1]*force*force+weights[2]*force*force*force; // ax+bx^2+cx^3
          } else { // Force is 0
            return 0.0;
          }
          return 0.0;
        }
        //! Model converting thrust actuation level to force (Untrimmed)
        //! @param[in] thrust value of thrust currently in the motor
        //! @return force.
        float thrustToForce(float thrust){
          if(thrust > 0) {
            float weights[] = {-146.4,948.7,-556.3}; 
            return weights[0]*thrust+weights[1]*thrust*thrust+weights[2]*thrust*thrust*thrust; // ax+bx^2+cx^3
          } else if(thrust < 0) {
            float weights[] = {-99.73, -609.1, -378.6};
            return weights[0]*thrust+weights[1]*thrust*thrust+weights[2]*thrust*thrust*thrust; // ax+bx^2+cx^3
          } else { // Force is 0
            return 0.0;
          }
          return 0.0;
        }

        void
        onRemoteActions(const IMC::RemoteActions* msg)
        {
          TupleList tuples(msg->actions);

          m_thrust[0].value = applyScale(tuples.get("Port Motor", 0));
          m_thrust[1].value = applyScale(tuples.get("Starboard Motor", 0));

          if (m_thrust[0].value == 0 && m_thrust[1].value == 0)
          {
              double throttle = applyScale(tuples.get("Throttle", 0));
              inf("%f", throttle);
              if (tuples.get("Decelerate", 0))
                m_speed -= 0.05;
              else if (tuples.get("Accelerate", 0))
                m_speed += 0.05;
              else if (throttle) {
                m_speed = throttle;
              }
                

              m_speed = Math::trimValue(m_speed, -1.0 , 1.0);

              double hdng = (tuples.get("Heading", 0)) / 127.0;
              double leftThrust = m_speed;
              double rightThrust = m_speed;


              switch(m_args.rcmode) {
                case RCM_FORCE:
                  {
                    double minForce = 130.0;
                    leftThrust *= 1+hdng*2;
                    rightThrust *= 1-hdng*2;
                    double leftForce  = thrustToForce(leftThrust);                    
                    double rightForce = thrustToForce(rightThrust);
                    if(leftForce < -minForce) {
                      rightForce = Math::trimValue(rightForce, -minForce, minForce);
                      inf("sat l");
                    }
                    if(rightForce < -minForce) {
                      leftForce = Math::trimValue(leftForce, -minForce, minForce);
                      inf("sat r");
                    }
                    leftThrust  = forceToThrust(leftForce);
                    rightThrust = forceToThrust(rightForce);
                  }
                break;
                case RCM_SKEWED:
                  if(hdng>0) {
                    leftThrust  *= 1+(hdng*2*m_args.skewed_motor_force_factor);
                    rightThrust *= 1-hdng*2;
                  } else if(hdng<0) {
                    rightThrust *= 1-(hdng*2*m_args.skewed_motor_force_factor);
                    leftThrust  *= 1+hdng*2;
                  }
                break;
                case RCM_NORMAL:
                  leftThrust *= 1+hdng*2;
                  rightThrust *= 1-hdng*2;
                break;
                default:
                  spew("Invalid mode %u, changing to normal mode.", m_args.rcmode);
                  m_args.rcmode = RCM_NORMAL;
              };
              spew("Heading: %f", hdng);

              m_thrust[0].value = Math::trimValue(leftThrust, -1.0, 1.0);
              m_thrust[1].value = Math::trimValue(rightThrust, -1.0, 1.0);

              if (tuples.get("Stop", 0))
                m_speed = m_thrust[0].value = m_thrust[1].value = 0;
          }
          else
            m_speed = m_heading = 0;
        }

        void
        actuate(void)
        {
          debug("%0.2f %0.2f", m_thrust[0].value, m_thrust[1].value);

          dispatch(m_thrust[0]);
          dispatch(m_thrust[1]);
        }
      };
    }
  }
}

DUNE_TASK
