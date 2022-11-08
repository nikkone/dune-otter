//***************************************************************************
// Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)*
// Department of Engineering Cybernetics (ITK)                              *
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
// Author: Nikolai Lauvås (based on GPS by Ricardo Martins)                 *
//***************************************************************************
 

//TODO: Add feature: position at 5s before tag registration in imc message.
//TODO: May not work without PPS anymore.

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>
#include <ctime> /* time_t, struct tm, time, mktime */
#include <string>
#include <sstream>
#include <inttypes.h>
#include <chrono>

// DUNE headers.
#include <DUNE/DUNE.hpp>
//#include <DUNE/Time/Clock.hpp>

// Local headers.
#include "Reader.hpp"
#include "TQbus.hpp"

namespace Actuators
{
  //! Device driver for ThelmaHydrophone
  namespace TQreader
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Serial port handle.
      IO::Handle* m_handle;
      //! Task arguments.
      Arguments m_args;
      //! TQSerialReader thread.
      TQSerialReader* m_TQSerialReader;

      Torqeedo::TQbus* tqbus;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_handle(NULL),
        m_TQSerialReader(NULL),
        tqbus(NULL)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyUSB0")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("19200")
        .description("Serial port baud rate");

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }
      void
      onResourceAcquisition(void)
      {
        try
        {
          if (!openSocket())
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);

          m_TQSerialReader = new TQSerialReader(this, m_handle);
          m_TQSerialReader->start();
        }
        catch (...)
        {
          throw RestartNeeded(DTR("1"), 5);
        }

       tqbus = new Torqeedo::TQbus(m_handle, Torqeedo::TQbus::ConnectionType::TYPE_TILLER, this);
      }

      bool
      openSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.uart_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_handle = sock;
        return true;
      }

      void
      onResourceRelease(void)
      {
        if (m_TQSerialReader != NULL)
        {
          m_TQSerialReader->stopAndJoin();
          delete m_TQSerialReader;
          m_TQSerialReader = NULL;
        }
        Memory::clear(tqbus);
        Memory::clear(m_handle);
        
      }

      void
      onResourceInitialization(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        spew("%s", sanitize(msg->value).c_str());

        processSentence(msg->value);
      }

      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
        //spew("Process: %s", line.c_str());
        tqbus->main_loop(line);
      }

      void
      onMain(void)
      {
        while(!stopping()) {

          //consumeMessages();
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
