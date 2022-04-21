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
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <mqtt/async_client.h>
#include <fstream>

// DUNE headers.
#include <DUNE/DUNE.hpp>
//#include <DUNE/Time/Clock.hpp>

#include "json.hpp"
namespace Transports
{
  //! Device driver for ThelmaHydrophone
  namespace MQTT
  {
    using DUNE_NAMESPACES;
    const std::string TOPIC 			{ "hello" };

    const std::string DFLT_SERVER_ADDRESS	{ "ssl://otter.itk.ntnu.no:8883" };
    const std::string DFLT_CLIENT_ID		{ "ssl_publish_cpp" };

    const std::string KEY_STORE				{ "/home/nikolai/Documents/NTNU/PhD/PIT/certs/otter.pem" };
    const std::string TRUST_STORE			{ "/home/nikolai/Documents/NTNU/PhD/PIT/ca/ca_certificates/ca.crt" };

    const std::string LWT_TOPIC				{ "events/disconnect" };
    const std::string LWT_PAYLOAD			{ "Last will and testament." };

    const int  QOS = 1;
    const auto TIMEOUT = std::chrono::seconds(10);
    struct Arguments
    {

      //! Sync Period;
      double sync_period;
      double lat;
      double lon;
      double alt;
      std::string id;
      std::string data;

    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      //! Timer.
      Time::Counter<float> m_sync_timer;
      bool storeProblem;
      mqtt::async_client *cli;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        cli(nullptr)
      {
        param("Sync Period", m_args.sync_period)
        .units(Units::Second)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");


        param("Latitude", m_args.lat)
        .defaultValue("63.33")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Longtitude", m_args.lon)
        .defaultValue("10.083333")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Altitude", m_args.alt)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("Data", m_args.data)
        .defaultValue("10.0")
        .minimumValue("0.0")
        .description("Period between sync messages");

        param("ID", m_args.id)
        .defaultValue("Fish_position_est_1")
        .minimumValue("0.0")
        .description("Period between sync messages");

      }
/////////////////////////////////////////////////////////////////////////////

/**
 * A callback class for use with the main MQTT client.
 */
class callback : public virtual mqtt::callback
{
public:
  DUNE::Tasks::Task *task;
  callback(DUNE::Tasks::Task *taskIn) {
    task=taskIn;
  }
	void connection_lost(const std::string& cause) override {
		task->spew("\nConnection lost");
		if (!cause.empty())
			task->spew("\tcause: %s",cause.c_str());
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override {
		//spew("\tDelivery complete for token: %s", std::to_string((tok ? tok->get_message_id() : -1)));
	}
};

/////////////////////////////////////////////////////////////////////////////
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {

      }

      void
      onResourceAcquisition(void)
      {
        std::string	address  = DFLT_SERVER_ADDRESS,
            clientID = DFLT_CLIENT_ID;

        // Note that we don't actually need to open the trust or key stores.
        // We just need a quick, portable way to check that they exist.
        storeProblem = false;
        {
          std::ifstream tstore(TRUST_STORE);
          if (!tstore) {
            err("The trust store file does not exist: %s", TRUST_STORE.c_str());
            storeProblem = true;
          }

          std::ifstream kstore(KEY_STORE);
          if (!kstore) {
            err("The key store file does not exist: %s", KEY_STORE.c_str());
            storeProblem = true;
          }
        }
        if(!storeProblem) {
          cli = new mqtt::async_client(address, clientID);

          callback cb(this);
          cli->set_callback(cb);
          // Build the connect options, including SSL and a LWT message.
          auto sslopts = mqtt::ssl_options_builder()
                    .trust_store(TRUST_STORE)
                    .key_store(KEY_STORE)
                    .error_handler([this](const std::string& msg) {
                      this->err("SSL Error: %s",msg.c_str());
                    })
                    .finalize();

          auto willmsg = mqtt::message(LWT_TOPIC, LWT_PAYLOAD, QOS, true);

          auto connOpts = mqtt::connect_options_builder()
                      .user_name("SLIM")
                      .password("SLIM")
                      .will(std::move(willmsg))
                    .ssl(std::move(sslopts))
                                .clean_session(false)
                    .finalize();

          inf("  ...OK");
          try {
            spew("\nConnecting...");
            mqtt::token_ptr conntok = cli->connect(connOpts);
            spew("Waiting for the connection...");
            conntok->wait();
            spew("  ...OK");
            // Start consumer before connecting to make sure to not miss messages

            cli->start_consuming();

            // Connect to the server
            //spew("Connecting to the MQTT server..." << flush;
            //auto tok = cli->connect(connOpts);

            // Getting the connect response will block waiting for the
            // connection to complete.
            auto rsp = conntok->get_connect_response();
            
            // If there is no session present, then we need to subscribe, but if
            // there is a session, then the server remembers us and our
            // subscriptions.
            //if (!rsp.is_session_present())
            cli->subscribe(TOPIC, QOS)->wait();

            spew("OK");
        

            // Consume messages
            // This just exits if the client is disconnected.
            // (See some other examples for auto or manual reconnect)

            spew("Waiting for messages on topic: '%s'",TOPIC.c_str());

          } catch (const mqtt::exception& exc) {
            err("strange");
            err("%s", exc.to_string().c_str());
          }
        }
      }


      void
      onResourceRelease(void) {
        if(cli != NULL) {
          if (cli->is_connected()) {
            spew("\nShutting down and disconnecting from the MQTT server...");
            cli->unsubscribe(TOPIC)->wait();
            cli->stop_consuming();
            cli->disconnect()->wait();
            spew("OK");
          } else {
            spew("\nClient was disconnected");
          }
          delete cli;
        }
      }

      void
      onResourceInitialization(void)
      {

      }

      void parsemsg(mqtt::const_message_ptr &msg) {
        std::string content = msg->to_string();
        spew("%s: %s", msg->get_topic().c_str(),content.c_str());
        try {
          nlohmann::json j3 = content;
          std::string topic = j3["topic"];
          inf("Topic: %s", topic.c_str());
        } catch(...) {
          err("JSON parsing failed");
        }
        

      }
      void
      onMain(void)
      {
        mqtt::const_message_ptr msg;
        while(!stopping()) {
          if(cli != nullptr) {
            if (cli->try_consume_message(&msg)) {
              inf("newmsg");
              parsemsg(msg);
            }
          }

          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
