#ifndef _COMMUNICATION_MANAGER_H
#define _COMMUNICATION_MANAGER_H

#include "definitions.h"
//#include "Controller.h"
#include "behavior.h"
#include "PathPlanner.h"
#include "metrobotics.h"
#include "Localization.h"
#include "libplayerc++/playerc++.h"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"
#include <stdint.h>
#include <stdlib.h>

using namespace metrobotics; 

class CommunicationManager
{
 public:
  //CommunicationManager(PlayerCc::PlayerClient& pc, Localization * i, string label, string type, Behavior* bp = 0);
  CommunicationManager(PlayerCc::PlayerClient& pc, Localization * i, string label, string type);
  ~CommunicationManager();
  
  // State management.
  int  GetState() const { return mCurrentState; }
  bool IsRegistered() const { return mSessionID >= 0; }
  bool IsLocked() const { return mPossessed; }
  
  // Connect to the central server.
  bool Connect(const std::string& hostname, unsigned short port);
  void Disconnect();
  bool isCommAlive() const;
		
  // Heart beat of the robot unit;
  // Updates and maintains the internal state machine.
  void Update();
		
  // Select CommunicationManager behavior.
  //void SetBehavior(Behavior* bp) { mBehavior = bp; }
  void SetPlanner(PathPlanner* p) { mPlanner = p; }
		
 private:
  // Binding to Player server
  PlayerCc::PlayerClient& mPlayerClient;

  // Player client proxies.
  boost::shared_ptr<PlayerCc::Position2dProxy> mPosition2D;

  // Boost ASIO (for sockets)
  boost::asio::io_service mIOService;
  boost::asio::ip::tcp::socket mSocket;

  // CommunicationManager properties.
  std::string mTypeID;
  std::string mNameID;
  std::vector<std::string> mProvidesList;
  
  // CommunicationManager behavior.
  //Behavior* mBehavior;
  PathPlanner* mPlanner;

  // Interfact to Localization
  Localization * itl;

  // State properties.
  int  mCurrentState;
  long mSessionID;
  bool mPossessed;

  bool foundSent;

  // Internal timers.
  static const double MAX_TIME_SILENCE = 60.0;
  static const double MAX_TIME_STATE   = 10.0;
  metrobotics::PosixTimer mSilenceTimer;
  metrobotics::PosixTimer mStateTimer;

  // Internal buffers.
  std::string mStringBuffer;

  // Internal functions.
  void init_state();
  void init_provides();
  bool msg_waiting() const;
  bool read(std::stringstream& ss);
  bool write(const std::stringstream& ss);

  // State actions.
  void do_state_change(int state);
  void do_state_action_init();
  void do_state_action_ack();
  void do_state_action_idle();
  void do_state_action_ping_send();
  void do_state_action_pong_read();
  void do_state_action_pong_send();
  void do_state_action_cmd_proc();
  void do_state_action_moving();
  void do_state_action_pose();
  void do_state_action_player();
  void do_state_action_found();
  void do_state_action_goto();
  void do_state_action_campose();
};

#endif
