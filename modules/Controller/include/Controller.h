#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "definitions.h"
#include "CommunicationManager.h"
#include "Localization.h"
#include "Surveyor.h"
#include "PathPlanner.h"
#include "libplayerc++/playerc++.h"
using namespace PlayerCc;

class Controller {
public:
  //Controller(PlayerClient*, CommunicationManager*, Map*, Localization*) ;
  Controller(string, int, string, int, Map*, string, string) ;
  ~Controller(){ 
    stop(); 
  }

  void stop(){ itl->setSpeed(0,0,0); }
  void setOpMode(Mode m) { 
    opMode = m; 
  }
  void resetPathInfo();

  PathPlanner * getPlanner() { return planner; }
  Localization * getLocalization() { return itl; }
  Graph * getNavgraph() { return navGraph; }
  CommunicationManager * getRobot() { return cMan; }
  Node getWaypoint() { return waypoint; }

  void operator()();
private:
  Mode opMode;

  PlayerClient * pCli; 
  CommunicationManager * cMan;
  Localization * itl ; 
  Graph * navGraph; 
  PathPlanner * planner;
  Map * localMap;
  
  void updateBehavior();
  void updateManualBehavior();
  void updateMixedInitBehavior();
  void updateAutoBehavior();
  bool isPlanValid(); 
  bool isTargetSet();   
  void localize();

  // behavior vars
  Position prevPos, currPos; 
  Node waypoint; 
};

#endif /* CONTROLLER_H */
