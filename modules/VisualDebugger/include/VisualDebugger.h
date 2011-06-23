#ifndef VISUAL_DEBUGGER_H
#define VISUAL_DEBUGGER_H

#include "MCPainter.h"
#include "Controller.h"
#include "MonteCarloDebugger.h"

class VisualDebugger {
  Controller * robot; 
  Map * myMap;

  Localization * itl ; 
  PathPlanner * planner; 
  Graph * g; 
  MonteCarlo * mc; 
  MonteCarloDebugger * debugger; 

  int keyboardCtrl; 
 public: 
  VisualDebugger(Map*, Controller*); 
  MonteCarloDebugger* getDebugger() { return debugger; }
  
  void reshape(int, int); 
  void keyboard(unsigned char, int, int); 
  void keyboardSpecial(int, int, int); 
  void mouse(int, int, int, int); 
  void draw(void); 
  void drawFog(void);

  void reshapeCameraWindows(int, int); 
  void drawPlayerBlobs(void); 
  void drawObservationBlobs(void); 
  void drawParticleOrientations(void); 

  // util functions
  int getWinX(int); 
  int getMapX(int); 
  int getWinY(int); 
  int getMapY(int);
};

#endif
