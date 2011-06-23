/*! \mainpage RobotController Documentation
 * \brief RobotController governs low-level actions of a robot.
 *
 * \authors A. Tuna Ozgelen (contact author), George Rabanca, Mark Manashirov, John Cummins 
 *
 * \version 1.0
 *
 * \date 2010 - 2011
 *
 * \b Introduction:
 *      This program is a collection of low-level control functions of a robot. Each robot 
 *  	posess different type of sensors and hardware, therefore the some of these behaviors
 *  	may not be available and/or may execute differently depending on the platform.
 *  	Low level behaviors implemented: 
 *  	- Localization: vision-based version development is in progress. 
 *  	- PathPlanner: given a map builds a navigation graph and finds the shortest path.  
 *  	- Obstacle avoidance: Not implemented yet. 
 *  	- Mapping: Not implemented yet.
 *  	- Perception: Vision-based version is in progress.
 *  	- Controller: basic behavior control
 *     	Also, to for debugging a visual debugger tool is available. 
 *
 * \b Compiling:
 *      - Run 'make' to build the libraries and the .controller executable
 *    	- Run 'make clean' to delete object files and the controller (doesn't rm libraries)
 *  	- Run 'make purge' to delete all *.o and *.a files from lib directories. 
 * 	- Run 'make docs' to install the documentation files into the /doc directory.
 *
 * \b Usage:
 *      Run ./controller with:
 *	   - -d option (optional) to run with the visual debugger
 *	   - -f option to specify the location of the configuration file. This file contains
 *	       - ip and port information of the central server
 *	       - map filename and path
 *	       - an entry for each robot containing:
 *	            - ip and port information of the player server
 *		    - robot's label
 *		    - robot's type (aibo, surveyor, scribbler, nxt)
 *		 
 * \todo implement ranger-based localization
 * \todo implement VFH for obstacle avoidance both with vision and range sensors
 * \todo implement mapping functionality
 * \todo implement ranger-based perception 
 *
 * \warning Still in development, contact author for questions. 
 *
 * \file main.cpp
 *
 * Parses the configuration files to connect to central server and player for the 
 * robot. Also parses the map file to use it in localization and initiates the visual 
 * debugger if -d option passed as an argument 
 *
 * Usage : ./controller [-d] -f <config-file>
 *
 * \defgroup Controller
 * \defgroup PathPlanner
 * \defgroup Localization
 * \defgroup VisualDebugger
 * \defgroup Perception
 */

#include "definitions.h"
#include "CommunicationManager.h"
#include "loiter.h"
#include "wallAvoid.h"
#include "Controller.h"

#include "libplayerc++/playerc++.h"
using namespace PlayerCc;

#include "Localization.h"
#include "VisualDebugger.h"
#include "Surveyor.h"
#include "Aibo.h"
#include <DataSerializer.h>
#include <MonteCarlo.h>
#include "MCPainter.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <cstring>
using namespace std;

Controller * ct; 
VisualDebugger * vD; 
Map * myMap;                
Localization * rbt;         

int mainWindow, mapWindow, thetaWindow, imageWindow, subWindow1, subWindow2;

/*! \brief Initializes the GLUT parameters. 

 */
void init(void) {
  glClearColor(1.0, 1.0, 1.0, 0.0);
  glShadeModel(GL_FLAT);
}

/*! \brief Wrapper function for GLUT callback register reshape. 
  
  \param w int width of the new window size
  \param h int height of the new window size

  Functions of another class cannot be used as functors. This is the workaround. 

  \sa VisualDebugger::reshape
*/
void reshape(int w, int h){ vD->reshape(w,h); } 

/*! \brief Wrapper function for GLUT callback register keyboard. 
  
  \param key ASCII value of the key pressed 
  \param x int x coordinate of the mouse pointer at the moment
  \param y int y coordinate of the mouse pointer at the moment

  Functions of another class cannot be used as functors. This is the workaround. 

  \sa VisualDebugger::keyboard
*/
void keyboard(unsigned char key, int x, int y){ vD->keyboard(key,x,y); } 

/*! \brief Wrapper function for GLUT callback register special keyboard commands (i.e., arrow keys). 

  \param key ASCII value of the key pressed 
  \param x int x coordinate of the mouse pointer at the moment
  \param y int y coordinate of the mouse pointer at the moment

  Functions of another class cannot be used as functors. This is the workaround. 

  \sa VisualDebugger::keyboardSpecial
*/
void keyboardSpecial(int key, int x, int y){ vD->keyboardSpecial(key,x,y); } 

/*! \brief Wrapper function for GLUT callback register mouse. 

  \param button
  \param state
  \param x int x coordinate of the mouse pointer at the moment
  \param y int y coordinate of the mouse pointer at the moment

  Functions of another class cannot be used as functors. This is the workaround. 
  
  \sa VisualDebugger::mouse
*/
void mouse(int button, int state, int x, int y){ vD->mouse(button, state, x, y); }

/*! \brief Draws main map window. 
  
  This function calls the VisualDebuggers function which draws everything in the main window: 
  robot, particles, walls, navigation graph, path etc. 

  \sa VisualDebugger::draw
 */
void draw(void){ 
  glutSetWindow(mainWindow); 
  vD->draw(); 
  glutPostRedisplay();
}

//void drawFog(void){ vD->drawFog(); }

/*! \brief Draws particle orientations in theta window
  
  This function calls a drawing function that displays each particle's theta value on the coordinate system. The distance of the particles from the origin is determined by the confidence value of the particle. Higher the confidence farther away it is displayed from the origin.

  \sa VisualDebugger::drawParticleOrientations
 */
void drawParticleOrientations(void){ 
  glutSetWindow(thetaWindow); 
  vD->drawParticleOrientations();
  glutPostRedisplay();
}

/*! \brief Draws Player blobs on a subwindow. 
  
  This function calls VisualDebugger::drawPlayerBlobs function to draws Player blobs read from blobfinder proxy and displays the drawings in a sub window which is the first of a series that is meant for visual display of steps taken by the Perception module. 

  \todo display camera image in the background. 

  \sa VisualDebugger::drawPlayerBlobs 
 */
void drawPlayerBlobs(void){   
  glutSetWindow(subWindow1); 
  vD->drawPlayerBlobs();
  glutPostRedisplay(); 
}

/*! \brief Draws observation blobs on a subwindow. 
  
  This function draws observation blobs which are part of a marker. This sub window is the one of a series of windows that is meant for visual display of steps taken by the Perception module. 

  \todo display camera image in the background.
  
  \sa VisualDebugger::drawObservationBlobs 
 */
void drawObservationBlobs(void){ 
  glutSetWindow(subWindow2); 
  vD->drawObservationBlobs();
  glutPostRedisplay(); 
}

/*! \brief Wrapper function for GLUT callback register reshape. This is for camera window containing multiple subwindows
  
  \param w int width of the new window size
  \param h int height of the new window size

  Functions of another class cannot be used as functors. This is the workaround. 

  \sa VisualDebugger::reshapeCameraWindows
*/
void reshapeCameraWindows(int w, int h){ vD->reshapeCameraWindows(w,h); }

/* end wrapper functions */

void renderScene() {
  glutSetWindow(imageWindow); 
  glClear(GL_COLOR_BUFFER_BIT); 
  glutSwapBuffers();
}

void renderAll(void) {
  drawPlayerBlobs(); 
  drawObservationBlobs(); 
}

/*! \brief Creates a default map of size \f$ 500 x 400 cm^2 \f$, enclosed by walls 
 */
void createMap_defaultField() {
  myMap = new Map(500, 400);
  
  // outer walls
  myMap->addWall(MapWall("wall1", 0, 0, 0, 400)); 
  myMap->addWall(MapWall("wall2", 0, 0, 500, 0)); 
  myMap->addWall(MapWall("wall3", 500, 0, 500, 400)); 
  myMap->addWall(MapWall("wall4", 0, 400, 500, 400)); 
}

/*! \brief Parses the map file. 

  If the map of the area is available its path and name appears as an entry in the configuration file, that is provided when the executable is run. This function adjusts the myMap object with the size, walls and markers that appear in the file.

  \param reference to an input file stream object pointing to the map file to be parsed

  \sa displayUsage readConfigFile
 */
void readMapFile(ifstream& mFile) {
  string  cmd, label, tmp;
  int x1, y1, x2, y2, lx, ly, rx, ry; 
  bool first = true ;

  while( !mFile.eof() ) {
    cmd = ""; label = ""; x1 = 0 ; y1 = 0 ; x2 = 0 ; y2 = 0 ; 
    mFile >> cmd ; 

    // in case the file is empty create a default map
    if ( mFile.eof() ){
      if ( first ){
	cout << "Empty map file!..." << endl; 
	createMap_defaultField();
      }
    }
    // if the line is commented skip it otherwise process.
    else if (! ( cmd[0] == '/' && cmd[1] == '/' ) ){ 
      // if the first command includes size, set the window for the specified values
      // else create the default map
      if ( first ){
	first = false;
	if ( cmd == "size" ) {
	  mFile >> x1 >> y1 ; 
	  myMap = new Map(x1, y1);
	  continue;
	}
	else
	  createMap_defaultField();
      }

      // process the command
      if ( cmd == "marker" ){ 
	mFile >> label >> x1 >> y1 >> lx >> ly >> rx >> ry ; 
	myMap->addMarker(MapMarker(label, x1, y1, lx, ly, rx, ry));	
      }
      else if ( cmd == "wall" ){
	mFile >> label >> x1 >> y1 >> x2 >> y2 ; 
	myMap->addWall(MapWall(label, x1, y1, x2, y2)); 
      }
      else {
	if ( cmd == "size" )
	  cout << "size command has to be the first command in map config file. command ignored." << endl;
	else
	  cout << "Unknown map config command: " << cmd << endl;
	getline(mFile, tmp); 
      }
    }
    else {
      // ignore the rest of the line.
      getline(mFile, tmp); 
    }
  } 
}

/*! \brief Displays the usage of the program. 

  Called when the arguments cannot be processed.
 */ 
void displayUsage(int argc, char** argv){
  cout << "USAGE: " << argv[0] << " [ options ]" << endl << endl;
  cout << "Where [ options ] can be: " << endl ;
  cout << "\t-d \t(for running optional visual debugger)" << endl;
  cout << "\t-f <config_filename>" << endl;
  exit(1);
}

/*! \brief Parses the configuration file passed as an argument

  \param input file stream for the config file
  \param string ref Central server hostname
  \param string ref Central server port
  \param string ref Player hostname (for the robot)
  \param string ref Player port (for the robot)
  \param string ref robot label
  \param string ref robot type
  
  This function parses and populates the information for the program to connect to the other components of the system.
 */
void readConfigFile(ifstream& cfFile, string& csHost, int& csPort, string& pHost, int& pPort, string& lab, string& ty){
  string cmd, mfile, tmp;

  // parse configuration file + attempt to connect the player and central servers
  while( !cfFile.eof() ) {
    cmd = ""; 
    mfile = "map.conf";    
    cfFile >> cmd ; 
    
    // if the line is commented skip it otherwise process.
    if (! (( cmd[0] == '/' && cmd[1] == '/' ) || ( cmd == "")) ){ 
      if ( cmd == "central_server" ) {
	cfFile >> csHost >> csPort ;
      }      
      else if ( cmd == "map" ){
	cfFile >> mfile;
	ifstream mapFile( mfile.c_str(), ios::in ); 
	if ( !mapFile ) {
	  cout << "Unable to open map file " <<  mfile << endl; 
	  createMap_defaultField();
	}
	else {
	  readMapFile(mapFile);
	  mapFile.close();
	}
      }
      else if ( cmd == "robot" ){ 
	cfFile >> lab >> ty >> pHost >> pPort ; 
      }
      else if ( cmd == "camera" ){
	getline(cfFile, tmp);
      }
      else {
	cout << "Unknown config command: " << cmd << endl;
	getline(cfFile, tmp); 
      }
    }
    else {
      // ignore the rest of the line.
      getline(cfFile, tmp); 
    }
  } 
}

/*! \brief Entry point to the program
  
  Creates a connection to the Central Server and player for the robot. It also generates map and other objects required by the controller. If ran with a -d option, displays visual debugger windows.  

  It generates a Controller object ct with the parameters read from the config file.

  \callgraph
 */
int main(int argc, char **argv)
{
  Utils::initRandom();          // srand(time(NULL))
  bool visualDEBUG = false; 

  // usage: <exec> [-d] [-f <config-file>]. to add more flags add it to the end of string followed
  // by a : or :: if the flag doesn't require an argument
  const char* optflags = "d::f:"; 
  int ch;

  ifstream configFile;
  string central_server_hostname = "127.0.0.1"; 
  int central_server_port = 6667;
  string  player_hostname = "127.0.0.1"; 
  int player_port = 6665;
  string label = "" , type = ""; 
  
  if ( argc == 1 ) {
    cout << "no config file arguments provided" << endl;
    displayUsage(argc, argv);
    exit(1); 
  }
  else {
    while ( -1 != ( ch = getopt( argc, argv, optflags ))){
      switch ( ch ) {
      case 'f': {
	cout << "configuration file: " << optarg << endl;
	configFile.open(optarg, ios::in); 
	if( !configFile ) {
	  cout << "Can't open configuration file: " << optarg << ". Aborted" << endl; 
	  exit(1);
	}
	readConfigFile(configFile, central_server_hostname, central_server_port, player_hostname, player_port, label, type);
	configFile.close();
	break;
      }
      case 'd':
	visualDEBUG = true; 
	break;
      default:
	displayUsage(argc, argv);
	break;
      }
    }
  }

  ct = new Controller(player_hostname, player_port, central_server_hostname, central_server_port, myMap, label, type);
  boost::thread * controllerThread = new boost::thread(*ct); 

  if ( visualDEBUG ) {
    vD = new VisualDebugger(myMap, ct);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInit(&argc, argv);      
    
    glutInitWindowSize(glutGet(GLUT_SCREEN_HEIGHT),glutGet(GLUT_SCREEN_HEIGHT));
    glutInitWindowPosition(0,0);
    mainWindow = glutCreateWindow(argv[0]);
    init();
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(keyboardSpecial); 
    glutSetCursor(GLUT_CURSOR_CROSSHAIR);
    glutDisplayFunc(draw);
    
    glutInitWindowSize(200,200);
    glutInitWindowPosition(glutGet(GLUT_SCREEN_HEIGHT)+10,0);      
    thetaWindow = glutCreateWindow("Orientations");
    glutReshapeFunc(reshape);
    glutDisplayFunc(drawParticleOrientations); 
    
    
    glutInitWindowSize(340, 527); 
    glutInitWindowPosition(1200,0);
    imageWindow = glutCreateWindow("camera");
    glutReshapeFunc(reshape); 
    glutDisplayFunc(renderScene);
    subWindow1 = glutCreateSubWindow(imageWindow, 5, 5, 330, 256); 
    {
      //glutReshapeFunc(reshapeCameraWindows); 
      glutReshapeFunc(reshape); 
      glutDisplayFunc(drawPlayerBlobs); 	
    }
    
    subWindow2 = glutCreateSubWindow(imageWindow, 5, 266, 330, 256); 
    {
      //glutReshapeFunc(reshapeCameraWindows); 
      glutReshapeFunc(reshape); 
      glutDisplayFunc(drawObservationBlobs); 	
    }
    
    glutMainLoop();  
  }
  controllerThread->join();
  
  return 0;
}
