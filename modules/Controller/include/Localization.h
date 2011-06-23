/*
 * Localization.h
 *
 */

#ifndef INTERFACE_TO_LOCALIZATION_H_
#define INTERFACE_TO_LOCALIZATION_H_

#include "boost/thread/mutex.hpp"
#include "libplayerc++/playerc++.h"
#include <unistd.h>
#include "Observation.h"
#include "Move.h"
#include "Position.h"
#include "Map.h"
#include "MonteCarlo.h"
#include "ObservationBlob.h"

using namespace PlayerCc;

class Localization {
public:
  Localization(Map * map, int fieldOfVision);

  void update();
  void move(Position relativePosition);
  Position getPosition();

  double getConfidence() {
    robotMutex.lock();
    double confidence = mc->getConfidence();
    robotMutex.unlock();
    return confidence;
  }
  
  MonteCarlo * getMonteCarlo() {
    return mc;
  }
  
  
  void setMCDebugger(){
    debugger = new MonteCarloDebugger(); 
    mc->setDebugger(debugger); 
  }
  
  MonteCarloDebugger* getMCDebugger(){ return debugger; }
  

  // move to Perception.cpp 
  vector<Observation> getObservations() {
    robotMutex.lock();
    vector<Observation> observations = obs;
    robotMutex.unlock();
    
    return observations;
  }

  void setObservationVariance(double observationVariance) {
    this->observationVariance = observationVariance;
  }

  bool isGlobalCameraAvailable() { return globalCameraAvailable; } 
  void setGlobalCamera(bool b) { globalCameraAvailable = b; }
  void setCurrentPosition(Position p) { currentPos = p; } 
  
  void setSpeed(double, double, double);
  void moveToMapPosition(int, int);

  bool isDestinationReached() { return destinationReached; }
  bool isDestinationSet();
  bool isFound() { return foundItem; }
  void resetDestinationInfo(); 

  void setBlobFinderProxy(PlayerClient*);
  BlobfinderProxy* getBlobfinderProxy() { return bfp; }
  void setPosition2dProxy(PlayerClient*);
  void setCameraProxy(PlayerClient*);
  CameraProxy* getCameraProxy() { return cam; }

  // move to Perception.cpp
  void printBlobColor(player_blobfinder_blob);
  void printBlobs(vector<observationBlob>& ); 
  void printBlobInfo(observationBlob);
  int getBlobColor(player_blobfinder_blob blob);

protected:
  MonteCarlo * mc;
  MonteCarloDebugger * debugger; 
  CameraProxy * cp;
  BlobfinderProxy * bfp;
  Position2dProxy * p2d;
  CameraProxy * cam;
  
  // copy in Perception.cpp 
  boost::mutex robotMutex;

  Position currentPos;
  Position startPos; 
  Position previousMove; 
  bool destinationReached; 
  Position destination;
  Position cumulativeMove;

  Map * map;
  int fov;						//the field of vision in degrees
  vector<Observation> obs;
  double observationVariance;
  
  bool foundItem ;
  bool globalCameraAvailable; 

  //bool positionEqual(Position p1, Position p2);

  //double radiansToDegrees(double rad);                  // not used. there is also Utils::ToDegrees function that does the same thing

  void readData();
  void updateObservations();
  Move getLastMove();
  
  Position convertToRobotCoordinates(Position mapPos); 
  Position convertToMapCoordinates(Position robotPos); 
  double calcHeadingToDestination(double x, double y);


  // move to Perception.cpp
  double getAngle(double x);
  
  bool isUsable( vector<observationBlob>& blob ) {
    vector<observationBlob>::iterator iter; 
    for ( iter = blob.begin() ; iter != blob.end() ; iter++ )
      if ( !iter->Used() ) 
	return true; 
    return false;
  }
  bool blobOnTopOf(observationBlob top, observationBlob bottom);

  void displayObservationSummary();
  void joinBlobs(vector<observationBlob>&);
  bool isOverlapping( observationBlob, observationBlob );

  vector<Observation> getRoomMarkers( vector<observationBlob>& topBlobs, 
						vector<observationBlob>& middleBlobs, 
						vector<observationBlob>& bottomBlobs, 
						string id);

  vector<Observation> getCornerMarkers( vector<observationBlob>& topBlobs, 
						  vector<observationBlob>& bottomBlobs, 
						  string id);

  vector<Observation> getEnteranceMarkers( vector<observationBlob>& blobs, 
						     string id);

  vector<Observation> findGreenObjects( vector<observationBlob>& blobs, string id);
};

#endif /* INTERFACE_TO_LOCALIZATION_H_ */
