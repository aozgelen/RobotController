/*
 * Perception.h
 *
 */

#ifndef PERCEPTION_H
#define PERCEPTION_H

class Perception {
public:
  Perception();

  void update();
  vector<Observation> getObservations() { return obs; }
  void setBlobFinderProxy(PlayerClient*);
  BlobfinderProxy* getBlobfinderProxy() { return bfp; }

  void setCameraProxy(PlayerClient*);
  CameraProxy* getCameraProxy() { return cam; }

  void printBlobColor(player_blobfinder_blob);
  void printBlobs(vector<observationBlob>& ); 
  void printBlobInfo(observationBlob);
  int getBlobColor(player_blobfinder_blob blob);

protected:
  CameraProxy * cp;
  BlobfinderProxy * bfp;
  Position2dProxy * p2d;
  CameraProxy * cam;

  vector<Observation> obs;
  
  // copy in Perception.cpp 
  boost::mutex robotMutex;

  int fov;						//the field of vision in degrees  
  bool foundItem ;

  void readData();
  void updateObservations();
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

#endif
