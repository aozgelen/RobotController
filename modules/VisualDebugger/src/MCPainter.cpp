/*
 * MCPainter.cpp
 *
 *  Created on: Jun 2, 2010
 *      Author: appleapple
 */

#include "MCPainter.h"

MCPainter::MCPainter() {
  // TODO Auto-generated constructor stub

}

void MCPainter::drawFogOfExploration(){
  glBegin(GL_POLYGON); 
  {
    glVertex2i(100, 100);
    glVertex2i(100, 300);
    glVertex2i(300, 300);
    glVertex2i(300, 100);
  }
}

void MCPainter::drawGrid(Map * map) {
  glBegin(GL_LINES);
  {
    glColor3f(.75, .75, .75);   
    for (int i = 0; i < 10; i++) {
      glVertex2f(0, i * map->getHeight() / 10.0);
      glVertex2f(map->getLength(), i * map->getHeight() / 10.0);
      glVertex2f(i * map->getLength() / 10.0, 0);
      glVertex2f(i * map->getLength() / 10.0, map->getHeight());
    }
  }
  glEnd();
}

void MCPainter::drawMarkerPatch(MapMarker m, char color, int x1, int x2, int y1, int y2){
  glBegin(GL_POLYGON);
  if ( color == 'p' )
    glColor3f(1,0,1);
  else if ( color == 'b' )
    glColor3f(0,0,1);
  else if ( color == 'o' )
    glColor3f(1,0.5,0);
  else if ( color == 'y' )
    glColor3f(1,1,0);

  glVertex2i(m.getX() + x1, m.getY() + y1);
  glVertex2i(m.getX() + x1, m.getY() + y2);
  glVertex2i(m.getX() + x2, m.getY() + y2);
  glVertex2i(m.getX() + x2, m.getY() + y1);
  
  glEnd();
}

void MCPainter::drawMarkers(Map * map) {
  vector<MapMarker> markers = map->getMarkers();
  for (int i = 0; i < markers.size(); i++) {
    // corner markers
    if (markers[i].getId() == "p/y"){
      drawMarkerPatch(markers[i], 'p', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'y', 0,2,-2,2);
    }
    else if (markers[i].getId() == "y/p"){
      drawMarkerPatch(markers[i], 'y', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'p', 0,2,-2,2);
    }
    else if (markers[i].getId() == "y/b"){
      drawMarkerPatch(markers[i], 'y', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'b', 0,2,-2,2);
    }
    else if (markers[i].getId() == "b/y"){
      drawMarkerPatch(markers[i], 'b', -2,0,-2,2);
      drawMarkerPatch(markers[i], 'y', 0,2,-2,2);
    }
    
    // room markers
    else if (markers[i].getId() == "b/p/y"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/y/p"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/o/p"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);      
    }
    else if (markers[i].getId() == "b/p/o"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/o/y"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);
    }
    else if (markers[i].getId() == "b/y/o"){
      drawMarkerPatch(markers[i], 'b', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }

    // corridor markers
    else if (markers[i].getId() == "p/o/y"){
      drawMarkerPatch(markers[i], 'p', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'y', 1,3,-2,2);      
    }
    else if (markers[i].getId() == "y/p/o"){
      drawMarkerPatch(markers[i], 'y', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'p', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }
    else if (markers[i].getId() == "y/o/p"){
      drawMarkerPatch(markers[i], 'y', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'p', 1,3,-2,2);
    }
    else if (markers[i].getId() == "p/y/o"){
      drawMarkerPatch(markers[i], 'p', -3,-1,-2,2);
      drawMarkerPatch(markers[i], 'y', -1,1,-2,2);
      drawMarkerPatch(markers[i], 'o', 1,3,-2,2);
    }

    // entrance markers
    else if (markers[i].getId() == "b"){
      drawMarkerPatch(markers[i], 'b', -1,1,-2,2);
    }
    else if (markers[i].getId() == "o"){
      drawMarkerPatch(markers[i], 'o', -1,1,-2,2);      
    }
  }
}

void MCPainter::drawWalls(Map * map){
  glBegin(GL_LINES);
  {
    vector<MapWall> walls = map->getWalls();
    vector<MapWall>::iterator iter; 
    for ( iter = walls.begin(); iter != walls.end(); iter++ ){
      //glColor3f(1,1,1);   // if background is white
      glColor3f(0,0,0);
      glVertex2f(iter->getX0(), iter->getY0()); 
      glVertex2f(iter->getX1(), iter->getY1());
    }
  }
  glEnd();
}

void MCPainter::drawNodes(Graph * g){
  if ( g->numNodes() > 0 ) {
    glBegin(GL_LINES);
    {
      vector<Node> nodes = g->getNodes();
      vector<Node>::iterator iter; 
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
	//glColor3f(0.25,0.25,0.25); 
	glColor3f(0.95,0.95,0.95);       // if bg white
	glVertex2f(iter->getX()-1, iter->getY()-1); 
	glVertex2f(iter->getX()-1, iter->getY()+1); 
	glVertex2f(iter->getX()-1, iter->getY()+1); 
	glVertex2f(iter->getX()+1, iter->getY()+1); 
	glVertex2f(iter->getX()+1, iter->getY()+1); 
	glVertex2f(iter->getX()+1, iter->getY()-1); 
	glVertex2f(iter->getX()+1, iter->getY()-1); 
	glVertex2f(iter->getX()-1, iter->getY()-1); 
      }
    }
    glEnd();
  }
}


void MCPainter::drawEdges(Graph * g){
  if ( g->numEdges() > 0 ) {
    glBegin(GL_LINES);
    {
      vector<Edge> edges = g->getEdges();
      vector<Edge>::iterator iter; 
      for( iter = edges.begin(); iter != edges.end(); iter++ ) {
	//glColor3f(0.1,0.1,0.1);  
	glColor3f(0.98,0.98,0.98);       // if bg white
	Node n1 = g->getNode(iter->getFrom()); 
	Node n2 = g->getNode(iter->getTo());
	glVertex2f(n1.getX(), n1.getY()); 
	glVertex2f(n2.getX(), n2.getY()); 
      }
    }
    glEnd();
  }
}


void MCPainter::drawSource(Graph * g, int x, int y){
  glBegin(GL_POLYGON);
  if ( g->isWithinBorders(x,y) ) {
    glColor3f(0,1,0);
    glVertex2i(x-2, y-2); 
    glVertex2i(x-2, y+2); 
    glVertex2i(x+2, y+2); 
    glVertex2i(x+2, y-2); 
  }
  glEnd();
}

void MCPainter::drawTarget(Graph * g, int x, int y){
  glBegin(GL_POLYGON);
  if ( g->isWithinBorders(x,y) ) {
    glColor3f(1,0,0);
    glVertex2i(x-2, y-2); 
    glVertex2i(x-2, y+2); 
    glVertex2i(x+2, y+2); 
    glVertex2i(x+2, y-2); 
  }
  glEnd();
}

void MCPainter::drawGoal(Graph * g, int x, int y){
  glBegin(GL_POLYGON);
  if ( g->isWithinBorders(x,y) ) {
    glColor3f(0,0,1);
    glVertex2i(x-2, y-2); 
    glVertex2i(x-2, y+2); 
    glVertex2i(x+2, y+2); 
    glVertex2i(x+2, y-2); 
  }
  glEnd();
}

void MCPainter::drawPath(Localization * itl, PathPlanner * planner, Graph * g) {
  list<int> nodes = planner->getPath();
  if ( !nodes.empty() ) {

    // draw a line between current position to first node in graph
    Position pos = itl->getPosition();
    glBegin(GL_LINES); 
    glColor3f(0.5,0.5,0);
    glVertex2f(pos.getX(), pos.getY()); 
    glVertex2f(g->getNode(nodes.front()).getX(), g->getNode(nodes.front()).getY()); 
    glEnd();

    // draw the lines between nodes
    list<int>::iterator iter;
    for( iter = nodes.begin(); iter != nodes.end(); ) {
      int f = *iter; 
      if( ++iter != nodes.end() ){
	glBegin(GL_LINES);
	glColor3f(0.5,0.5,0);
	glVertex2f(g->getNode(f).getX(), g->getNode(f).getY()); 
	glVertex2f(g->getNode(*iter).getX(), g->getNode(*iter).getY()); 
	glEnd();
      }
    }
    // draw a line between the last node to target
    glBegin(GL_LINES); 
    glColor3f(0.5,0.5,0);
    glVertex2f(g->getNode(nodes.back()).getX(), g->getNode(nodes.back()).getY()); 
    glVertex2f(planner->getTarget().getX(), planner->getTarget().getY()); 
    glEnd();
  }
}

void MCPainter::drawParticles(MonteCarloDebugger * debugger) {
  
  glBegin(GL_POINTS);
  {
    // glVertex2f(0, 0);
    //glVertex2f(150, 150);
    for (int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      glVertex2f(p.getX(), p.getY());
    }
  }
  
  // to plot the particles as lines to see the orientation
  /*
  glBegin(GL_LINES);
  {
    for (int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      int len = 5; 
      double xp = p.getX() + len * cos(p.getTheta()) ;
      double yp = p.getY() + len * sin(p.getTheta()) ;
      glVertex2f(p.getX(), p.getY()); 
      glVertex2f(xp,yp);
    } 
  }
  */
  glEnd();
}

void MCPainter::drawParticleTheta(MonteCarloDebugger * debugger){
  int sizeMultiplier = 3 ; // for some reason GLUT displays correctly if every point in window is multiplied with this. TODO: find out why
  

  // draw x, y coordinates
  glBegin(GL_LINES); 
  {
    glColor3f(1.0,0.0,0.0); 
    glVertex2f(sizeMultiplier * glutGet(GLUT_WINDOW_WIDTH)/2, 0); 
    glVertex2f(sizeMultiplier * glutGet(GLUT_WINDOW_WIDTH)/2, sizeMultiplier * glutGet(GLUT_WINDOW_HEIGHT));
  }
  glEnd(); 
  
  glBegin(GL_LINES); 
  {
    glColor3f(1.0,0.0,0.0); 
    glVertex2f(0, sizeMultiplier * glutGet(GLUT_WINDOW_HEIGHT)/2); 
    glVertex2f(sizeMultiplier * glutGet(GLUT_WINDOW_WIDTH), sizeMultiplier * glutGet(GLUT_WINDOW_HEIGHT)/2); 
  }
  glEnd(); 

  glBegin(GL_POINTS); 
  {
    for (int i = 0; i < debugger->particles.size(); i++) {
      double c = debugger->particles[i].probability;
      glColor3f(.5*(1-c), 1 *c, 0);
      Position p = debugger->particles[i].getPosition();
      int len = sizeMultiplier * c * 100; 
      int xp = sizeMultiplier * glutGet(GLUT_WINDOW_WIDTH)/2 + len * cos(p.getTheta()) ;
      int yp = sizeMultiplier * glutGet(GLUT_WINDOW_HEIGHT)/2 + len * sin(p.getTheta()) ;
      //cout << "particle " << i << "\tprob: " << c << "\tlen: " << len << "\txp: " << xp << "\typ: " << yp << endl;
      glVertex2f(xp,yp);
    }
  }
  glEnd();
}

void MCPainter::drawPlayerBlobs(Localization * itl){
  BlobfinderProxy * bfp = itl->getBlobfinderProxy();
  int wMult = 2 ;      // width multiplier: the width ratio of display window to camera image 
  int hMult = 2 ;      // height multiplier: the height ratio of display window to camera image 
  drawWindowBorders(wMult, hMult); 
  for (int i = 0; i < bfp->GetCount(); i++) {
    player_blobfinder_blob p_blob = bfp->GetBlob(i); 
    int color = itl->getBlobColor(p_blob);
    double r, g, b;  
    switch(color){
    case 0: // pink 255 0 255
      r = 1; g = 0; b = 1; 
      break ; 
    case 1: // yellow 255 255 0
      r = 1; g = 1; b = 0; 
      break;
    case 2: // blue 0 0 255
      r = 0; g = 0; b = 1; 
      break;
    case 3: // green 0 255 0
      r = 0; g = 1; b = 0; 
      break; 
    case 4: // orange 255 125 0
      r = 1; g = 0.5; b = 0; 
      break; 
    default: 
      r = 0; g = 0; b = 0; 
    }

    int x = p_blob.x; 
    int y = p_blob.y; 
    int height_half = static_cast<int>((p_blob.bottom - p_blob.top)/2) ; 
    int width_half = static_cast<int>((p_blob.right - p_blob.left)/2) ;
    int win_h = glutGet(GLUT_WINDOW_HEIGHT); 

    // draws 2 rectangles so that the blob appears thick
    glBegin(GL_LINES);
    {
      glColor3f(r, g, b); 
      glVertex2i( (x - width_half) * wMult, win_h - (y - height_half) * hMult ); 
      glVertex2i( (x - width_half) * wMult, win_h - (y + height_half) * hMult ); 

      glVertex2i( (x - width_half) * wMult, win_h - (y + height_half) * hMult ); 
      glVertex2i( (x + width_half) * wMult, win_h - (y + height_half) * hMult ); 

      glVertex2i( (x + width_half) * wMult, win_h - (y + height_half) * hMult ); 
      glVertex2i( (x + width_half) * wMult, win_h - (y - height_half) * hMult ); 

      glVertex2i( (x + width_half) * wMult, win_h - (y - height_half) * hMult ); 
      glVertex2i( (x - width_half) * wMult, win_h - (y - height_half) * hMult ); 

      // inner rectangle
      //glColor3f(.5,.5,.5);
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y - height_half) * hMult + 1 ); 
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y + height_half) * hMult - 1); 

      glVertex2i( (x - width_half) * wMult + 1, win_h - (y + height_half) * hMult - 1); 
      glVertex2i( (x + width_half) * wMult - 1, win_h - (y + height_half) * hMult - 1); 

      glVertex2i( (x + width_half) * wMult - 1, win_h - (y + height_half) * hMult - 1); 
      glVertex2i( (x + width_half) * wMult - 1, win_h - (y - height_half) * hMult + 1); 

      glVertex2i( (x + width_half) * wMult - 1, win_h - (y - height_half) * hMult + 1); 
      glVertex2i( (x - width_half) * wMult + 1, win_h - (y - height_half) * hMult + 1); 
    }
    glEnd();
  }
}

void MCPainter::drawObservationBlobs(Localization * itl){
  vector<Observation> obs = itl->getObservations(); 
}

void MCPainter::drawCameraImage(Localization * itl){
  /*CameraProxy * cam = itl->getCameraProxy();
  uint cam_width = cam->GetWidth();
  uint cam_height = cam->GetHeight();
  uint cam_depth = cam->GetDepth();

  uint8_t* imgBuffer = new uint8_t[cam_width * cam_height * cam_depth];

  cam->Decompress(); 
  cam->GetImage(imgBuffer);

  
  IplImage * img = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U, 3); 

  for ( uint i = 0 ; i < cam_width ; i++ ){
    for ( uint j = 0 ; j < cam_height ; j++ ){
      img->imageData[cam_width * j*3 + i*3 + 0] = (char)imgBuffer[cam_width * j*3 + i*3 + 2]; 
      img->imageData[cam_width * j*3 + i*3 + 1] = (char)imgBuffer[cam_width * j*3 + i*3 + 1]; 
      img->imageData[cam_width * j*3 + i*3 + 2] = (char)imgBuffer[cam_width * j*3 + i*3 + 0]; 
    }
  }
  
  delete[] imgBuffer; 
  //cvReleaseImage(&img);
  */
}

void MCPainter::drawWindowBorders(int wm, int hm){
  glBegin(GL_LINES);
  {
    glColor3f(0,1,0);
    glVertex2f( 0, 0 ) ; 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, 0 ); 

    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, 0 ); 
    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 

    glVertex2f( glutGet(GLUT_WINDOW_WIDTH) * wm, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 
    glVertex2f( 0, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 

    glVertex2f( 0, glutGet(GLUT_WINDOW_HEIGHT) * hm ); 
    glVertex2f( 0, 0 ) ; 
  }
  glEnd();
}

void MCPainter::drawObservations(MonteCarloDebugger * debugger, Localization * itl) {
  vector<Observation> obs = debugger->getObservations();
  glBegin(GL_LINES);
  {
    for (int i = 0; i < obs.size(); i++) {
      glColor3f(1, 0, 1);
      
      Position position = itl->getPosition();
      glVertex2f(position.getX(), position.getY());

      int lineLen = 1000;
      int x = position.getX() + lineLen * cos(obs[i].getBearing() + position.getTheta());
      int y = position.getY() + lineLen * sin(obs[i].getBearing() + position.getTheta());
      glVertex2f(x, y);
    }
  }
  glEnd();
}

/* current call for this function does not include a realPosition. That
   should be added if there is such an info coming from an overhead camera 
   or something of the sort 
*/
void MCPainter::drawPosition(Localization * itl, Position realPosition) {
  //draw estimated position
  glBegin(GL_POLYGON);
  {
    Position p = itl->getPosition();
    double conf = itl->getConfidence() / 2; // % 0-50
    
    glColor3f(1, 0.75 - 4*conf, 0.75 - 4*conf);  // more confident the robot is about its localization more red it will get
    int lineLen = 15;
    int x1 = p.getX() - lineLen * cos(p.getTheta() + .3);
    int y1 = p.getY() - lineLen * sin(p.getTheta() + .3);
    int x2 = p.getX() - lineLen * cos(p.getTheta() - .3);
    int y2 = p.getY() - lineLen * sin(p.getTheta() - .3);
    glVertex2f(p.getX(), p.getY());
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glVertex2f(p.getX(), p.getY());
  }
  glEnd();

  //draw real position
  glBegin(GL_LINES);
  {
    Position p = realPosition;
    glColor3f(0, 0, 1);
    int lineLen = 100;
    int x = p.getX() + lineLen * cos(p.getTheta());
    int y = p.getY() + lineLen * sin(p.getTheta());
    glVertex2f(p.getX(), p.getY());
    glVertex2f(x, y);
  }
  glEnd();
}


