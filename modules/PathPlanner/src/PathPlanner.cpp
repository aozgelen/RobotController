/*! 
  \file PathPlanner.cpp
  \addtogroup PathPlanner
  @{
 */ 

#include "PathPlanner.h"
#include <limits.h>

/*! 
  \brief Calculates the shortest path from a start point to a destination point on the navigation graph. 

  This function makes necessary calls to populate the PathPlanner::path list, which is a list of node indexes. The list doesn't contain the source (start point) and the target (destination point), only the nodes that the robot needs to get to in sequence, in order to reach its destination.

  First A* algorithm is run on the navigation graph and then the path is smoothed by removing unnecessary waypoints.
  
  \b Warning a source and a target must be specified prior to this function call. 

 */
void PathPlanner::calcPath(){
  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    Node s, t;
    if ( navGraph.isNode(source) )
      s = source ; 
    else {
      s = getClosestNode(source); 
    }

    if ( navGraph.isNode(target) )
      t = target ; 
    else {
      t = getClosestNode(target);
    }

    astar newsearch(navGraph, s, t, 2);
    path = newsearch.getPathToTarget();
    printPath();
    //head = path.begin();
    objectiveSet = false;
    pathCompleted = false;
    smoothPath(); 
  }
}

/*! 
  \brief Returns the closest Node in the navigation graph to an arbirary \f$(x, y)\f$ position.  
  
  \param Node, this can be any \f$(x, y)\f$ on the map 
  \return Node, this is a member node of the navigation graph

  This function is used to determine the closest member nodes of the navigation graph to target and the source, since the A* runs only over the member nodes. 

 */ 
Node PathPlanner::getClosestNode(Node n){
  vector<Node> nodes = navGraph.getNodes(); 
  vector<Node>::iterator iter; 
  double dist = INT_MAX;
  Node temp; 
  for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
    double d = get_euclidian_distance(iter->getX(), iter->getY(), n.getX(), n.getY() );
    if ( d < dist ) {
      dist = d; 
      temp = *iter; 
    }
  }
  return temp;
}

/*! 
  \brief Used to remove extra points off of the path
 */
void PathPlanner::smoothPath(){
  int proximity = navGraph.getProximity();

  //list<int> tempPath; 

  if ( path.size() > 1 ) {
    // smooth the end points 
    // if getting to second node from source is shorter and not obstructed remove first node. 
    list<int>::iterator iter = path.begin(); 
    Node first = navGraph.getNode(*iter++);
    Node second = navGraph.getNode(*iter);
    iter--; // point back to the first element

    cout << "source: "; 
    source.printNode(); 
    cout << " - first: " ; 
    first.printNode(); 
    cout << " - second: " ; 
    second.printNode(); 
    cout << endl ;
    
    if ( !navGraph.isPathObstructed(source.getX(), source.getY(), second.getX(), second.getY()) &&
	 get_euclidian_distance( source.getX(), source.getY(), second.getX(), second.getY() ) + proximity / 4 < 
	 ( get_euclidian_distance( source.getX(), source.getY(), first.getX(), first.getY() ) + 
	   get_euclidian_distance( first.getX(), first.getY(), second.getX(), second.getY() ) )){
      cout << "Erasing first" << endl;
      path.erase(iter);
    }
  }
  
  cout << "after checking the first element: " << endl; 
  printPath(); 

  if ( path.size() > 1 ) {
    // if getting to second node from source is shorter and not obstructed remove first node. 
    list<int>::iterator iter = path.end(); 
    Node last = navGraph.getNode(*(--iter)); 
    Node onebeforelast = navGraph.getNode(*(--iter)); 
    iter++;

    cout << "onebeforelast: "; 
    onebeforelast.printNode(); 
    cout << " - last: " ; 
    last.printNode(); 
    cout << " - target: " ; 
    target.printNode(); 
    cout << endl;

    if ( !navGraph.isPathObstructed(onebeforelast.getX(), onebeforelast.getY(), target.getX(), target.getY()) &&
	 get_euclidian_distance( onebeforelast.getX(), onebeforelast.getY(), target.getX(), target.getY() ) + proximity / 4 < 
	 ( get_euclidian_distance( onebeforelast.getX(), onebeforelast.getY(), last.getX(), last.getY() ) + 
	   get_euclidian_distance( last.getX(), last.getY(), target.getX(), target.getY() ) )){
      cout << "Erasing last" << endl ;
      path.erase(iter);
    }
  }

  cout << "after checking the last element: " << endl; 
  printPath(); 

  
  // remove points which are unnecessary deviations from main path ( by products of astar )
  // TODO: test, still some bugs 
  /*
  if ( path.size() > 2 ) {
    list<int>::iterator end, head, proc, proc_end;
    Node nH, nE, nP, nPE; 
    end = path.end();
    head = path.begin(); 
    proc = head; 
    proc++; 
    proc_end = proc; 
    proc_end++; 

    cout << "Path size: " << path.size() << endl;
    
    while ( proc != proc_end ){
      cout << "head: " ; 
      nH = navGraph.getNode(*head); 
      nH.printNode();
      cout << "\tproc: " ;
      nP = navGraph.getNode(*proc); 
      nP.printNode(); 
      cout << "\tproc_end: "; 
      nPE = navGraph.getNode(*proc_end); 
      nPE.printNode();
      cout << "\tend: " ; 
      nE = navGraph.getNode(*end); 
      nE.printNode(); 
      cout << endl; 

      if ( !navGraph.isPathObstructed(nH.getX(), nH.getY(), nPE.getX(), nPE.getY()) &&
	   get_euclidian_distance( nH.getX(), nH.getY(), nPE.getX(), nPE.getY() ) < 
	   ( get_euclidian_distance( nH.getX(), nH.getY(), nP.getX(), nP.getY() ) + 
	     get_euclidian_distance( nP.getX(), nP.getY(), nPE.getX(), nPE.getY() ) )){
	cout << "the proc is extra, removing it from path" << endl; 
	proc = path.erase(proc);
	nP = navGraph.getNode(*proc);
	cout << "new proc: " ; 
	nP.printNode(); 
	cout << endl;
      }
      else {
	head = proc; 
	proc++; 
	nH = navGraph.getNode(*head); 
	nP = navGraph.getNode(*proc); 
	cout << "the proc is NOT extra, new head: " ; 
	nH.printNode(); 
	cout << "\tnew proc: " ; 
	nP.printNode(); 
	cout << endl; 
      }
      if ( *proc_end != *end ){ 
	cout << "proc_end still didn't reach the end of the list, new proc_end: "; 
	proc_end++ ;
	nPE = navGraph.getNode(*proc_end); 
	nPE.printNode(); 
	cout << endl;
      }
    }
  }
  */
  // remove points which don't signify direction change. this is done for path segments of length < 1m.
  // TODO: test 
  /*
  if ( path.size() > 2 ) {
    list<int>::iterator end, head, proc, proc_end;
    Node nH, nE, nP, nPE; 
    end = path.end();
    nE = navGraph.getNode(*(--end)); 
    head = path.begin(); 
    proc = ++head; 
    head--; 
    proc_end = ++proc; 
    proc--; 
  
    cout << "path.size(): " << path.size() << endl; 
  
    while( proc != proc_end ){
      nH = navGraph.getNode(*head); 
      nP = navGraph.getNode(*proc); 
      nPE = navGraph.getNode(*proc_end);
      if ( get_euclidian_distance( nH.getX(), nH.getY(), nPE.getX(), nPE.getY() ) < 100 &&
	   ( nH.getX() - nP.getX() == nP.getX() - nPE.getX() && nH.getY() - nP.getY() == nP.getY() - nPE.getY() )) {
	path.erase(proc); 
      }
      else{
	head = proc; 
	proc++; 
      } 	
      if ( *proc_end != *end ) 
	proc_end++;
    }
  }
  */
  cout << "after smoothing: " << endl ; 
  printPath();
}

/*! 
  \brief Prints the path node by node
 */
void PathPlanner::printPath(){
  list<int>::iterator it ;
  for ( it = path.begin(); it != path.end(); it++ ){
    navGraph.getNode(*it).printNode() ; 
    cout << endl;
  }
}

/*! @} */
