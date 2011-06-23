

class Robot {
  PlayerClient * pc;

  Motion * mot; 
  Localization * loc;
  Perception * per; 

  Pose currentPos; 
  Pose targetPos; 
  
public: 
  Robot(PlayerClient& p); 
  ~Robot();

  // high level actions - robot specific
  virtual void sensorSweep() = 0 ; 
  virtual void localize() = 0 ; 

 
  // low level actions - all robots provide
  void goto_map_location(Position);   // position -> (x, y)
  void goto_map_location(Pose);       // pose -> (x, y, theta)
  
  void goto_position(Position);     // no obs. avoidance
  void goto_position(Pose);         
  void goto_position_safe(Position);  // avoid obstacles 
  void goto_position_safe(Pose);   

  void move_forward(); 
  void move_forward_safe(); 
  void move_forward(double); 
  void move_forward_safe(double); 

  void turn_right(); 
  void turn_right(double); 

  void turn_left(); 
  void turn_left(double); 

  void move_back(); 
  void move_back(double);
  void turn_back();

}
