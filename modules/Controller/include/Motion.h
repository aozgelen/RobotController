
class Motion {
  Position2dProxy p2d; 
public: 
  Motion();
  ~Motion();
  void goto(Position); 
  void goto(Pose); 
  void setSpeed(double, double, double); 
  Pose getLastMove(); 
}
