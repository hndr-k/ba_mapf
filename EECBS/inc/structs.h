 #ifndef STRUCTS_H
 #define STRUCTS_H

struct rosAgent{
  int id;
  int start_x;
  int start_y;
  int goal_x;
  int goal_y;
};

struct rosPath{
  std::vector<int> x_poses;
  std::vector<int> y_poses;
  int robot_id;
};

#endif