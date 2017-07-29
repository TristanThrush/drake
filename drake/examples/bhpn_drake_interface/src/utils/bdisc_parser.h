#include <fstream>

namespace bdisc_parser{

struct simulator_conf {
  Eigen::Vector3d initial_robot_pose_xyz;
  Eigen::Vector3d initial_robot_pose_rpy;
  Eigen::VectorXd initial_robot_joint_positions;
  std::vector<std::string> object_names;
  std::vector<std::string> object_description_paths;
  std::vector<Eigen::Vector3d> initial_object_poses_xyz;
  std::vector<Eigen::Vector3d> initial_object_poses_rpy;
  std::vector<bool> object_fixed;
};

simulator_conf parse(std::string conf_file_path){

  simulator_conf conf;

  // Parse the simulator conf file.
  std::ifstream input_stream(conf_file_path);
  if (!input_stream) std::cerr << "Can't open simulator config file.";
  std::vector<std::string> simulator_conf_lines;
  std::string line;
  while (getline(input_stream, line)) {
    simulator_conf_lines.push_back(line);
  }

  // Get everything in string form
  std::istringstream iss(simulator_conf_lines[0]);
  std::vector<std::string> initial_robot_pose{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
  conf.initial_robot_pose_xyz << std::stod(initial_robot_pose[0]), std::stod(initial_robot_pose[1]), std::stod(initial_robot_pose[2]);
   conf.initial_robot_pose_rpy << std::stod(initial_robot_pose[3]), std::stod(initial_robot_pose[4]), std::stod(initial_robot_pose[5]);
  
  iss = std::istringstream(simulator_conf_lines[1]);
  std::vector<std::string> initial_joint_positions_strings{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
  std::vector<double> initial_joint_positions_temp;
  for (std::string position : initial_joint_positions_strings){
    initial_joint_positions_temp.push_back(std::stod(position));
  }
  conf.initial_robot_joint_positions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(initial_joint_positions_temp.data(), initial_joint_positions_temp.size());

  for (int index = 2; index < (int)simulator_conf_lines.size(); index ++){
    iss = std::istringstream(simulator_conf_lines[index]);
    std::vector<std::string> object{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
    conf.object_names.push_back(object[0]);
    conf.object_description_paths.push_back(object[1]);
    Eigen::Vector3d pose_xyz;
    pose_xyz << std::stod(object[2]), std::stod(object[3]), std::stod(object[4]);
    conf.initial_object_poses_xyz.push_back(pose_xyz);
    Eigen::Vector3d pose_rpy;
    pose_rpy << std::stod(object[5]), std::stod(object[6]), std::stod(object[7]);
    conf.initial_object_poses_rpy.push_back(pose_rpy);
    conf.object_fixed.push_back(!object[8].compare("true"));
  }

  return conf;

}


}


