#include "drake/examples/bhpn_drake_interface/simulation_utils/world_sim_tree_builder.h"
#include "drake/examples/bhpn_drake_interface/simulation_utils/sim_diagram_builder.h"
#include "drake/examples/bhpn_drake_interface/lcm_utils/robot_state_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;

namespace drake {
    namespace examples {
        namespace bhpn_drake_interface {

            std::unique_ptr <RigidBodyTree<double>>
            build_world_tree(std::vector <ModelInstanceInfo<double>> *world_info, std::vector <std::string> urdf_paths,
                             std::vector <Vector3d> poses_xyz, std::vector <Vector3d> poses_rpy,
                             std::vector <std::string> fixed) {
                auto tree_builder = std::make_unique < WorldSimTreeBuilder < double >> ();
                for (auto urdf : urdf_paths) {
                    tree_builder->StoreModel(urdf, urdf);
                }
                world_info->clear();
                for (int index = 0; index < (int) urdf_paths.size(); index++) {
                    if (!(fixed[index]).compare("true")) {
                        int object_id = tree_builder->AddFixedModelInstance(urdf_paths[index], poses_xyz[index],
                                                                            poses_rpy[index]);
                        world_info->push_back(tree_builder->get_model_info_for_instance(object_id));
                    } else {
                        int object_id = tree_builder->AddFloatingModelInstance(urdf_paths[index],
                                                                               poses_xyz[index],
                                                                               poses_rpy[index]);
                        world_info->push_back(tree_builder->get_model_info_for_instance(object_id));
                    }
                }

                tree_builder->AddGround();
                return tree_builder->Build();
            }

            void main(int argc, char *argv[]) {

                //parse the arguments (the arguments are the urdf paths of objects to place
                //in the world - the first urdf should be the robot's urdf)
                std::vector <std::string> urdf_paths;
                std::vector <Vector3d> poses_xyz;
                std::vector <Vector3d> poses_rpy;
                std::vector <std::string> fixed;

                for (int index = 1; index < argc; index++) {
                    urdf_paths.push_back(argv[index]);
                    index++;
                    double x = std::stod(argv[index]);
                    index++;
                    double y = std::stod(argv[index]);
                    index++;
                    double z = std::stod(argv[index]);
                    poses_xyz.push_back(Vector3d(x, y, z));
                    index++;
                    double roll = std::stod(argv[index]);
                    index++;
                    double pitch = std::stod(argv[index]);
                    index++;
                    double yaw = std::stod(argv[index]);
                    poses_rpy.push_back(Vector3d(roll, pitch, yaw));
                    index++;
                    fixed.push_back(argv[index]);
                }

                //construct the world and initialize some other things
                drake::lcm::DrakeLcm lcm;
                std::vector <ModelInstanceInfo<double>> world_info;
                SimDiagramBuilder<double> builder;
                systems::DiagramBuilder<double> *diagram_builder = builder.get_mutable_builder();
                systems::RigidBodyPlant<double> *plant = builder.AddPlant(
                        build_world_tree(&world_info, urdf_paths, poses_xyz, poses_rpy, fixed));
                const int num_actuators = plant->model_instance_actuator_command_input_port(world_info[0].instance_id).size();
                std::cout << num_actuators << "\n";
                builder.AddVisualizer(&lcm);

                //give the robot a controller
                //TODO: give the ability to pass in the parameters of the controller, so that different robots can be switched in without limitation
                const VectorX<double> kp = VectorX<double>::Constant(num_actuators, 300.0);
                const VectorX<double> ki = VectorX<double>::Constant(num_actuators, 0.0);
                const VectorX<double> kd = VectorX<double>::Constant(num_actuators, 5.0);
                std::unique_ptr <systems::PidController<double>> controller_ptr =
                        std::make_unique < systems::PidController <
                        double >> (plant->get_rigid_body_tree().B.block(0, 0, num_actuators, num_actuators).inverse(),
                                MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp, ki, kd);
                auto controller =
                        builder.AddController < systems::PidController < double >> (
                                world_info[0].instance_id, std::move(controller_ptr));

                //set up communication with BHPN
                auto command_sub = diagram_builder->AddSystem(
                        systems::lcm::LcmSubscriberSystem::Make<lcmt_robot_state>(
                                "BHPN_ROBOT_STATE_COMMAND", &lcm));
                command_sub->set_name("command_subscriber");
                auto command_receiver =
                        diagram_builder->AddSystem<RobotStateReceiver>(num_actuators);
                command_receiver->set_name("command_receiver");
                auto status_pub = diagram_builder->AddSystem(
                        systems::lcm::LcmPublisherSystem::Make<lcmt_robot_state>(
                                "DRAKE_ROBOT_STATE", &lcm));
                status_pub->set_name("status_publisher");
                status_pub->set_publish_period(lcmStatusPeriod);
                auto status_sender = diagram_builder->AddSystem<RobotStateSender>(num_actuators);
                status_sender->set_name("status_sender");
                diagram_builder->Connect(command_sub->get_output_port(0),
                                         command_receiver->get_input_port(0));
                diagram_builder->Connect(command_receiver->get_output_port(0),
                                         controller->get_input_port_desired_state());
                diagram_builder->Connect(plant->model_instance_state_output_port(world_info[0].instance_id),
                                         status_sender->get_state_input_port());
                diagram_builder->Connect(command_receiver->get_output_port(0),
                                         status_sender->get_command_input_port());
                diagram_builder->Connect(status_sender->get_output_port(0),
                                         status_pub->get_input_port(0));
              
                //start the simulation
                lcm.StartReceiveThread();
                std::unique_ptr <systems::Diagram<double>> diagram = builder.Build();
                systems::Simulator<double> simulator(*diagram);
                simulator.Initialize();
                simulator.set_target_realtime_rate(1.0);
                simulator.StepTo(50000);
            }
        }  // namespace bhpn_drake_interface
    }  // namespace examples
}  // namespace drake

int main(int argc, char *argv[]) {
    drake::examples::bhpn_drake_interface::main(argc, argv);
    return 0;
}


