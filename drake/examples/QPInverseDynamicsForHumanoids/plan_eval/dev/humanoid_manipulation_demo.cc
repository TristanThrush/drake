/**
 * @brief This is a demo program that sends a full body manipulation plan
 * encoded as a robotlocomotion::robot_plan_t to ValkyrieController. Upon
 * receiving the plan, the Valkyrie robot should return to the nominal
 * configuration except the right shoulder pitch joint.
 */
#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using std::default_random_engine;

namespace drake {
namespace {

void send_manip_message() {
  RigidBodyTree<double> robot;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_limited_finger_movement_no_collisions.urdf",
      multibody::joints::kRollPitchYaw, &robot);

  //DRAKE_DEMAND(examples::valkyrie::kRPYValkyrieDof ==
               //robot.get_num_positions());
  VectorX<double> q(40);
  q << 0, 0, 0, 0., 0., 0., 0, 0, 0, 0, 0.300196631343025, 1.25, 0, 0.785398163397448, 1.571, 0, 0, 0.300196631343025, -1.25, 0, -0.785398163397448, 1.571, 0, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0, 0, 0; // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());

  VectorX<double> q2(40);
  q2 << 0, 0, 0, 0., 0., 0., 0, 0, 0, 0, 0.300196631343025, 1.25, 0, 0.785398163397448, 1.571, 0, 0, -0.200196631343025, -1.25, 0, -0.785398163397448, 1.571, 0, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0, -0.49, 1.205, -0.71, 0, -1.4, 0, 0, -1.5; // 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, -0.5, -0.5, -0.5, -0.08, -0.5, -0.5, -0.5, -0.5, -0.5, -0.2, -0.5, -0.5;
  VectorX<double> v2 = VectorX<double>::Zero(robot.get_num_velocities());

  VectorX<double> q3(40);
  q2 << 0, 0, 0, 0., 0., 0., 0, 0, 0, 0, 0.300196631343025, 1.25, 0, 0.785398163397448, 1.571, 0, 0, -0.200196631343025, -1.25, 0, -0.785398163397448, 1.571, 0, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0, -0.49, 1.205, -0.71, 0, -1.4, 0, 0, -1.5; // 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, -0.5, -0.5, -0.5, -0.08, -0.5, -0.5, -0.5, -0.5, -0.5, -0.2, -0.5, -0.5;



  const manipulation::RobotStateLcmMessageTranslator translator(robot);

  // There needs to be at least 1 knot point, the controller will insert its
  // current desired q to the beginning to make the desired trajectories.
  //
  // Some notes about the message:
  // 1. The first timestamp needs to be bigger than 0.
  // 2. msg.utime needs to be different for different messages. The controller
  // ignores messages with the same utime.
  // 3. Assumes the message is packaged with a RPY floating joint.
  // 4. Assumes that the given knot points are stable given the current actual
  // contact points.
  // 5. Pelvis z and orientation + torso orientation are tracked in Cartesian
  // mode (These can be changed by the gains in the configuration file). CoM
  // in controlled by a LQR like controller, where the desired is given by the
  // knot points specified here.
  robotlocomotion::robot_plan_t msg{};
  msg.utime = time(NULL);
  msg.num_states = 2;
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 2);
  
  q2[10] -= 0.5;  // right shoulder pitch
  q3[10] -= 1.0;  
  translator.InitializeMessage(&(msg.plan[0]));
  translator.EncodeMessageKinematics(q2, v2, &(msg.plan[0]));
  msg.plan[0].utime = 2e6;
  
  //q2[10] -= 0.5;
  translator.InitializeMessage(&(msg.plan[1]));
  translator.EncodeMessageKinematics(q3, v2, &(msg.plan[1]));
  msg.plan[1].utime = 5e6;


  lcm::LCM lcm;
  lcm.publish("VALKYRIE_MANIP_PLAN", &msg);

  sleep(1);
}

}  // namespace
}  // namespace drake

int main() {
  drake::send_manip_message();
  return 0;
}
