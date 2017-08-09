#include "drake/examples/pr2/src/utils/robot_state_lcm.h"

#include <gtest/gtest.h>

TEST (RobotStateLcmTest, Send) {
  const int num_joints = 7
  RobotStateSender sender(num_joints);
  std::unique_ptr<systems::Context<double>>
      context = sender.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      sender.AllocateOutput(*context);

  Eigen::VectorXd state_input(num_joints);
  state_input << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  context->FixInputPort(sender.get_input_port(0).get_index(), state_input);

  lcmt_robot_state state = sender.MakeOutputStatus();
  ASSERT_EQ(status.num_joints, num_joints);
  for (int i = 0; i < kNumJoints; i++) {
    EXPECT_EQ(state.joint_position[i], state_input(i));
  }
}

