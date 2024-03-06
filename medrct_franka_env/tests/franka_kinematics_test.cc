// #include <gtest/gtest.h>

// #include <memory>

// #include <medrct/log.hh>
// #include <medrct/types/types.hh>

// #include <medrct_dvrk_env/psm_tool_params.hh>
// #include <medrct_dvrk_env/psm_kinematics_solver.hh>
// #include <medrct_dvrk_env/psm_kinematics_factory.hh>

// using namespace medrct;
// using namespace medrct::env;

// struct PsmKinematicsTest : public ::testing::Test
// {
// protected:
//   PsmKinematicsData psm_kin_data;
//   std::unique_ptr<PsmForwardKinematics> psm_forward_kin;
//   std::unique_ptr<PsmInverseKinematics> psm_inverse_kin;
//   virtual void SetUp()
//   {
//     psm_kin_data.init(LND400006());
//     psm_forward_kin = std::make_unique<PsmForwardKinematics>(psm_kin_data);
//     psm_inverse_kin = std::make_unique<PsmInverseKinematics>(psm_kin_data);
//   }

//   virtual void TearDown() {}
// };

// TEST_F(PsmKinematicsTest, testFKIKOutput)
// {
//   auto testFKIK = [&](const std::vector<real_t>& joint_positions) {
//     Transform output_fk;
//     int fwd_ret = psm_forward_kin->computeFK(output_fk, joint_positions);
//     ASSERT_EQ(fwd_ret, 0);
//     auto ik_solutions = psm_inverse_kin->computeIK(output_fk);
//     ASSERT_EQ(ik_solutions.size(), 1);
//     std::vector<real_t> out_jps = ik_solutions[0];
//     medrctlog::info("output_fk: \n{}", output_fk);
//     medrctlog::info(
//         "joint_positions: \n{} {} {} {} {} {}",
//         joint_positions[0],
//         joint_positions[1],
//         joint_positions[2],
//         joint_positions[3],
//         joint_positions[4],
//         joint_positions[5]);
//     medrctlog::info(
//         "out_joint_positions: \n{} {} {} {} {} {}",
//         out_jps[0],
//         out_jps[1],
//         out_jps[2],
//         out_jps[3],
//         out_jps[4],
//         out_jps[5]);

//     for (unsigned int i = 0; i < joint_positions.size(); ++i)
//     {
//       ASSERT_NEAR(out_jps[i], joint_positions[i], 10e-3);
//     }
//     medrctlog::info("--------------------------------");
//   };
//   real_t joint_change = 0.1;
//   std::vector<real_t> joint_pos = {0.0, 0.0, joint_change, 0.0, 0.0, 0.0};
//   testFKIK(joint_pos);
//   joint_pos[1] = -joint_change;
//   testFKIK(joint_pos);
//   joint_pos[0] = joint_change;
//   testFKIK(joint_pos);
//   joint_pos[3] = joint_change;
//   testFKIK(joint_pos);
//   joint_pos[4] = joint_change;
//   testFKIK(joint_pos);
//   joint_pos[5] = joint_change;
//   testFKIK(joint_pos);
// }
