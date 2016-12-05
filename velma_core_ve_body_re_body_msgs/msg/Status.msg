StatusArm rArm                  # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

StatusArmFriRobot rArmFriRobot      # subsystem_buffer{type: port; data_type: tFriRobotState; validity: rArmFriRobot_valid; includes: <kuka_lwr_fri/friComm.h>}
bool rArmFriRobot_valid

StatusArmFriIntf rArmFriIntf        # subsystem_buffer{type: port; data_type: tFriIntfState; validity: rArmFriIntf_valid; includes: <kuka_lwr_fri/friComm.h>}
bool rArmFriIntf_valid

StatusArm lArm                  # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

StatusArmFriRobot lArmFriRobot      # subsystem_buffer{type: port; data_type: tFriRobotState; validity: lArmFriRobot_valid; includes: <kuka_lwr_fri/friComm.h>}
bool lArmFriRobot_valid

StatusArmFriIntf lArmFriIntf        # subsystem_buffer{type: port; data_type: tFriIntfState; validity: lArmFriIntf_valid; includes: <kuka_lwr_fri/friComm.h>}
bool lArmFriIntf_valid

StatusHand rHand                # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

StatusHand lHand                # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

StatusFT rFt                    # subsystem_buffer{type: container; validity: rFt_valid}
bool rFt_valid

StatusFT lFt                    # subsystem_buffer{type: container; validity: lFt_valid}
bool lFt_valid

StatusMotor tMotor              # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

StatusMotor hpMotor             # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

StatusMotor htMotor             # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

barrett_hand_controller_msgs/BHPressureState rHand_p    # subsystem_buffer{type: port; data_type: barrett_hand_controller_msgs::BHPressureState; validity: rHand_p_valid}
bool rHand_p_valid

geometry_msgs/Wrench[3] lHand_f                         # subsystem_buffer{type: port; data_type: geometry_msgs::Wrench[3]; validity: lHand_f_valid}
bool lHand_f_valid

