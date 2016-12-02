CommandSimple rTact     # subsystem_buffer{type: container; validity: rTact_valid}
bool rTact_valid

CommandMotor tMotor     # subsystem_buffer{type: container; validity: tMotor_valid}
bool tMotor_valid

CommandMotor hpMotor    # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

CommandMotor htMotor    # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

std_msgs/Int32 lArmFri  # subsystem_buffer{type: port; data_type: std_msgs::Int32; validity: lArmFri_valid}
bool lArmFri_valid

std_msgs/Int32 rArmFri  # subsystem_buffer{type: port; data_type: std_msgs::Int32; validity: rArmFri_valid}
bool rArmFri_valid

CommandHand lHand       # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

CommandHand rHand       # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

