float64 tMotor_i        # subsystem_buffer{type: port; validity: tMotor_i_valid}
bool tMotor_i_valid

CommandMotor hpMotor    # subsystem_buffer{type: container; validity: hpMotor_valid}
bool hpMotor_valid

CommandMotor htMotor    # subsystem_buffer{type: container; validity: htMotor_valid}
bool htMotor_valid

CommandArm lArm         # subsystem_buffer{type: container; validity: lArm_valid}
bool lArm_valid

CommandArm rArm         # subsystem_buffer{type: container; validity: rArm_valid}
bool rArm_valid

std_msgs/Int32 lArmFri  # subsystem_buffer{type: port; validity: lArmFri_valid}
bool lArmFri_valid

std_msgs/Int32 rArmFri  # subsystem_buffer{type: port; validity: rArmFri_valid}
bool rArmFri_valid

int32 tact              # subsystem_buffer{type: port; validity: tact_valid}
bool tact_valid

CommandHand rHand       # subsystem_buffer{type: container; validity: rHand_valid}
bool rHand_valid

CommandHand lHand       # subsystem_buffer{type: container; validity: lHand_valid}
bool lHand_valid

