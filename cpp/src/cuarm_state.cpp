#include "cuarm_state.h"

void print_panel_command(PanelCommand* panel){
	int i, arm_i;
	printf("\nRobot: (send_timestamp: %ld, ArmSize: %d)\n", panel->send_timestamp, panel->ArmSize);
	printf("simulation: %d, motion_type: %d, task_orien_type: %d, reset_control_mem: %d \n", panel->simulation, (int)panel->motion_type, (int)panel->task_orien_type, panel->reset_control_mem);
	printf("target_type: %d, actuator_mode: %d, interpolation_type: %d\n", (int)panel->target_type, (int)panel->actuator_mode, (int)panel->interpolation_type);
    printf("InterpolationAccTime: %f, InterpolationConstVelTime: %f, reset_joint_interpolation: %d \n", panel->InterpolationAccTime, panel->InterpolationConstVelTime, panel->reset_joint_interpolation);
	printf("force_control: %d \n", (int)panel->force_control);
    
    for (arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        printf("Arm_i: %d\n", arm_i);
        printf("JointSize: %d\n", panel->JointSize[arm_i]);
        printf("\nMotorCmd      :");
        for (i = 0; i < panel->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)panel->MotorCmd[arm_i][i]);
        }
        printf("\nJointCmd      :");
        for (i = 0; i < panel->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)panel->JointCmd[arm_i][i]);
        }
        printf("\nTaskCmd       :");
        for (i = 0; i < 6; ++ i) {
            printf("%9.4lf ", (double)panel->TaskCmd[arm_i][i]);
        }
        printf("\n");
    }
}

void print_planner_state(PlannerState* robot){
	int i, j, arm_i;
	printf("\nRobot: (received_panel_command_timestamp: %ld ArmSize: %d)\n", robot->received_panel_command_timestamp, robot->ArmSize);
    printf("setting_update_finished: %d\n", (int)robot->setting_update_finished);
    
    for (arm_i = 0; arm_i < robot->ArmSize; arm_i++){
        printf("Arm_i: %hhd\n", arm_i);
        printf("JointSize: %hhd", robot->JointSize[arm_i]);
        printf("\ninterpolated_target       :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->interpolated_target[arm_i][i]);
        }

        printf("\nJointPos        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointPos[arm_i][i]);
        }
        printf("\nJointVel        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointVel[arm_i][i]);
        }
        printf("\nJointTor        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointTor[arm_i][i]);
        }

        printf("\nMotorPos        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorPos[arm_i][i]);
        }
        printf("\nMotorVel        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorVel[arm_i][i]);
        }
        printf("\nMotorTor        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorTor[arm_i][i]);
        }
        printf("\nEEPose            :\n");
        for (j=0; j<7; j++){
            printf("%9.4lf ", (double)robot->EEPose[arm_i][j]);
        }
        printf("\nToolPose            :\n");
        for (j=0; j<7; j++){
            printf("%9.4lf ", (double)robot->ToolPose[arm_i][j]);
        }

        printf("\n");
    }
}

void print_planner_command(PlannerCommand* command){
	int i, arm_i;
	printf("connection_state: %hhd, command_mode: %hhd, ArmSize: %hhd\n", command->connection_state, (int)command->command_mode, command->ArmSize);
    for (arm_i = 0; arm_i < command->ArmSize; arm_i++){
        printf("Arm %d: \n", arm_i);
        printf("JointSize %d:\n", command->JointSize[arm_i]);
        printf("    Target        :");
        for (i = 0; i < command->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)command->Target[arm_i][i]);
        }
        printf("\n");
    }
    // for (i=0; i<MAX_MESSAGE_SIZE; i++){
    //     r2->ErrorTitle[i] = r1->ErrorTitle[i];
    //     r2->ErrorMsg[i] = r1->ErrorMsg[i];
    // }
}

void print_rt_config_state(RtConfigState* state){
    printf("arm_size: %d\n", state->arm_size);
    printf("arm_joint_size: ");
    for (int i = 0; i < state->arm_size; i++){
        printf("%d ", state->arm_joint_size[i]);
    }
    printf("\n");

    printf("gripper_size: %d\n", state->gripper_size);
    printf("gripper_joint_size: ");
    for (int i = 0; i < state->gripper_size; i++){
        printf("%d ", state->gripper_joint_size[i]);
    }
    printf("\n");

    printf("\njoint_soft_limit_position:");
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < state->arm_size; j++){
            for (int k = 0; k < state->arm_joint_size[j]; k++){
                printf("%f ", state->joint_soft_limit_position[i][j][k]);
            }
        }
        printf("| ");
    }
    printf("\njoint_soft_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_soft_limit_velocity[i][j]);
        }
    }
    printf("\njoint_soft_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_soft_limit_torque[i][j]);
        }
    }

    printf("\njoint_hard_limit_position:");
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < state->arm_size; j++){
            for (int k = 0; k < state->arm_joint_size[i]; k++){
                printf("%f ", state->joint_hard_limit_position[i][j][k]);
            }
        }
        printf("| ");
    }
    printf("\njoint_hard_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_hard_limit_velocity[i][j]);
        }
    }
    printf("\njoint_hard_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_hard_limit_torque[i][j]);
        }
    }
    
    printf("\njoint_follow_limit_position:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_position[i][j]);
        }
    }
    printf("\njoint_follow_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_velocity[i][j]);
        }
    }
    printf("\njoint_follow_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_torque[i][j]);
        }
    }

    printf("\njoint_jump_limit_position:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_jump_limit_position[i][j]);
        }
    }
    printf("\njoint_jump_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_jump_limit_torque[i][j]);
        }
    }

    printf("\ntool_offset:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < 3; j++){
            printf("%f ", state->tool_offset[i][j]);
        }
    }
    printf("\n");
}