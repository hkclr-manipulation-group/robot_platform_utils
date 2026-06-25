#include "cuarm_message_handler.h"

#include "time_utils.h"

#include <iostream>
// #include <stdexcept>

// #include <typeinfo>
// #include <stdio.h>
// #include <inttypes.h>
// #include <unistd.h>		   // Needed for sysconf(int name);

/* gettimeofday() does not appear on linux without this. */
// #define _BSD_SOURCE
int CuarmMessageHandler::get_next_data_index(char* receive_buffer, int index){
    while (receive_buffer[index] && receive_buffer[index] != '#') 
        index ++; 
    return index + 1;
}

void CuarmMessageHandler::pack_panel_command(PanelCommand* panel, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%ld#", panel->send_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", panel->sequence_id);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->connection_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->need_setting_update);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->simulation);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->motion_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->task_orien_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->reset_control_mem);
    
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->target_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->actuator_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->interpolation_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->interpolation_speed_ratio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->InterpolationAccTime);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->InterpolationConstVelTime);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->NoneInterpolationSaturationRatio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->reset_interpolation);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->control_algorithm);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->enable_recording);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->playback_cmd);

    //Arm related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->arm_target_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->ArmSize);
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->JointSize[arm_i]);
        snprintf(&(send_buffer[strlen(send_buffer)]), MAX_ROBOTNAME_SIZE+1, "%s#", panel->RobotName[arm_i]);
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->enable_joint[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->MotorCmd[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->JointCmd[arm_i][i]);
        }
        for (int i = 0; i <6; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->TaskCmd[arm_i][i]);
        }
    }

    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->ArmWaypointSize);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->ArmWaypointDt);
    for (int point_i =0; point_i < panel->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
            for (int i =0; i < panel->JointSize[arm_i]; i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->ArmWaypointCmd[point_i][arm_i][i]);
            }
        }
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->ArmWaypointInterpolationTime[point_i]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->ArmWaypointInterpolationSpeedRatio[point_i]);
    }

    //Gripper related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->GripperSize);
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->GripperJointSize[gri_i]);
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->GripperJointCmd[gri_i][i]);
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][0]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][1]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][2]);
    }
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_velocity[arm_i][i][1]);
        }
    }
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->gripper_soft_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->gripper_soft_limit_position[gri_i][i][1]);
        }
    }
}

void CuarmMessageHandler::unpack_panel_command(PanelCommand* panel, char* receive_buffer){
    int index = 0;
    char robot_name_format[10];
    snprintf(robot_name_format, sizeof(robot_name_format), "%%%d[^#]", MAX_ROBOTNAME_SIZE+1);
    sscanf(receive_buffer, "%ld#", &(panel->send_timestamp));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(panel->sequence_id));
    scan_type<ConnectionState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->connection_state);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->need_setting_update);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->simulation);
    scan_type<MotionControl, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->motion_type);
    scan_type<OrientControl, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->task_orien_type);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->reset_control_mem);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->target_type);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->actuator_mode);
    scan_type<InterpolationMethod, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->interpolation_type);

    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->interpolation_speed_ratio));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->InterpolationAccTime));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->InterpolationConstVelTime));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->NoneInterpolationSaturationRatio));
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->reset_interpolation);
    scan_type<ControlAlgorithm, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->control_algorithm);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->enable_recording);
    scan_type<PlaybackState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->playback_cmd);
    
    //Arm related
    scan_type<PanelTargetMode, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->arm_target_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->ArmSize));
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->JointSize[arm_i]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), robot_name_format, &(panel->RobotName[arm_i]));
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->enable_joint[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->MotorCmd[arm_i][i]));
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->JointCmd[arm_i][i]));
        }
        for (int i = 0; i < 6; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->TaskCmd[arm_i][i]));
        }
    }

    //Path related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->ArmWaypointSize));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointDt));
    for (int point_i = 0; point_i < panel->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
            for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointCmd[point_i][arm_i][i]));
            }
        }
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointInterpolationTime[point_i]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointInterpolationSpeedRatio[point_i]));
    }

    //Gripper related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->GripperSize));
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->GripperJointSize[gri_i]));
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->GripperJointCmd[gri_i][i]));
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][0]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][1]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][2]));
    }
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_velocity[arm_i][i][1]));
        }
    }
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->gripper_soft_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->gripper_soft_limit_position[gri_i][i][1]));
        }
    }
}

void CuarmMessageHandler::pack_panel_state(PanelState* state, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%d#", (int)state->state);
    
    //Arm
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->ArmSize);
    for (int arm_i = 0; arm_i < (int)state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->JointSize[arm_i]);
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPos[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointVel[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointTor[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->EEPose[arm_i][i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->ToolPose[arm_i][i]);
        }
    }

    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->interpolated_target[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperSize);
    if (state->GripperSize > 0){
        for (int gripper_i = 0; gripper_i < (int)state->GripperSize; gripper_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperJointSize[gripper_i]);
        }
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            for (int joint_i = 0; joint_i < (int)state->GripperJointSize[gripper_i]; joint_i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->GripperJointPos[gripper_i][joint_i]);
            }
        }
    }else{
        sprintf(&(send_buffer[strlen(send_buffer)]), "#");
        sprintf(&(send_buffer[strlen(send_buffer)]), "#");
    }
}

void CuarmMessageHandler::unpack_panel_state(PanelState* state, char* receive_buffer){
    int index = 0;
    scan_type<PanelRemoteState, int>(receive_buffer, index, state->state);

    //Arm 
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->ArmSize));
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->JointSize[arm_i]));
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointPos[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointVel[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointTor[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->EEPose[arm_i][i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->ToolPose[arm_i][i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->interpolated_target[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperSize));
    if (state->GripperSize > 0){
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperJointSize[gripper_i]));
        }
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            for (int joint_i = 0; joint_i < state->GripperJointSize[gripper_i]; joint_i++){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->GripperJointPos[gripper_i][joint_i]));
            }
        }
    }else{
        //If no gripper, skip the two fields for gripper joint size and position
        index = get_next_data_index(receive_buffer, index);
        index = get_next_data_index(receive_buffer, index);
    }
}

void CuarmMessageHandler::pack_planner_command(PlannerCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%hhd#", (uint8_t)command->connection_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->command_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmSize);
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->JointSize[arm_i]);
        for (int i = 0; i < command->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->Target[arm_i][i]);
        }
    }
}

void CuarmMessageHandler::unpack_planner_command(PlannerCommand* command, char* receive_buffer){
    int index = 0;
    scan_type<ConnectionState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->connection_state);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->command_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmSize));
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->JointSize[arm_i]));
        for (int i = 0; i < command->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(command->Target[arm_i][i]));
        }
    }
}

void CuarmMessageHandler::unpack_planner_state(PlannerState* state, char* receive_buffer){
    int index = 0;
    sscanf(receive_buffer, "%ld#", &(state->received_panel_command_timestamp));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%ld#", &(state->send_timestamp));
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->setting_update_finished);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->received_sequence_id));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->ArmSize));
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->JointSize[arm_i]));
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->enabled_joint[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointPos[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointVel[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointTor[arm_i][i]));
        }

        
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorPos[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorVel[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorCur[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorTor[arm_i][i]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            for (int j = 0; j < 7; j++) {
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointPose[arm_i][i][j]));
            }
        }
        for (int j = 0; j < 7; j++) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->EEPose[arm_i][j]));
        }
        for (int j = 0; j < 7; j++) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->ToolPose[arm_i][j]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->interpolated_target[arm_i][i]));
        }
    }

    //Gripper related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperSize));
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperJointSize[gri_i]));
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->GripperJointPos[gri_i][i]));
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][0]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][1]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][2]));
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_velocity[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_torque[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_torque[arm_i][i][1]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_velocity[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_torque[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_torque[arm_i][i][1]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_position[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_velocity[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_torque[arm_i][i]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_position[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_velocity[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_torque[arm_i][i]));
        }
    }

    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_soft_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_soft_limit_position[gri_i][i][1]));
        }
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_hard_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_hard_limit_position[gri_i][i][1]));
        }
    }

    scan_type<SystemState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->system_state);
    scan_type<PlanResult, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->plan_result);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->system_diagnostic_flags));
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->arm_joint_diagnostic_flags[arm_i][i]));
        }
    }
}

void CuarmMessageHandler::pack_planner_state(PlannerState* state, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%ld#", state->received_panel_command_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%ld#", state->send_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->setting_update_finished);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->received_sequence_id);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->ArmSize);
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->JointSize[arm_i]);
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->enabled_joint[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPos[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointVel[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointTor[arm_i][i]);
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorPos[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorVel[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorCur[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorTor[arm_i][i]);
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            for (int j = 0; j < 7; ++ j){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPose[arm_i][i][j]);
            }
        }
        for (int j = 0; j < 7; ++ j){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->EEPose[arm_i][j]);
        }
        for (int j = 0; j < 7; ++ j){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->ToolPose[arm_i][j]);
        }
        
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->interpolated_target[arm_i][i]);
        }
    }

    //Gripper related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperSize);
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperJointSize[gri_i]);
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->GripperJointPos[gri_i][i]);
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][0]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][1]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][2]);
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_velocity[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_torque[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_torque[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_velocity[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_torque[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_torque[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_position[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_velocity[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_torque[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_position[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_velocity[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_torque[arm_i][i]);
        }
    }
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_soft_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_soft_limit_position[gri_i][i][1]);
        }
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_hard_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_hard_limit_position[gri_i][i][1]);
        }
    }

    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->system_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->plan_result);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->system_diagnostic_flags);
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->arm_joint_diagnostic_flags[arm_i][i]);
        }
    }
}

void CuarmMessageHandler::pack_remote_panel_command(RemotePanelCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%f#", command->interpolation_acceleration_time);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->no_interpolation_max_velocity_ratio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->target_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->actuator_mode);
    
    //Arm
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->arm_target_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmSize);
    for (int arm_i = 0; arm_i < (int)command->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmJointSize[arm_i]);
    }
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)command->ArmJointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmJointCmd[arm_i][joint_i]);
        }
    }

    //Path
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmWaypointSize);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmWaypointDt);
    for (int point_i =0; point_i < command->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
            for (int joint_i =0; joint_i < (int)command->ArmJointSize[arm_i]; joint_i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmWaypointCmd[point_i][arm_i][joint_i]);
            }
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->GripperSize);
    for (int gripper_i = 0; gripper_i < (int)command->GripperSize; gripper_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->GripperJointSize[gripper_i]);
    }
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        for (int joint_i = 0; joint_i < (int)command->GripperJointSize[gripper_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->GripperJointCmd[gripper_i][joint_i]);
        }
    }
}

void CuarmMessageHandler::unpack_remote_panel_command(RemotePanelCommand* command, char* receive_buffer){
    int index = 0;
    sscanf(receive_buffer, "%f#", &(command->interpolation_acceleration_time));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->no_interpolation_max_velocity_ratio));
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->target_type);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->actuator_mode);

    //Arm 
    scan_type<PanelTargetMode, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->arm_target_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmSize));
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmJointSize[arm_i]));
    }
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < command->ArmJointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmJointCmd[arm_i][joint_i]));
        }
    }

    //Path
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmWaypointSize));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmWaypointDt));
    if (command->ArmWaypointSize > 0){
        for (int point_i =0; point_i < command->ArmWaypointSize; point_i++){
            for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
                for (int joint_i =0; joint_i < command->ArmJointSize[arm_i]; joint_i++){
                    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmWaypointCmd[point_i][arm_i][joint_i]));
                }
            }
        }
    }else{
        index = get_next_data_index(receive_buffer, index); //Remove a redundant #
    }
    
    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->GripperSize));
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->GripperJointSize[gripper_i]));
    }
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        for (int joint_i = 0; joint_i < command->GripperJointSize[gripper_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->GripperJointCmd[gripper_i][joint_i]));
        }
    }
}

void CuarmMessageHandler::pack_panel_config_command(PanelConfigCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);

    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->read_only);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", command->remote_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->arm_size);
    for (int arm_i = 0; arm_i < command->arm_size; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->arm_joint_size[arm_i]);
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_velocity[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_torque[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->gripper_size);
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->gripper_joint_size[gripper_i]);
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < command->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->gripper_soft_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < command->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->tool_offset[gripper_i][i]);
        }
    }

}

void CuarmMessageHandler::unpack_panel_config_command(PanelConfigCommand* command, char* receive_buffer){
    int index = 0;

    scan_type<bool, int>(receive_buffer, index, command->read_only);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(command->remote_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->arm_size));
    for (int arm_i = 0; arm_i < command->arm_size; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->arm_joint_size[arm_i]));
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_velocity[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_torque[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->gripper_size));
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->gripper_joint_size[gripper_i]));
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < command->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->gripper_soft_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        for (int i = 0; i < 3; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->tool_offset[gripper_i][i]));
        }
    }
}

void CuarmMessageHandler::pack_rt_config_state(RtConfigState* state, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);

    sprintf(&(send_buffer[strlen(send_buffer)]), "%s#", state->server_udp_ip);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%s#", state->client_udp_ip);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", state->server_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", state->client_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->arm_size);
    for (int arm_i = 0; arm_i < state->arm_size; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->arm_joint_size[arm_i]);
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_torque[arm_i][joint_i]);
        }
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_torque[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_position[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_torque[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_jump_limit_position[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_jump_limit_torque[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->gripper_size);
    for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->gripper_joint_size[gripper_i]);
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_joint_soft_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_joint_hard_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }

    for (int gripper_i = 0; gripper_i < state->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[gripper_i][i]);
        }
    }
}

void CuarmMessageHandler::unpack_rt_config_state(RtConfigState* state, char* receive_buffer){
    int index = 0;
    char ip_format[10];
    snprintf(ip_format, sizeof(ip_format), "%%%d[^#]", 14);

    sscanf(receive_buffer, ip_format, state->server_udp_ip);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), ip_format, state->client_udp_ip);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(state->server_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(state->client_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->arm_size));
    for (int arm_i = 0; arm_i < state->arm_size; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->arm_joint_size[arm_i]));
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_torque[arm_i][joint_i]));
        }
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_torque[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_position[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_torque[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_jump_limit_position[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_jump_limit_torque[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->gripper_size));
    for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->gripper_joint_size[gripper_i]));
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->gripper_joint_soft_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->gripper_joint_hard_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < state->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->tool_offset[gripper_i][i]));
        }
    }
}

