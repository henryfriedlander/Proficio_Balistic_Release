from ctypes import *



# def clear_flag_bits(A,bitmask): return ((A) &= ~(bitmask)) # macro
MAX_CONTROL_DIMS = 18 # Variable c_int '18'
MAX_EM_DIMS = MAX_CONTROL_DIMS # alias
# def set_flag_bits(A,bitmask): return ((A) |= (bitmask)) # macro
def check_flag_bits(A,bitmask): return (((A) & (bitmask)) == (bitmask)) # macro
class SAMPLE_HEADER(Structure):
    pass
SAMPLE_HEADER._fields_ = [
    ('SerialNo', c_int),
    ('Flags', c_int),
    ('DeltaTime', c_double),
]
class MOVEMENT_COMMAND_DATA(Structure):
    pass
MOVEMENT_COMMAND_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('sample_interval', c_double),
    ('pos', c_double * 18),
    ('vel', c_double * 18),
    ('controlledDims', c_int * 18),
    ('tag', c_char * 64),
]
class INPUT_DOF_DATA(Structure):
    pass
INPUT_DOF_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('dof_vals', c_double * 21),
    ('tag', c_char * 64),
]
class TRIAL_EVENT(Structure):
    pass
TRIAL_EVENT._fields_ = [
    ('event_time', c_double),
    ('event_code', c_int),
    ('reserved', c_int),
]
class PLAN_PROCESSOR_STATE(Structure):
    pass
PLAN_PROCESSOR_STATE._fields_ = [
    ('D', c_double * 576),
]
class AUTOMAGIC_CONTROLLER_PARAMS(Structure):
    pass
AUTOMAGIC_CONTROLLER_PARAMS._fields_ = [
    ('virtualPadPosition', c_double * 18),
    ('centralPointPosition', c_double * 3),
    ('posMin', c_double * 18),
    ('posMax', c_double * 18),
    ('velMin', c_double * 18),
    ('velMax', c_double * 18),
    ('graspingGraspApertureTarget', c_double),
    ('graspingOpenApertureTarget', c_double),
    ('graspingIntermediateApertureTarget', c_double),
    ('maxApproachSwingAngle', c_double),
    ('graspTooCloseIfClosedRadius', c_double),
]
class ROBOT_CONTROL_CONFIG(Structure):
    pass
ROBOT_CONTROL_CONFIG._fields_ = [
    ('target', c_double * 18),
    ('coriMatrix', c_double * 9),
    ('trialEvent', TRIAL_EVENT),
    ('autoControlParams', AUTOMAGIC_CONTROLLER_PARAMS),
    ('autoPosControlFraction', c_double * 18),
    ('autoVelControlFraction', c_double * 18),
    ('orthPosImpedance', c_double * 18),
    ('orthVelImpedance', c_double * 18),
    ('extrinsicVelControlled', c_int * 18),
    ('extrinsicPosControlled', c_int * 18),
    ('importantDOF', c_int * 18),
]
class ROBOT_ACTUAL_STATE(Structure):
    pass
ROBOT_ACTUAL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('RTMA_received_time', c_double),
    ('Jpos', c_double * 7),
    ('Jvel', c_double * 7),
    ('Cpos', c_double * 3),
    ('Cori', c_double * 3),
    ('Cforce', c_double * 3),
    ('Ctrq', c_double * 3),
    ('Cvel', c_double * 3),
    ('CangVel', c_double * 3),
    ('Hpos', c_double * 4),
    ('Hstrain', c_double * 4),
    ('Jtrq', c_double * 3),
]
class ROBOT_CONTROL_SPACE_ACTUAL_STATE(Structure):
    pass
ROBOT_CONTROL_SPACE_ACTUAL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('RTMA_received_time', c_double),
    ('pos', c_double * 18),
    ('vel', c_double * 18),
    ('CoriMatrix', c_double * 9),
]
class PLANNER_STATE(Structure):
    pass
PLANNER_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('targetPos', c_double * 18),
]
class JVEL_COMMAND(Structure):
    pass
JVEL_COMMAND._fields_ = [
    ('cmd', c_double * 11),
]
class OVERRIDE_COMMAND(Structure):
    pass
OVERRIDE_COMMAND._fields_ = [
    ('DOFCommand', c_double * 18),
    ('JposCommand', c_double * 7),
    ('controllerId', c_int),
    ('j0nks', c_int),
]
class ROBOT_CONTROL_STATE(Structure):
    pass
ROBOT_CONTROL_STATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('autoControl', PLANNER_STATE),
    ('overrideCommand', OVERRIDE_COMMAND),
    ('RTMA_received_time', c_double),
    ('actualControlPos', c_double * 18),
    ('actualControlVel', c_double * 18),
    ('autoControlPos', c_double * 18),
    ('autoControlVel', c_double * 18),
    ('autoPosControlFraction', c_double * 18),
    ('autoVelControlFraction', c_double * 18),
    ('orthPosImpedance', c_double * 18),
    ('orthVelImpedance', c_double * 18),
    ('intrinsicVelControlled', c_int * 18),
    ('intrinsicPosControlled', c_int * 18),
    ('extrinsicVelControlled', c_int * 18),
    ('extrinsicPosControlled', c_int * 18),
    ('overrideDimensions', c_int * 18),
    ('actualCommandComposition', c_uint),
    ('blank', c_int),
]
MDF_INPUT_DOF_DATA = INPUT_DOF_DATA
MDF_OPERATOR_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_FIXTURED_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_SHADOW_COMPOSITE_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
MDF_FIXTURED_COMPOSITE_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class MDF_PROBOT_FEEDBACK(Structure):
    pass
MDF_PROBOT_FEEDBACK._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('tool_pos', c_double * 7),
    ('wrist_pos', c_double * 7),
]
class MDF_GROBOT_COMMAND(Structure):
    pass
MDF_GROBOT_COMMAND._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('command', c_double * 18),
    ('mode', c_int),
    ('reserved', c_int),
]
class MDF_GROBOT_BYPASS(Structure):
    pass
MDF_GROBOT_BYPASS._fields_ = [
    ('command', c_double * 18),
    ('mode', c_int),
    ('reserved', c_int),
]
class MDF_OPTO_CNTRL_CMD(Structure):
    pass
MDF_OPTO_CNTRL_CMD._fields_ = [
    ('Cmd', c_double * 6),
]
class MDF_OPTO_POS_CMD(Structure):
    pass
MDF_OPTO_POS_CMD._fields_ = [
    ('Cmd', c_double * 4),
]
class MDF_KIN_POS_CMD(Structure):
    pass
MDF_KIN_POS_CMD._fields_ = [
    ('Cmd', c_double * 4),
]
class MDF_KINECT_SKELETON(Structure):
    pass
MDF_KINECT_SKELETON._fields_ = [
    ('x', c_double * 20),
    ('y', c_double * 20),
    ('z', c_double * 20),
    ('w', c_double * 20),
    ('Which', c_int),
    ('Reserved', c_int),
]
class MDF_GROBOT_SEGMENT_PERCEPTS(Structure):
    pass
MDF_GROBOT_SEGMENT_PERCEPTS._fields_ = [
    ('ind_force', c_double * 3),
    ('mid_force', c_double * 3),
    ('rng_force', c_double * 3),
    ('lit_force', c_double * 3),
    ('thb_force', c_double * 3),
    ('ind_accel', c_double * 3),
    ('mid_accel', c_double * 3),
    ('rng_accel', c_double * 3),
    ('lit_accel', c_double * 3),
    ('thb_accel', c_double * 3),
]
class MDF_CERESTIM_CONFIG_MODULE(Structure):
    pass
MDF_CERESTIM_CONFIG_MODULE._fields_ = [
    ('configID', c_int),
    ('afcf', c_int),
    ('pulses', c_double),
    ('amp1', c_double),
    ('amp2', c_double),
    ('width1', c_double),
    ('width2', c_double),
    ('frequency', c_double),
    ('interphase', c_double),
]
class MDF_CERESTIM_CONFIG_CHAN(Structure):
    pass
MDF_CERESTIM_CONFIG_CHAN._fields_ = [
    ('stop', c_int),
    ('group_stimulus', c_int),
    ('group_numChans', c_int),
    ('group_channel', c_int * 16),
    ('group_pattern', c_int * 16),
    ('manual_stimulus', c_int),
    ('manual_channel', c_int),
    ('manual_pattern', c_int),
]
class MDF_GROBOT_RAW_FEEDBACK(Structure):
    pass
MDF_GROBOT_RAW_FEEDBACK._fields_ = [
    ('j_ang', c_double * 28),
    ('j_vel', c_double * 28),
    ('j_trq', c_double * 28),
]
class MDF_GROBOT_FEEDBACK(Structure):
    pass
MDF_GROBOT_FEEDBACK._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('position', c_double * 18),
    ('velocity', c_double * 18),
    ('force', c_double * 18),
    ('cori_matrix', c_double * 9),
]
class MDF_SESSION_CONFIG(Structure):
    pass
MDF_SESSION_CONFIG._fields_ = [
    ('data_dir', c_char * 128),
]
class MDF_GLOVE_DATA(Structure):
    pass
MDF_GLOVE_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 30),
]
class MDF_MICROSTRAIN_DATA(Structure):
    pass
MDF_MICROSTRAIN_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 30),
]
class MDF_EM_OVERRIDE_CONFIG(Structure):
    pass
MDF_EM_OVERRIDE_CONFIG._fields_ = [
    ('chosen_channel_mask', c_int * 1728),
]
class MDF_CHANGE_TOOL(Structure):
    pass
MDF_CHANGE_TOOL._fields_ = [
    ('next_tool_id', c_int),
    ('reserved', c_int),
]
class MDF_CHANGE_TOOL_COMPLETE(Structure):
    pass
MDF_CHANGE_TOOL_COMPLETE._fields_ = [
    ('tool_id', c_int),
    ('reserved', c_int),
]
class MDF_TASK_STATE_CONFIG(Structure):
    pass
MDF_TASK_STATE_CONFIG._fields_ = [
    ('id', c_int),
    ('rep_num', c_int),
    ('use_for_calib', c_int),
    ('target_combo_index', c_int),
    ('timed_out_conseq', c_int),
    ('reach_offset', c_int),
    ('relax_arm', c_int),
    ('idle_gateable', c_int),
    ('force_gateable', c_int),
    ('direction', c_int),
    ('idle_timeout', c_double),
    ('ts_time', c_double),
    ('target', c_double * 33),
    ('coriMatrix', c_double * 9),
    ('idle_target', c_double * 18),
    ('trans_threshold', c_double),
    ('ori_threshold', c_double),
    ('trans_threshold_f', c_double),
    ('ori_threshold_f', c_double),
    ('sep_threshold', c_double * 12),
    ('sep_threshold_f', c_double * 12),
    ('sep_threshold_judging_polarity', c_int * 12),
    ('sep_threshold_f_judging_polarity', c_int * 12),
    ('sep_threshold_judging_outcome', c_int * 12),
    ('trans_threshold_judging_polarity', c_int),
    ('ori_threshold_judging_polarity', c_int),
    ('trans_threshold_f_judging_polarity', c_int),
    ('ori_threshold_f_judging_polarity', c_int),
    ('handle_judging_polarity', c_int),
    ('handle_judging_outcome', c_int),
    ('timeout', c_double),
    ('tags', c_char * 64),
    ('fdbk_display_color', c_char * 64),
    ('background_color', c_char * 64),
]
class MDF_JUDGE_VERDICT(Structure):
    pass
MDF_JUDGE_VERDICT._fields_ = [
    ('id', c_int),
    ('reserved', c_int),
    ('reason', c_char * 64),
]
class MDF_END_TASK_STATE(Structure):
    pass
MDF_END_TASK_STATE._fields_ = [
    ('id', c_int),
    ('outcome', c_int),
    ('reason', c_char * 64),
]
class MDF_RAW_SAMPLE_RESPONSE(Structure):
    pass
MDF_RAW_SAMPLE_RESPONSE._fields_ = [
    ('source_index', c_int),
    ('reserved', c_int),
    ('source_timestamp', c_double),
]
class MDF_CODE_VERSION(Structure):
    pass
MDF_CODE_VERSION._fields_ = [
    ('module_name', c_char * 64),
    ('version', c_char * 64),
]
class MDF_SPM_FIRINGRATE(Structure):
    pass
MDF_SPM_FIRINGRATE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('rates', c_double * 1728),
]
MDF_EM_DECODER_CONFIGURATION = c_ubyte * 0
class MDF_LOAD_DECODER_CONFIG(Structure):
    pass
MDF_LOAD_DECODER_CONFIG._fields_ = [
    ('full_path', c_char * 128),
]
class MDF_XM_START_SESSION(Structure):
    pass
MDF_XM_START_SESSION._fields_ = [
    ('load_calibration', c_int),
    ('calib_session_id', c_int),
    ('num_reps', c_int),
    ('reserved', c_int),
    ('subject_name', c_char * 64),
]
class MDF_PING(Structure):
    pass
MDF_PING._fields_ = [
    ('module_name', c_char * 64),
]
class MDF_PING_ACK(Structure):
    pass
MDF_PING_ACK._fields_ = [
    ('module_name', c_char * 64),
]
class MDF_DEBUG_VECTOR(Structure):
    pass
MDF_DEBUG_VECTOR._fields_ = [
    ('data', c_double * 32),
]
class MDF_APP_START(Structure):
    pass
MDF_APP_START._fields_ = [
    ('config', c_char * 64),
]
class MDF_MODULE_START(Structure):
    pass
MDF_MODULE_START._fields_ = [
    ('module', c_char * 64),
]
class MDF_SPM_SPIKE_SNIPPET(Structure):
    pass
MDF_SPM_SPIKE_SNIPPET._fields_ = [
    ('time', c_double),
    ('chan', c_int),
    ('unit', c_int),
    ('box_id', c_int),
    ('length', c_int),
    ('snippet', c_short * 48),
]
class MDF_SPM_SPIKE_TIMES(Structure):
    pass
MDF_SPM_SPIKE_TIMES._fields_ = [
    ('time', c_double * 256),
    ('chan', c_short * 256),
    ('unit', c_char * 256),
    ('box_id', c_char * 256),
]
class MDF_IDLY_LABELLING(Structure):
    pass
MDF_IDLY_LABELLING._fields_ = [
    ('state', c_int),
    ('reserved', c_int),
]
class MDF_EM_DRIFT_CORRECTION(Structure):
    pass
MDF_EM_DRIFT_CORRECTION._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('drift_correction', c_double * 18),
]
class MDF_PLAY_SOUND(Structure):
    pass
MDF_PLAY_SOUND._fields_ = [
    ('id', c_int),
    ('reserved', c_int),
]
class MDF_ARTIFACT_REJECTED(Structure):
    pass
MDF_ARTIFACT_REJECTED._fields_ = [
    ('time', c_double),
]
class MDF_IDLE(Structure):
    pass
MDF_IDLE._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('score', c_double),
    ('gain', c_double),
    ('idle', c_int),
    ('reserved', c_int),
]
MDF_WAM_FEEDBACK = ROBOT_CONTROL_SPACE_ACTUAL_STATE
MDF_WAM_HAND_FEEDBACK = ROBOT_CONTROL_SPACE_ACTUAL_STATE
class MDF_PLANNER_CONTROL_CONFIG(Structure):
    pass
MDF_PLANNER_CONTROL_CONFIG._fields_ = [
    ('target', c_double * 18),
    ('coriMatrix', c_double * 9),
]
class MDF_ROBOT_JOINT_COMMAND(Structure):
    pass
MDF_ROBOT_JOINT_COMMAND._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('pos', c_double * 8),
    ('controlledDims', c_int * 8),
    ('overrideDims', c_int * 8),
]
MDF_IDLEGATED_MOVEMENT_COMMAND = MOVEMENT_COMMAND_DATA
class MDF_OUTPUT_DOF_DATA(Structure):
    pass
MDF_OUTPUT_DOF_DATA._fields_ = [
    ('dof_vals', c_double * 21),
    ('tag', c_char * 64),
]
class MDF_FORCE_APPLIED(Structure):
    pass
MDF_FORCE_APPLIED._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('applied', c_int),
    ('reserved', c_int),
]
class MDF_RAW_FORCE_SENSOR_DATA(Structure):
    pass
MDF_RAW_FORCE_SENSOR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('rdt_sequence', c_uint),
    ('ft_sequence', c_uint),
    ('status', c_uint),
    ('reserved', c_uint),
    ('data', c_double * 6),
]
class MDF_FORCE_SENSOR_DATA(Structure):
    pass
MDF_FORCE_SENSOR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('rdt_sequence', c_uint),
    ('ft_sequence', c_uint),
    ('status', c_uint),
    ('reserved', c_uint),
    ('data', c_double * 6),
    ('offset', c_double * 6),
]
class MDF_SYNC_PULSE(Structure):
    pass
MDF_SYNC_PULSE._fields_ = [
    ('SerialNo', c_uint),
    ('reserved', c_uint),
]
class MDF_MUSCLE(Structure):
    pass
MDF_MUSCLE._fields_ = [
    ('channelNum', c_uint),
    ('reserved', c_uint),
    ('name', c_char * 64),
]
class MDF_COMBO_WAIT(Structure):
    pass
MDF_COMBO_WAIT._fields_ = [
    ('duration', c_uint),
    ('reserved', c_uint),
]
class MDF_FSR(Structure):
    pass
MDF_FSR._fields_ = [
    ('channelNum', c_uint),
    ('reserved', c_uint),
    ('name', c_char * 64),
]
class MDF_FSR_DATA(Structure):
    pass
MDF_FSR_DATA._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('data', c_double * 8),
]
class MDF_FORCE_FEEDBACK(Structure):
    pass
MDF_FORCE_FEEDBACK._fields_ = [
    ('x', c_double),
    ('y', c_double),
    ('z', c_double),
]
class MDF_POSITION_FEEDBACK(Structure):
    pass
MDF_POSITION_FEEDBACK._fields_ = [
    ('x', c_double),
    ('y', c_double),
    ('z', c_double),
]
class MDF_TRIAL_INPUT(Structure):
    pass
MDF_TRIAL_INPUT._fields_ = [
    ('XorYorZ', c_int),
    ('UpOrDown', c_int),
    ('forceThreshold', c_double),
    ('targetDistance', c_double),
]
class MDF_RT_POSITION_FEEDBACK(Structure):
    pass
MDF_RT_POSITION_FEEDBACK._fields_ = [
    ('distanceFromCenter', c_double),
]
class MDF_RAW_SPIKECOUNT(Structure):
    pass
MDF_RAW_SPIKECOUNT._fields_ = [
    ('source_index', c_int),
    ('reserved', c_int),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('counts', c_ubyte * 576),
]
SPIKE_COUNT_DATA_TYPE = c_ubyte
class MDF_SPM_SPIKECOUNT(Structure):
    pass
MDF_SPM_SPIKECOUNT._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
    ('count_interval', c_double),
    ('counts', SPIKE_COUNT_DATA_TYPE * 1728),
]
class MDF_SAMPLE_GENERATED(Structure):
    pass
MDF_SAMPLE_GENERATED._fields_ = [
    ('sample_header', SAMPLE_HEADER),
    ('source_timestamp', c_double),
]
class MDF_SAMPLE_RESPONSE(Structure):
    pass
MDF_SAMPLE_RESPONSE._fields_ = [
    ('sample_response_timestamp', c_double),
    ('sample_alignment_timestamp', c_double),
    ('sample_response_count', c_uint),
    ('sample_alignment_count', c_uint),
]
MID_GATING_JUDGE = 43 # Variable c_int '43'
MAX_SPIKE_TIMES_PER_PACKET = 256 # Variable c_int '256'
KINECT_JOINTS = 20 # Variable c_int '20'
SF_IRREGULAR = 4 # Variable c_int '4'
MT_ENABLE_DATA_COLLECTION = 2380 # Variable c_int '2380'
MT_JUDGE_VERDICT = 1960 # Variable c_int '1960'
MID_ARDUINO_IO_WRITE_TESTER = 41 # Variable c_int '41'
MT_KIN_POS_CMD = 1713 # Variable c_int '1713'
MT_SYNCH_DONE = 1802 # Variable c_int '1802'
MAX_OPTO_CTRL_JOINTS = 6 # Variable c_int '6'
MT_IDLY_LABELLING = 2190 # Variable c_int '2190'
MT_APP_START_COMPLETE = 2121 # Variable c_int '2121'
MT_APP_START = 2120 # Variable c_int '2120'
MID_SIMPLE_ARBITRATOR = 13 # Variable c_int '13'
MT_XM_END_OF_SESSION = 2180 # Variable c_int '2180'
MID_DIGITAL_IO = 17 # Variable c_int '17'
MT_EM_DECODER_CONFIGURATION = 2090 # Variable c_int '2090'
MID_EM_OVERRIDE_CONFIG = 31 # Variable c_int '31'
MID_NETBOX_MODULE = 25 # Variable c_int '25'
MT_FIXTURED_MOVEMENT_COMMAND = 1910 # Variable c_int '1910'
MAX_CYBER_GLOVE_DIMS = 30 # Variable c_int '30'
MT_SPM_FIRINGRATE = 2100 # Variable c_int '2100'
MT_CHANGE_TOOL_FAILED = 1943 # Variable c_int '1943'
SF_FRACTINT = 8 # Variable c_int '8'
MT_CERESTIM_CONFIG_MODULE = 1889 # Variable c_int '1889'
MT_DEBUG_VECTOR = 2160 # Variable c_int '2160'
MT_ROBOT_JOINT_COMMAND = 2270 # Variable c_int '2270'
MT_OUTPUT_DOF_DATA = 2310 # Variable c_int '2310'
MT_RAW_FORCE_SENSOR_DATA = 2340 # Variable c_int '2340'
MID_GROBOT_FEEDBACK = 18 # Variable c_int '18'
MT_IDLEGATED_MOVEMENT_COMMAND = 2280 # Variable c_int '2280'
MT_CHANGE_TOOL_COMPLETE = 1942 # Variable c_int '1942'
MID_OPTO_MPL_CTRL = 32 # Variable c_int '32'
MT_SYNCH_START = 1801 # Variable c_int '1801'
MT_ENABLE_SYNC_PULSE = 2400 # Variable c_int '2400'
MT_END_TASK_STATE = 1970 # Variable c_int '1970'
MID_SSH_CONTROLLER = 35 # Variable c_int '35'
MT_TASK_STATE_CONFIG = 1950 # Variable c_int '1950'
MAX_OPTO_POS = 4 # Variable c_int '4'
MT_SPM_SPIKECOUNT = 1751 # Variable c_int '1751'
MT_SESSION_CONFIG = 1710 # Variable c_int '1710'
MID_COLOR_CUE = 46 # Variable c_int '46'
MT_EM_DRIFT_CORRECTION = 2210 # Variable c_int '2210'
MID_FSR_READ = 45 # Variable c_int '45'
MT_EXIT_ACK = 2130 # Variable c_int '2130'
MID_CERESTIM_CONFIG = 57 # Variable c_int '57'
MAX_UNITS_PER_CHAN = 6 # Variable c_int '6'
MID_GROBOT_SEGMENT_PERCEPTS = 56 # Variable c_int '56'
MT_CHANGE_TOOL_INVALID = 1941 # Variable c_int '1941'
MT_FORCE_SENSOR_DATA = 2330 # Variable c_int '2330'
MPL_AT_ARM_EPV_FING_JV = 0 # Variable c_int '0'
MT_FIXTURED_COMPOSITE_MOVEMENT_COMMAND = 1920 # Variable c_int '1920'
MT_EM_OVERRIDE_CONFIG = 1708 # Variable c_int '1708'
MPL_AT_ARM_EPV_FING_JP = 1 # Variable c_int '1'
MT_KINECT_SKELETON = 1711 # Variable c_int '1711'
MID_ARDUINO_SYNC = 44 # Variable c_int '44'
MT_OPTO_CNTRL_CMD = 1709 # Variable c_int '1709'
SF_UNFREEZE = 2 # Variable c_int '2'
NUM_FINGER_DIMS = 10 # Variable c_int '10'
MID_VIRTUAL_FIXTURING = 28 # Variable c_int '28'
MT_RAW_SAMPLE_RESPONSE = 1980 # Variable c_int '1980'
MT_POSITION_FEEDBACK = 2423 # Variable c_int '2423'
MT_CERESTIM_CONFIG_CHAN = 1890 # Variable c_int '1890'
MID_FSR_EMG = 47 # Variable c_int '47'
MAX_TOTAL_SPIKE_CHANS = 1728 # Variable c_int '1728'
MT_OPERATOR_MOVEMENT_COMMAND = 1900 # Variable c_int '1900'
MAX_PROBOT_FEEDBACK_DIMS = 7 # Variable c_int '7'
MT_SPM_SPIKE_TIMES = 2171 # Variable c_int '2171'
MPL_AT_ALL_JV = 3 # Variable c_int '3'
MPL_AT_ALL_JP = 4 # Variable c_int '4'
MT_FSR = 2420 # Variable c_int '2420'
MT_IDLE = 2250 # Variable c_int '2250'
MT_PLAY_SOUND = 2230 # Variable c_int '2230'
MT_IDLY_RESET_LABELLING = 2200 # Variable c_int '2200'
MT_MODULE_START = 2125 # Variable c_int '2125'
MT_SAMPLE_GENERATED = 1752 # Variable c_int '1752'
MAX_SEPARATE_DIMS = 12 # Variable c_int '12'
MT_FORCE_APPLIED = 2320 # Variable c_int '2320'
MT_FORCE_FEEDBACK = 2422 # Variable c_int '2422'
MT_START_PAD_RELEASED = 2300 # Variable c_int '2300'
MAX_EM_CHANNELS = 1728 # Variable c_int '1728'
MAX_GROBOT_COMMAND_DIMS = 18 # Variable c_int '18'
MT_GROBOT_RAW_FEEDBACK = 1701 # Variable c_int '1701'
MID_TASK_JUDGE = 16 # Variable c_int '16'
MT_GROBOT_FEEDBACK = 1702 # Variable c_int '1702'
MT_FSR_DATA = 2421 # Variable c_int '2421'
MAX_TOTAL_SPIKE_CHANS_PER_SOURCE = 576 # Variable c_int '576'
MAX_SPIKE_CHANS_PER_SOURCE = 96 # Variable c_int '96'
MID_SURFACE_EMG = 36 # Variable c_int '36'
MT_TRIAL_DATA_SAVED = 2080 # Variable c_int '2080'
MAX_FINGER_DIMS = 10 # Variable c_int '10'
MT_WAM_HAND_FEEDBACK = 2242 # Variable c_int '2242'
MSEC_PER_RAW_SAMPLE = 10 # Variable c_int '10'
MID_DENSO_GATE = 15 # Variable c_int '15'
MT_GLOVE_DATA = 1706 # Variable c_int '1706'
MAX_DOFS = 21 # Variable c_int '21'
SAMPLE_DT = 0.03 # Variable c_double '2.99999999999999988897769753748434595763683319091796875e-2'
MID_MESSAGE_WATCHER = 88 # Variable c_int '88'
MT_WAM_FEEDBACK = 2240 # Variable c_int '2240'
MT_PING = 2140 # Variable c_int '2140'
MT_PING_ACK = 2141 # Variable c_int '2141'
MID_GROBOT_RAW_FEEDBACK = 19 # Variable c_int '19'
MID_INPUT_TRANSFORM = 14 # Variable c_int '14'
MID_HAND_VIZ = 30 # Variable c_int '30'
MT_PROBOT_FEEDBACK = 1930 # Variable c_int '1930'
MT_RESET_SAMPLE_ALIGNMENT = 1754 # Variable c_int '1754'
MT_GROBOT_END_BYPASS = 1707 # Variable c_int '1707'
MID_OBSTACLE_COURSE = 33 # Variable c_int '33'
MT_MODULE_START_COMPLETE = 2126 # Variable c_int '2126'
MT_SHADOW_COMPOSITE_MOVEMENT_COMMAND = 1919 # Variable c_int '1919'
MT_SYNCH_NOW = 1800 # Variable c_int '1800'
MID_ARDUINO_MODULE = 42 # Variable c_int '42'
MAX_JOINT_DIMS = 8 # Variable c_int '8'
MT_MICROSTRAIN_DATA = 1705 # Variable c_int '1705'
TAG_LENGTH = 64 # Variable c_int '64'
MID_FEEDBACK_TRANSFORM = 11 # Variable c_int '11'
MT_INPUT_DOF_DATA = 1850 # Variable c_int '1850'
MAX_HAND_DIMS = 25 # Variable c_int '25'
MT_DATA_COLLECTED = 2370 # Variable c_int '2370'
MT_ARTIFACT_REJECTED = 2220 # Variable c_int '2220'
MID_CALIBRATION = 37 # Variable c_int '37'
MT_PLANNER_CONTROL_CONFIG = 2260 # Variable c_int '2260'
SF_ALIGNMENT = 1 # Variable c_int '1'
NUM_D_COLS = 16 # Variable c_int '16'
MT_START_PAD_PRESSED = 2290 # Variable c_int '2290'
MID_ARDUINO_IO = 39 # Variable c_int '39'
MT_RT_POSITION_FEEDBACK = 2425 # Variable c_int '2425'
MT_COMBO_WAIT = 2410 # Variable c_int '2410'
MPL_AT_ARM_JV_FING_JP = 2 # Variable c_int '2'
MT_STOP_DATA_COLLECT = 2365 # Variable c_int '2365'
MT_START_DATA_COLLECT = 2360 # Variable c_int '2360'
MPL_AT_ARM_EPP_FING_JP = 5 # Variable c_int '5'
MAX_KIN_POS = 4 # Variable c_int '4'
MT_COMMANDSPACE_FEEDBACK = 1703 # Variable c_int '1703'
MAX_DATA_DIR_LEN = 128 # Variable c_int '128'
MT_MUSCLE = 2390 # Variable c_int '2390'
MT_DATA_COLLECT_STARTED = 2362 # Variable c_int '2362'
MAX_D_COLS = 32 # Variable c_int '32'
MT_CODE_VERSION = 1990 # Variable c_int '1990'
MT_SPM_SPIKE_SNIPPET = 2170 # Variable c_int '2170'
MT_GROBOT_SEGMENT_PERCEPTS = 1888 # Variable c_int '1888'
MID_CERESTIM_CONTROL = 58 # Variable c_int '58'
MAX_PERCEPT_DIMS = 15 # Variable c_int '15'
MID_ARDUINO_IO_READ_TESTER = 40 # Variable c_int '40'
MAX_GROBOT_JOINTS = 28 # Variable c_int '28'
MT_XM_START_SESSION = 2150 # Variable c_int '2150'
MT_GROBOT_COMMAND = 1700 # Variable c_int '1700'
MT_IDLE_DETECTION_ENDED = 2251 # Variable c_int '2251'
MID_OUTPUT_TRANSFORM = 12 # Variable c_int '12'
MT_CHANGE_TOOL = 1940 # Variable c_int '1940'
MID_SKELETON_CTRL = 34 # Variable c_int '34'
RAW_COUNTS_PER_SAMPLE = 3 # Variable c_int '3'
MAX_GROBOT_FEEDBACK_DIMS = 18 # Variable c_int '18'
MT_RAW_SPIKECOUNT = 1750 # Variable c_int '1750'
MID_SAMPLE_GENERATOR = 29 # Variable c_int '29'
MID_CANNED_MOVEMENT = 27 # Variable c_int '27'
MAX_SPIKE_SOURCES = 3 # Variable c_int '3'
MID_COMMAND_SPACE_FEEDBACK_GUI = 89 # Variable c_int '89'
MT_GROBOT_BYPASS = 1704 # Variable c_int '1704'
MT_TRIAL_INPUT = 2424 # Variable c_int '2424'
MT_OPTO_POS_CMD = 1712 # Variable c_int '1712'
MT_SAMPLE_RESPONSE = 1753 # Variable c_int '1753'
MT_LOAD_DECODER_CONFIG = 2110 # Variable c_int '2110'
MT_SYNC_PULSE = 2350 # Variable c_int '2350'
__all__ = ['MID_GATING_JUDGE', 'MDF_GROBOT_COMMAND', 'SF_IRREGULAR',
           'MT_ENABLE_DATA_COLLECTION', 'MT_JUDGE_VERDICT',
           'MID_ARDUINO_IO_WRITE_TESTER', 'MT_KIN_POS_CMD',
           'MDF_PING_ACK', 'MAX_OPTO_CTRL_JOINTS',
           'MDF_GROBOT_RAW_FEEDBACK', 'SAMPLE_HEADER',
           'MT_APP_START_COMPLETE', 'MT_APP_START',
           'MID_SIMPLE_ARBITRATOR', 'MT_XM_END_OF_SESSION',
           'MDF_SHADOW_COMPOSITE_MOVEMENT_COMMAND', 'INPUT_DOF_DATA',
           'check_flag_bits', 'MDF_OUTPUT_DOF_DATA', 'MID_DIGITAL_IO',
           'MT_EM_DECODER_CONFIGURATION', 'MT_FSR_DATA',
           'MID_NETBOX_MODULE', 'MT_FIXTURED_MOVEMENT_COMMAND',
           'MAX_CYBER_GLOVE_DIMS', 'MT_SPM_FIRINGRATE',
           'ROBOT_CONTROL_STATE', 'MDF_SPM_SPIKE_SNIPPET',
           'MDF_EM_OVERRIDE_CONFIG', 'MT_CERESTIM_CONFIG_MODULE',
           'MT_DEBUG_VECTOR', 'MT_ROBOT_JOINT_COMMAND',
           'MT_OUTPUT_DOF_DATA', 'MT_MICROSTRAIN_DATA',
           'ROBOT_ACTUAL_STATE', 'MAX_EM_DIMS', 'MID_GROBOT_FEEDBACK',
           'MT_IDLEGATED_MOVEMENT_COMMAND', 'TAG_LENGTH',
           'MT_CHANGE_TOOL_COMPLETE', 'MID_OPTO_MPL_CTRL',
           'MT_SYNCH_START', 'MT_SYNCH_DONE', 'MT_ENABLE_SYNC_PULSE',
           'MT_SPM_SPIKE_SNIPPET', 'MT_END_TASK_STATE', 'MDF_PING',
           'MT_GROBOT_SEGMENT_PERCEPTS', 'MT_TASK_STATE_CONFIG',
           'MDF_SAMPLE_GENERATED', 'MAX_OPTO_POS',
           'MT_SPM_SPIKECOUNT', 'MT_SESSION_CONFIG', 'MID_FSR_EMG',
           'MT_EXIT_ACK', 'MDF_KIN_POS_CMD', 'MDF_MODULE_START',
           'MT_INPUT_DOF_DATA', 'MDF_CERESTIM_CONFIG_CHAN',
           'MT_CHANGE_TOOL_INVALID', 'MDF_GROBOT_SEGMENT_PERCEPTS',
           'MT_OPTO_POS_CMD', 'OVERRIDE_COMMAND',
           'MT_FORCE_SENSOR_DATA', 'MPL_AT_ARM_EPV_FING_JV',
           'MT_FIXTURED_COMPOSITE_MOVEMENT_COMMAND',
           'MID_VIRTUAL_FIXTURING', 'SPIKE_COUNT_DATA_TYPE',
           'MDF_OPTO_POS_CMD', 'SAMPLE_DT', 'MT_KINECT_SKELETON',
           'MAX_SPIKE_TIMES_PER_PACKET', 'MID_SSH_CONTROLLER',
           'MT_IDLY_LABELLING', 'ROBOT_CONTROL_CONFIG',
           'MDF_GROBOT_FEEDBACK', 'NUM_FINGER_DIMS', 'MT_SYNCH_NOW',
           'MDF_RT_POSITION_FEEDBACK', 'MT_RAW_SAMPLE_RESPONSE',
           'MDF_IDLE', 'MT_POSITION_FEEDBACK',
           'MT_CERESTIM_CONFIG_CHAN', 'MT_EM_DRIFT_CORRECTION',
           'MAX_TOTAL_SPIKE_CHANS', 'MT_GLOVE_DATA',
           'MDF_TRIAL_INPUT', 'MDF_WAM_FEEDBACK',
           'MT_OPERATOR_MOVEMENT_COMMAND',
           'MDF_EM_DECODER_CONFIGURATION', 'MID_FSR_READ',
           'MT_SPM_SPIKE_TIMES', 'MPL_AT_ALL_JV', 'MPL_AT_ALL_JP',
           'MDF_PLANNER_CONTROL_CONFIG', 'MDF_INPUT_DOF_DATA',
           'TRIAL_EVENT', 'MT_IDLE', 'MDF_EM_DRIFT_CORRECTION',
           'MT_PLAY_SOUND', 'MT_FSR', 'MT_IDLY_RESET_LABELLING',
           'MT_XM_START_SESSION', 'MDF_DEBUG_VECTOR',
           'MDF_CHANGE_TOOL', 'MT_SAMPLE_GENERATED',
           'MAX_PROBOT_FEEDBACK_DIMS',
           'MID_COMMAND_SPACE_FEEDBACK_GUI', 'MT_FORCE_APPLIED',
           'MDF_XM_START_SESSION', 'MAX_CONTROL_DIMS',
           'MDF_CODE_VERSION', 'MAX_KIN_POS', 'MT_FORCE_FEEDBACK',
           'MT_WAM_HAND_FEEDBACK', 'PLAN_PROCESSOR_STATE',
           'MDF_COMBO_WAIT', 'MT_START_PAD_RELEASED',
           'MDF_RAW_FORCE_SENSOR_DATA', 'MAX_SEPARATE_DIMS',
           'MAX_EM_CHANNELS', 'MAX_GROBOT_COMMAND_DIMS',
           'MDF_CERESTIM_CONFIG_MODULE', 'MID_TASK_JUDGE',
           'MT_GROBOT_FEEDBACK', 'MID_EM_OVERRIDE_CONFIG',
           'MID_OUTPUT_TRANSFORM', 'MAX_TOTAL_SPIKE_CHANS_PER_SOURCE',
           'MAX_SPIKE_CHANS_PER_SOURCE', 'MID_SURFACE_EMG',
           'NUM_D_COLS', 'MT_TRIAL_DATA_SAVED', 'MDF_END_TASK_STATE',
           'MAX_FINGER_DIMS', 'MDF_FSR', 'SF_UNFREEZE',
           'MSEC_PER_RAW_SAMPLE', 'MID_DENSO_GATE',
           'MDF_IDLEGATED_MOVEMENT_COMMAND', 'MAX_DOFS',
           'MT_RAW_FORCE_SENSOR_DATA', 'MID_MESSAGE_WATCHER',
           'MT_WAM_FEEDBACK', 'MT_PING', 'MT_PING_ACK',
           'MDF_ARTIFACT_REJECTED', 'MDF_FORCE_FEEDBACK',
           'MAX_SPIKE_SOURCES', 'PLANNER_STATE', 'MID_ARDUINO_SYNC',
           'MDF_OPTO_CNTRL_CMD', 'RAW_COUNTS_PER_SAMPLE',
           'MDF_FIXTURED_COMPOSITE_MOVEMENT_COMMAND',
           'MDF_IDLY_LABELLING', 'MDF_SPM_SPIKECOUNT',
           'MID_INPUT_TRANSFORM', 'MDF_SYNC_PULSE', 'MID_HAND_VIZ',
           'MT_PROBOT_FEEDBACK', 'MDF_SAMPLE_RESPONSE',
           'MT_GROBOT_END_BYPASS', 'MID_OBSTACLE_COURSE',
           'MDF_FSR_DATA', 'MDF_CHANGE_TOOL_COMPLETE',
           'MT_SHADOW_COMPOSITE_MOVEMENT_COMMAND', 'MDF_MUSCLE',
           'MDF_JUDGE_VERDICT', 'MT_MODULE_START',
           'MID_CANNED_MOVEMENT', 'MAX_JOINT_DIMS',
           'MAX_UNITS_PER_CHAN', 'MID_ARDUINO_MODULE',
           'MDF_FIXTURED_MOVEMENT_COMMAND', 'MID_FEEDBACK_TRANSFORM',
           'MDF_SESSION_CONFIG', 'MID_GROBOT_SEGMENT_PERCEPTS',
           'MID_CERESTIM_CONFIG', 'MAX_HAND_DIMS', 'MDF_PLAY_SOUND',
           'MT_DATA_COLLECTED', 'MT_ARTIFACT_REJECTED',
           'MDF_APP_START', 'MID_CALIBRATION',
           'MDF_POSITION_FEEDBACK', 'MID_GROBOT_RAW_FEEDBACK',
           'MT_PLANNER_CONTROL_CONFIG', 'MT_CHANGE_TOOL_FAILED',
           'MDF_FORCE_APPLIED', 'SF_ALIGNMENT', 'MID_COLOR_CUE',
           'MDF_WAM_HAND_FEEDBACK', 'MT_START_PAD_PRESSED',
           'MID_ARDUINO_IO', 'SF_FRACTINT', 'MT_RT_POSITION_FEEDBACK',
           'MT_COMBO_WAIT', 'MPL_AT_ARM_JV_FING_JP',
           'MT_COMMANDSPACE_FEEDBACK', 'MT_STOP_DATA_COLLECT',
           'MT_START_DATA_COLLECT', 'MPL_AT_ARM_EPP_FING_JP',
           'MDF_KINECT_SKELETON', 'MDF_ROBOT_JOINT_COMMAND',
           'MDF_OPERATOR_MOVEMENT_COMMAND',
           'AUTOMAGIC_CONTROLLER_PARAMS', 'MDF_RAW_SPIKECOUNT',
           'MAX_DATA_DIR_LEN', 'MDF_FORCE_SENSOR_DATA',
           'MDF_RAW_SAMPLE_RESPONSE', 'MDF_SPM_SPIKE_TIMES',
           'MT_MUSCLE', 'MDF_PROBOT_FEEDBACK',
           'MT_DATA_COLLECT_STARTED', 'MAX_D_COLS', 'MT_CODE_VERSION',
           'MOVEMENT_COMMAND_DATA', 'MT_GROBOT_RAW_FEEDBACK',
           'MID_CERESTIM_CONTROL', 'MAX_PERCEPT_DIMS',
           'MID_ARDUINO_IO_READ_TESTER',
           'ROBOT_CONTROL_SPACE_ACTUAL_STATE',
           'MDF_LOAD_DECODER_CONFIG', 'MDF_GROBOT_BYPASS',
           'MT_OPTO_CNTRL_CMD', 'MDF_MICROSTRAIN_DATA',
           'MT_GROBOT_COMMAND', 'MT_IDLE_DETECTION_ENDED',
           'MDF_TASK_STATE_CONFIG', 'MT_CHANGE_TOOL',
           'MT_RESET_SAMPLE_ALIGNMENT', 'MID_SKELETON_CTRL',
           'MT_EM_OVERRIDE_CONFIG', 'MAX_GROBOT_FEEDBACK_DIMS',
           'MT_RAW_SPIKECOUNT', 'MID_SAMPLE_GENERATOR',
           'MPL_AT_ARM_EPV_FING_JP', 'MDF_GLOVE_DATA',
           'KINECT_JOINTS', 'MAX_GROBOT_JOINTS', 'MT_GROBOT_BYPASS',
           'MT_TRIAL_INPUT', 'MDF_SPM_FIRINGRATE',
           'MT_MODULE_START_COMPLETE', 'JVEL_COMMAND',
           'MT_SAMPLE_RESPONSE', 'MT_LOAD_DECODER_CONFIG',
           'MT_SYNC_PULSE']
