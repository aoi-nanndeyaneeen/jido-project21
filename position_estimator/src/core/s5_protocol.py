"""
s5_protocol.py  -  ★ 自動生成。手で編集しないこと ★

  python protocol/gen_py_protocol.py

protocol/S5Cmd.h / S5Telem.h (機体と地上局が共有する無線プロトコル) から
Python 側で使う定数を生成したもの。値の意味は元ヘッダのコメントを読む。
XXX_NAME は 値 -> 表示名 の dict (C++ の xxxName() と同じ文字列)。
"""

# ======================================================================
#  namespace S5C  (S5Cmd.h)
# ======================================================================
S5C_VERSION = 9
MAGIC = 0x4b
IM920SL_MAX_PAYLOAD = 32
CHECKSUM_BYTES = 4

S5C_SC_MMPS = 1000.0
S5C_SC_CM = 100.0
S5C_SC_CDPS = 100.0
S5C_SC_MM = 1000.0
S5C_SC_CDEG = 100.0

# enum Req
REQ_IDLE = 0
REQ_HOLD = 1
REQ_TAKEOFF = 2
REQ_GUIDED = 3
REQ_LAND = 4
REQ_ABORT = 5
REQ_CIRCLE = 6
REQ_FIGURE8 = 7
REQ_CLIMB_TURN = 8

# enum Action
ACT_NONE = 0
ACT_PID_RESET = 1
ACT_IMU_CAL = 2
ACT_SELFTEST = 3

# enum CmdFlag
CF_ARMED_OK = 1 << 0
CF_POS_VALID = 1 << 1
CF_YAW_VALID = 1 << 2
CF_ALT_ABS = 1 << 3
CF_POS_CORR = 1 << 4
CF_POS_SHIFT = 1 << 5

REQ_NAME = {
    REQ_IDLE: 'IDLE',
    REQ_HOLD: 'HOLD',
    REQ_TAKEOFF: 'TAKEOFF',
    REQ_GUIDED: 'GUIDED',
    REQ_LAND: 'LAND',
    REQ_ABORT: 'ABORT',
    REQ_CIRCLE: 'CIRCLE',
    REQ_FIGURE8: 'FIGURE8',
    REQ_CLIMB_TURN: 'CLIMB',
}

ACTION_NAME = {
    ACT_NONE: '-',
    ACT_PID_RESET: 'PID_RESET',
    ACT_IMU_CAL: 'IMU_CAL',
    ACT_SELFTEST: 'SELFTEST',
}

# ======================================================================
#  namespace S5T  (S5Telem.h)
# ======================================================================
S5T_VERSION = 9
PACKET_BYTES = 28
TYPE_ALT = 0x41
TYPE_POS = 0x42
TYPE_ATT = 0x43
TYPE_DV = 0x44
TYPE_PARAM = 0x50

S5T_SC_CDEG = 100.0
S5T_SC_DDEG = 10.0
S5T_SC_MM = 1000.0
S5T_SC_CM = 100.0
S5T_SC_1E4 = 10000.0
S5T_SC_GAIN = 1000.0
S5T_SC_STICK = 100.0

# enum Flag
F_ARMED = 1 << 0
F_FLOW_OK = 1 << 1
F_RANGE_OK = 1 << 2
F_RANGE_VALID = 1 << 3
F_ALT_EN = 1 << 4
F_ALT_ACT = 1 << 5
F_POS_HOLD = 1 << 6
F_AIRBORNE = 1 << 7
F_DRY_RUN = 1 << 8
F_SAT = 1 << 9
F_TX_DROP = 1 << 10
F_GUIDED = 1 << 11
F_CMD_FRESH = 1 << 12
F_LANDED = 1 << 13
F_MANEUVER = 1 << 14
F_FRAME_OK = 1 << 15

# enum Mode
MODE_RATE = 0
MODE_ANGLE = 1
MODE_GUIDED = 2
MODE_POSHOLD = 3
MODE_ALTHOLD = 4

# enum AltState
ALT_OFF = 0
ALT_STANDBY = 1
ALT_NOHOVERTHR = 2
ALT_NORANGE = 3
ALT_HOLDING = 4
ALT_RANGELOST = 5

# enum ParamFlag
PF_ALT_HOLD_EN = 1
PF_DRY_RUN = 2
PF_SONAR = 4
PF_STICK_VZ = 8

# enum AckResult
ACK_OK = 0
ACK_REFUSED_ARMED = 1
ACK_CAL_REJECTED = 2

MODE_NAME = {
    MODE_RATE: 'RATE',
    MODE_ANGLE: 'ANGLE',
    MODE_GUIDED: 'GUIDED',
    MODE_POSHOLD: 'POSHOLD',
    MODE_ALTHOLD: 'ALTHOLD',
}

ALT_STATE_NAME = {
    ALT_OFF: 'OFF',
    ALT_STANDBY: 'STANDBY',
    ALT_NOHOVERTHR: 'NO_HOVER_THR',
    ALT_NORANGE: 'NO_RANGE',
    ALT_HOLDING: 'HOLDING',
    ALT_RANGELOST: 'RANGE_LOST',
}
