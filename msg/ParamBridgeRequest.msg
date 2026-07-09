# Request to the param_dds_bridge module to read/write a PX4 parameter by NAME
# (or by INDEX for backup). Companion publishes on /fmu/in/param_bridge_request.

uint64 timestamp        # time since system start (microseconds)

uint8 operation         # what to do (see constants below)
char[17] name           # parameter name (max 16 chars + null); ignored for RESET_ALL / GET_INDEX
uint16 index            # parameter index, used only by OPERATION_GET_INDEX

int32 int_value         # value to set when the parameter is INT32
float32 float_value     # value to set when the parameter is FLOAT32

uint8 OPERATION_GET = 0
uint8 OPERATION_SET = 1
uint8 OPERATION_RESET = 2
uint8 OPERATION_RESET_ALL = 3
uint8 OPERATION_SAVE = 4
uint8 OPERATION_GET_INDEX = 5
