# Response from the param_dds_bridge module. Published on
# /fmu/out/param_bridge_response via uXRCE-DDS.

uint64 timestamp        # time since system start (microseconds)

char[17] name           # echoed / resolved parameter name
uint8 param_type        # TYPE_* below (matches PX4 param_type_t)
int32 int_value         # current value if INT32
float32 float_value     # current value if FLOAT32

uint16 index            # parameter index (meaningful for GET_INDEX / backup)
uint16 total            # total parameter count (param_count())

bool found              # parameter exists
bool success            # operation succeeded (and readback matched for SET)

uint8 TYPE_INT32 = 0
uint8 TYPE_FLOAT = 1
uint8 TYPE_UNKNOWN = 255
