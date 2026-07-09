# Request to log_download_bridge: fetch the latest .ulg from SD over uXRCE-DDS.
# Companion publishes on /fmu/in/log_download_request.

uint64 timestamp        # time since system start (microseconds)

uint8 operation         # OP_INFO = open latest log + report size; OP_READ = read a chunk
uint32 offset           # byte offset into the file (OP_READ)
uint16 length           # bytes to read (OP_READ), capped to the response data size

uint8 OP_INFO = 0
uint8 OP_READ = 1
