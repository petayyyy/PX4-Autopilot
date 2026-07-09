# Response from log_download_bridge. Published on /fmu/out/log_download_response.

uint64 timestamp        # time since system start (microseconds)

uint8 operation         # echoed request operation
uint8 result            # RESULT_* below
uint32 file_size        # total size of the latest .ulg (bytes)
uint32 offset           # echoed request offset (OP_READ)
uint16 length           # number of valid bytes in data
uint8[240] data         # file bytes for this chunk (OP_READ)
char[64] name           # basename of the latest .ulg (OP_INFO)

uint8 OP_INFO = 0
uint8 OP_READ = 1

uint8 RESULT_OK = 0
uint8 RESULT_NO_FILE = 1
uint8 RESULT_EOF = 2
uint8 RESULT_ERROR = 3
