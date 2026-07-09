/****************************************************************************
 * log_download_bridge — download the latest .ulg from the SD card over
 * uXRCE-DDS with a reliable, offset-addressed pull (companion re-requests
 * lost chunks). Runs as its OWN TASK (not a work-queue item) because it does
 * blocking SD file I/O, which must not run on a shared work queue.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/log_download_request.h>
#include <uORB/topics/log_download_response.h>

class LogDownloadBridge : public ModuleBase<LogDownloadBridge>
{
public:
	LogDownloadBridge() = default;
	~LogDownloadBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static LogDownloadBridge *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	void handle_request(const log_download_request_s &req);
	bool find_latest(char *out_path, size_t out_sz, uint32_t *out_size);

	int _fd{-1};
	uint32_t _file_size{0};

	uORB::Publication<log_download_response_s> _response_pub{ORB_ID(log_download_response)};
};
