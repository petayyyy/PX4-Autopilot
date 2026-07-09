#include "LogDownloadBridge.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/log_download_request.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

static constexpr const char *LOG_ROOT = "/fs/microsd/log";

LogDownloadBridge::~LogDownloadBridge()
{
	if (_fd >= 0) {
		close(_fd);
	}
}

void LogDownloadBridge::run()
{
	int sub = orb_subscribe(ORB_ID(log_download_request));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sub;
	fds[0].events = POLLIN;

	while (!should_exit()) {
		int pret = px4_poll(fds, 1, 1000);

		if (pret > 0 && (fds[0].revents & POLLIN)) {
			log_download_request_s req;

			if (orb_copy(ORB_ID(log_download_request), sub, &req) == PX4_OK) {
				handle_request(req);
			}
		}
	}

	orb_unsubscribe(sub);

	if (_fd >= 0) {
		close(_fd);
		_fd = -1;
	}
}

bool LogDownloadBridge::find_latest(char *out_path, size_t out_sz, uint32_t *out_size)
{
	// PX4 log paths encode creation order in their NAME, not in mtime (without an
	// RTC the file timestamps are unreliable). Dated logs: /log/YYYY-MM-DD/HH_MM_SS.ulg;
	// undated: /log/sess%03u/log%03u.ulg (session index increments monotonically).
	// So the last-created log is the lexicographically-largest path.
	DIR *root = opendir(LOG_ROOT);

	if (root == nullptr) {
		return false;
	}

	char best_path[192] = {0};
	uint32_t best_size = 0;
	bool found = false;
	struct dirent *de;

	while ((de = readdir(root)) != nullptr) {
		if (de->d_name[0] == '.') {
			continue;
		}

		char subpath[128];
		snprintf(subpath, sizeof(subpath), "%s/%s", LOG_ROOT, de->d_name);

		struct stat st;

		if (stat(subpath, &st) != 0) {
			continue;
		}

		if (S_ISDIR(st.st_mode)) {
			DIR *sub = opendir(subpath);

			if (sub == nullptr) {
				continue;
			}

			struct dirent *fe;

			while ((fe = readdir(sub)) != nullptr) {
				const char *ext = strrchr(fe->d_name, '.');

				if (ext == nullptr || strcmp(ext, ".ulg") != 0) {
					continue;
				}

				char fpath[192];
				snprintf(fpath, sizeof(fpath), "%s/%s", subpath, fe->d_name);

				if (!found || strcmp(fpath, best_path) > 0) {
					struct stat fst;

					if (stat(fpath, &fst) == 0) {
						strncpy(best_path, fpath, sizeof(best_path) - 1);
						best_path[sizeof(best_path) - 1] = '\0';
						best_size = (uint32_t)fst.st_size;
						found = true;
					}
				}
			}

			closedir(sub);

		} else if (S_ISREG(st.st_mode)) {
			const char *ext = strrchr(de->d_name, '.');

			if (ext != nullptr && strcmp(ext, ".ulg") == 0
			    && (!found || strcmp(subpath, best_path) > 0)) {
				strncpy(best_path, subpath, sizeof(best_path) - 1);
				best_path[sizeof(best_path) - 1] = '\0';
				best_size = st.st_size;
				found = true;
			}
		}
	}

	closedir(root);

	if (found) {
		strncpy(out_path, best_path, out_sz - 1);
		out_path[out_sz - 1] = '\0';
		*out_size = best_size;
	}

	return found;
}

void LogDownloadBridge::handle_request(const log_download_request_s &req)
{
	log_download_response_s resp{};
	resp.operation = req.operation;
	resp.offset = req.offset;

	if (req.operation == log_download_request_s::OP_INFO) {
		char path[192];
		uint32_t size = 0;

		if (find_latest(path, sizeof(path), &size)) {
			if (_fd >= 0) {
				close(_fd);
				_fd = -1;
			}

			_fd = open(path, O_RDONLY);

			if (_fd < 0) {
				resp.result = log_download_response_s::RESULT_ERROR;

			} else {
				_file_size = size;
				resp.result = log_download_response_s::RESULT_OK;
				resp.file_size = size;
				const char *base = strrchr(path, '/');
				base = (base != nullptr) ? base + 1 : path;
				strncpy(resp.name, base, sizeof(resp.name) - 1);
				PX4_INFO("latest log: %s (%u bytes)", path, (unsigned)size);
			}

		} else {
			resp.result = log_download_response_s::RESULT_NO_FILE;
			PX4_WARN("no .ulg found under %s", LOG_ROOT);
		}

		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	if (req.operation != log_download_request_s::OP_READ) {
		resp.result = log_download_response_s::RESULT_ERROR;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	// OP_READ
	resp.file_size = _file_size;

	if (_fd < 0) {
		resp.result = log_download_response_s::RESULT_ERROR;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	if (req.length == 0) {
		resp.result = log_download_response_s::RESULT_ERROR;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	if (req.offset >= _file_size) {
		resp.result = log_download_response_s::RESULT_EOF;
		resp.length = 0;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	uint16_t want = req.length;

	if (want > sizeof(resp.data)) {
		want = sizeof(resp.data);
	}

	if (lseek(_fd, req.offset, SEEK_SET) != (off_t)req.offset) {
		resp.result = log_download_response_s::RESULT_ERROR;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	ssize_t n = read(_fd, resp.data, want);

	if (n < 0) {
		resp.result = log_download_response_s::RESULT_ERROR;

	} else {
		resp.result = log_download_response_s::RESULT_OK;
		resp.length = (uint16_t)n;
	}

	resp.timestamp = hrt_absolute_time();
	_response_pub.publish(resp);
}

int LogDownloadBridge::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("log_download_bridge",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(3000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LogDownloadBridge *LogDownloadBridge::instantiate(int argc, char *argv[])
{
	return new LogDownloadBridge();
}

int LogDownloadBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LogDownloadBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Download the latest .ulg from the SD card over uXRCE-DDS, reliable pull by
offset (log_download_request/response). Runs as its own task (blocking file I/O).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("log_download_bridge", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int log_download_bridge_main(int argc, char *argv[])
{
	return LogDownloadBridge::main(argc, argv);
}
