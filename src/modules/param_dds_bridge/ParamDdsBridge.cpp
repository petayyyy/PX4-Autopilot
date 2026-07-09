#include "ParamDdsBridge.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#include <string.h>

ParamDdsBridge::ParamDdsBridge() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool ParamDdsBridge::init()
{
	if (!_request_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void ParamDdsBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	param_bridge_request_s req;

	while (_request_sub.update(&req)) {
		handle_request(req);
	}
}

void ParamDdsBridge::handle_request(const param_bridge_request_s &req)
{
	param_bridge_response_s resp{};
	resp.param_type = param_bridge_response_s::TYPE_UNKNOWN;
	resp.total = param_count();

	// Whole-set operations first.
	if (req.operation == param_bridge_request_s::OPERATION_RESET_ALL) {
		param_reset_all();
		resp.found = true;
		resp.success = true;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	if (req.operation == param_bridge_request_s::OPERATION_SAVE) {
		resp.found = true;
		resp.success = (param_save_default(true) == PX4_OK);
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	// Resolve the handle: by index for GET_INDEX (backup), else by name.
	param_t handle;

	if (req.operation == param_bridge_request_s::OPERATION_GET_INDEX) {
		resp.index = req.index;
		handle = param_for_index(req.index);

	} else {
		char name[sizeof(req.name)];
		memcpy(name, req.name, sizeof(name));
		name[sizeof(name) - 1] = 0;
		handle = param_find(name);
	}

	if (handle == PARAM_INVALID) {
		resp.found = false;
		resp.success = false;
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
		return;
	}

	resp.found = true;

	// Echo the resolved parameter name (authoritative for GET_INDEX).
	const char *pname = param_name(handle);

	if (pname != nullptr) {
		strncpy(resp.name, pname, sizeof(resp.name));
		resp.name[sizeof(resp.name) - 1] = 0;
	}

	const param_type_t ptype = param_type(handle);

	switch (req.operation) {
	case param_bridge_request_s::OPERATION_SET: {
			int ret = PX4_ERROR;

			if (ptype == PARAM_TYPE_INT32) {
				int32_t v = req.int_value;
				ret = param_set(handle, &v);

			} else if (ptype == PARAM_TYPE_FLOAT) {
				float v = req.float_value;
				ret = param_set(handle, &v);
			}

			resp.success = (ret == PX4_OK);
			break;
		}

	case param_bridge_request_s::OPERATION_RESET:
		param_reset(handle);
		resp.success = true;
		break;

	case param_bridge_request_s::OPERATION_GET:
	case param_bridge_request_s::OPERATION_GET_INDEX:
	default:
		resp.success = true;
		break;
	}

	// Read back current value into the response.
	if (ptype == PARAM_TYPE_INT32) {
		int32_t v = 0;
		param_get(handle, &v);
		resp.int_value = v;
		resp.param_type = param_bridge_response_s::TYPE_INT32;

	} else if (ptype == PARAM_TYPE_FLOAT) {
		float v = 0.f;
		param_get(handle, &v);
		resp.float_value = v;
		resp.param_type = param_bridge_response_s::TYPE_FLOAT;
	}

	resp.timestamp = hrt_absolute_time();
	_response_pub.publish(resp);
}

int ParamDdsBridge::task_spawn(int argc, char *argv[])
{
	ParamDdsBridge *instance = new ParamDdsBridge();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ParamDdsBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ParamDdsBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Set/get PX4 parameters by name over uXRCE-DDS. Subscribes to
param_bridge_request and answers on param_bridge_response.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("param_dds_bridge", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int param_dds_bridge_main(int argc, char *argv[])
{
	return ParamDdsBridge::main(argc, argv);
}
