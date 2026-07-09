/****************************************************************************
 * param_dds_bridge — set/get PX4 parameters by NAME over uXRCE-DDS.
 *
 * Subscribes to param_bridge_request (published by a companion via
 * /fmu/in/param_bridge_request) and answers on param_bridge_response
 * (/fmu/out/param_bridge_response). Works on single-FMU builds where the
 * built-in ParameterPrimary sync is not active.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/param_bridge_request.h>
#include <uORB/topics/param_bridge_response.h>

class ParamDdsBridge : public ModuleBase<ParamDdsBridge>, public px4::ScheduledWorkItem
{
public:
	ParamDdsBridge();
	~ParamDdsBridge() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void handle_request(const param_bridge_request_s &req);

	uORB::SubscriptionCallbackWorkItem _request_sub{this, ORB_ID(param_bridge_request)};
	uORB::Publication<param_bridge_response_s> _response_pub{ORB_ID(param_bridge_response)};
};
