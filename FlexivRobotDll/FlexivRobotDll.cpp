#define _CRT_SECURE_NO_WARNINGS
#define EXPORT_API extern "C" __declspec(dllexport)

#include <cstdio>
#include <iostream>
#include <string>
#include <cstring>
#include <stdexcept>
#include <map>
#include <vector>
#include <array>

#include <comdef.h>
#include <comutil.h>

#include <flexiv\rdk\data.hpp>
#include <flexiv\rdk\robot.hpp>
#include <flexiv\rdk\utility.hpp>
#include <flexiv\rdk\tool.hpp>
#include <flexiv\rdk\work_coord.hpp>

struct FlexivError {	         // 传递异常
	int error_code;              // 错误代码
	char error_msg[512];         // 错误信息
};

struct Pointer {
	char ptr[256];
};

struct W_RobotStates {
	double q[7];
	double theta[7];
	double dq[7];
	double dtheta[7];
	double tau[7];
	double tau_des[7];
	double tau_dot[7];
	double tau_ext[7];
	// 未添加外部轴数据
	double tcp_pose[7];
	double tcp_pose_des[7];
	double tcp_vel[6];
	double flange_pose[7];
	double ft_sensor_raw[6];
	double ext_wrench_in_tcp[6];
	double ext_wrench_in_world[6];
	double ext_wrench_in_tcp_raw[6];
	double ext_wrench_in_world_raw[6];
};

struct W_PlanInfo {
	char pt_name[512];
	char node_name[512];
	char node_path[512];
	char node_path_time_period[512];
	char node_path_number[512];
	char assigned_plan_name[512];
	double velocity_scale;
	bool waiting_for_step;
};

struct W_Coord {
	double x;
	double y;
	double z;
	double rx;
	double ry;
	double rz;
	char coord_type[256];
	char coord_name[256];
};

struct W_Joint {
	double A1;
	double A2;
	double A3;
	double A4;
	double A5;
	double A6;
	double A7;
	double E1;
	double E2;
	double E3;
	double E4;
	double E5;
	double E6;
};

struct W_ToolParams {
	double mass;
	double CoM_x;
	double CoM_y;
	double CoM_z;
	double Ixx;
	double Iyy;
	double Izz;
	double Ixy;
	double Ixz;
	double Iyz;
	double tcp_x;
	double tcp_y;
	double tcp_z;
	double tcp_qw;
	double tcp_qx;
	double tcp_qy;
	double tcp_qz;
};

struct W_ForceWrench {
	double fx;
	double fy;
	double fz;
	double mx;
	double my;
	double mz;
};

void CopyExceptionMsg(const std::exception& e, FlexivError* error) {
	std::strncpy(error->error_msg, e.what(), sizeof(error->error_msg) - 1);
	error->error_msg[sizeof(error->error_msg) - 1] = '\0';
}

void CopyMsgSrc2Dst(const char* src, char* dst) {
	std::strncpy(dst, src, sizeof(dst) - 1);
	dst[sizeof(dst) - 1] = '\0';
}

flexiv::rdk::Coord ToRdkCoord(const W_Coord& w_c) {
	flexiv::rdk::Coord coord;
	coord.position[0] = w_c.x;
	coord.position[1] = w_c.y;
	coord.position[2] = w_c.z;
	coord.orientation[0] = w_c.rx;
	coord.orientation[1] = w_c.ry;
	coord.orientation[2] = w_c.rz;
	coord.ref_frame[0] = w_c.coord_type;
	coord.ref_frame[1] = w_c.coord_name;
	return coord;
}

EXPORT_API void quat_2_euler_zyx(double qw, double qx, double qy, double qz,
	double& x, double& y, double& z) {    // 四元数转ZYX欧拉角
	std::array<double, 4> quat{ qw, qx, qy, qz };
	std::array<double, 3> zyx = flexiv::rdk::utility::Quat2EulerZYX(quat);
	x = zyx[0];
	y = zyx[1];
	z = zyx[2];
}


EXPORT_API flexiv::rdk::Robot* CreateFlexivRobot(const char* robot_sn, FlexivError* error) {
	try {
		flexiv::rdk::Robot* robot = new flexiv::rdk::Robot(robot_sn);
		error->error_code = 0;
		return robot;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API flexiv::rdk::Robot* CreateFlexivRobotWithWhiteList(const char* robot_sn,
	const char** interfaces, int interface_count, FlexivError* error) {
	std::vector<std::string> white_list;
	for (int i = 0; i < interface_count; ++i) {
		white_list.push_back(interfaces[i]);
	}
	try {
		flexiv::rdk::Robot* robot = new flexiv::rdk::Robot(robot_sn, white_list);
		error->error_code = 0;
		return robot;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API flexiv::rdk::Tool* CreateFlexivTool(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		flexiv::rdk::Tool* tool = new flexiv::rdk::Tool(*robot);
		error->error_code = 0;
		return tool;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API flexiv::rdk::WorkCoord* CreateFlexivWork(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		flexiv::rdk::WorkCoord* work = new flexiv::rdk::WorkCoord(*robot);
		error->error_code = 0;
		return work;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteFlexivRobot(flexiv::rdk::Robot* robot) {
	delete robot;
}

EXPORT_API void DeleteFlexivTool(flexiv::rdk::Tool* tool) {
	delete tool;
}

EXPORT_API void DeleteFlexivWork(flexiv::rdk::WorkCoord* work) {
	delete work;
}

EXPORT_API bool fault(flexiv::rdk::Robot* robot) {
	return robot->fault();
}

EXPORT_API bool ClearFault(flexiv::rdk::Robot* robot, int timeout_sec, FlexivError* error) {
	try {
		bool flag = robot->ClearFault(timeout_sec);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void Enable(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		robot->Enable();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API bool operational(flexiv::rdk::Robot* robot, bool verbose = true) {
	return robot->operational(verbose);
}

EXPORT_API bool is_busy(flexiv::rdk::Robot* robot) {
	return robot->busy();
}

EXPORT_API bool is_stopped(flexiv::rdk::Robot* robot) {
	return robot->stopped();
}

EXPORT_API bool is_in_reduced_state(flexiv::rdk::Robot* robot) {
	return robot->reduced();
}

EXPORT_API bool is_in_recovery_state(flexiv::rdk::Robot* robot) {
	return robot->recovery();
}

EXPORT_API bool estop_is_released(flexiv::rdk::Robot* robot) {
	return robot->estop_released();
}

EXPORT_API bool enabling_button_is_pressed(flexiv::rdk::Robot* robot) {
	return robot->enabling_button_pressed();
}

EXPORT_API void robot_brake(flexiv::rdk::Robot* robot,
	bool engage, FlexivError* error) {
	try {
		robot->Brake(engage);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void run_auto_recovery(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		robot->RunAutoRecovery();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API int get_global_int_value(flexiv::rdk::Robot* robot, const char* key, bool& flag, FlexivError* error) {
	try {
		auto global_vari = robot->global_variables();
		std::string chk_key(key);
		int res = 0;
		if (global_vari.find(chk_key) != global_vari.end()) {
			flag = false;
			res = std::get<int>(global_vari[chk_key]);
		}
		else {
			flag = true;
		}
		error->error_code = 0;
		return res;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return 0;
	}
}

EXPORT_API int get_global_double_value(flexiv::rdk::Robot* robot, const char* key, bool& flag, FlexivError* error) {
	try {
		auto global_vari = robot->global_variables();
		std::string chk_key(key);
		double res = 0;
		if (global_vari.find(chk_key) != global_vari.end()) {
			flag = false;
			res = std::get<double>(global_vari[chk_key]);
		}
		else {
			flag = true;
		}
		error->error_code = 0;
		return res;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return 0;
	}
}

EXPORT_API void set_global_int_value(flexiv::rdk::Robot* robot, const char* key, int value, FlexivError* error) {
	try {
		std::string chk_key(key);
		std::map<std::string, flexiv::rdk::FlexivDataTypes> vars;
		vars[chk_key] = value;
		robot->SetGlobalVariables(vars);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_global_double_value(flexiv::rdk::Robot* robot, const char* key, double value, FlexivError* error) {
	try {
		std::string chk_key(key);
		std::map<std::string, flexiv::rdk::FlexivDataTypes> vars;
		vars[chk_key] = value;
		robot->SetGlobalVariables(vars);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API int get_current_mode(flexiv::rdk::Robot* robot) {
	flexiv::rdk::Mode mode = robot->mode();
	using namespace flexiv::rdk;
	int ret = 0;
	if (mode == Mode::UNKNOWN) {
		ret = 0;
	}
	else if (mode == Mode::IDLE) {
		ret = 1;
	}
	else if (mode == Mode::RT_JOINT_TORQUE) {
		ret = 2;
	}
	else if (mode == Mode::RT_JOINT_IMPEDANCE) {
		ret = 3;
	}
	else if (mode == Mode::NRT_JOINT_IMPEDANCE) {
		ret = 4;
	}
	else if (mode == Mode::RT_JOINT_POSITION) {
		ret = 5;
	}
	else if (mode == Mode::NRT_JOINT_POSITION) {
		ret = 6;
	}
	else if (mode == Mode::NRT_PLAN_EXECUTION) {
		ret = 7;
	}
	else if (mode == Mode::NRT_PRIMITIVE_EXECUTION) {
		ret = 8;
	}
	else if (mode == Mode::RT_CARTESIAN_MOTION_FORCE) {
		ret = 9;
	}
	else if (mode == Mode::NRT_CARTESIAN_MOTION_FORCE) {
		ret = 10;
	}
	else if (mode == Mode::MODES_CNT) {
		ret = 11;
	}
	else {}

	return ret;
}

EXPORT_API void GetRobotStates(flexiv::rdk::Robot* robot, W_RobotStates* robot_states) {
	const flexiv::rdk::RobotStates& states = robot->states();
	for (int i = 0; i < 7; ++i) {
		robot_states->q[i] = states.q[i];
		robot_states->theta[i] = states.theta[i];
		robot_states->dq[i] = states.dq[i];
		robot_states->dtheta[i] = states.dtheta[i];
		robot_states->tau[i] = states.tau[i];
		robot_states->tau_des[i] = states.tau_des[i];
		robot_states->tau_dot[i] = states.tau_dot[i];
		robot_states->tau_ext[i] = states.tau_ext[i];
		robot_states->tcp_pose[i] = states.tcp_pose[i];
		robot_states->tcp_pose_des[i] = states.tcp_pose_des[i];
		robot_states->flange_pose[i] = states.flange_pose[i];
	}
	for (int i = 0; i < 6; ++i) {
		robot_states->tcp_vel[i] = states.tcp_vel[i];
		robot_states->ft_sensor_raw[i] = states.ft_sensor_raw[i];
		robot_states->ext_wrench_in_tcp[i] = states.ext_wrench_in_tcp[i];
		robot_states->ext_wrench_in_world[i] = states.ext_wrench_in_world[i];
		robot_states->ext_wrench_in_tcp_raw[i] = states.ext_wrench_in_tcp_raw[i];
		robot_states->ext_wrench_in_world_raw[i] = states.ext_wrench_in_world_raw[i];
	}
}

EXPORT_API void get_plan_info(flexiv::rdk::Robot* robot, W_PlanInfo* plan_info, FlexivError* error) {
	try {
		const auto& info = robot->plan_info();
		CopyMsgSrc2Dst(info.pt_name.c_str(), plan_info->pt_name);
		CopyMsgSrc2Dst(info.node_name.c_str(), plan_info->node_name);
		CopyMsgSrc2Dst(info.node_path.c_str(), plan_info->node_path);
		CopyMsgSrc2Dst(info.node_path_time_period.c_str(), plan_info->node_path_time_period);
		CopyMsgSrc2Dst(info.node_path_number.c_str(), plan_info->node_path_number);
		CopyMsgSrc2Dst(info.assigned_plan_name.c_str(), plan_info->assigned_plan_name);
		plan_info->velocity_scale = info.velocity_scale;
		plan_info->waiting_for_step = info.waiting_for_step;
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SwitchRobotMode(flexiv::rdk::Robot* robot, int mode, FlexivError* error) {
	flexiv::rdk::Mode robot_mode = flexiv::rdk::Mode::UNKNOWN;
	switch (mode) {
	case 0:
		robot_mode = flexiv::rdk::Mode::UNKNOWN;
		break;
	case 1:
		robot_mode = flexiv::rdk::Mode::IDLE;
		break;
	case 2:
		robot_mode = flexiv::rdk::Mode::RT_JOINT_TORQUE;
		break;
	case 3:
		robot_mode = flexiv::rdk::Mode::RT_JOINT_IMPEDANCE;
		break;
	case 4:
		robot_mode = flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
		break;
	case 5:
		robot_mode = flexiv::rdk::Mode::RT_JOINT_POSITION;
		break;
	case 6:
		robot_mode = flexiv::rdk::Mode::NRT_JOINT_POSITION;
		break;
	case 7:
		robot_mode = flexiv::rdk::Mode::NRT_PLAN_EXECUTION;
		break;
	case 8:
		robot_mode = flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION;
		break;
	case 9:
		robot_mode = flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE;
		break;
	case 10:
		robot_mode = flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE;
		break;
	case 11:
		robot_mode = flexiv::rdk::Mode::MODES_CNT;
		break;
	default:
		break;
	}
	try
	{
		robot->SwitchMode(robot_mode);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Stop(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		robot->Stop();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_velocity_scale(flexiv::rdk::Robot* robot, int velocity_scale, FlexivError* error) {
	try {
		robot->SetVelocityScale(velocity_scale);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API int curr_prim_specified_integer(flexiv::rdk::Robot* robot, const char* key,
	bool& flag, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		std::string chk_key(key);
		int res = 0;
		if (pt_states.find(chk_key) != pt_states.end()) {
			flag = false;
			res = std::get<int>(pt_states[chk_key]);
		}
		else {
			flag = true;
		}
		error->error_code = 0;
		return res;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return 0;
	}
}

EXPORT_API double curr_prim_specified_double(flexiv::rdk::Robot* robot, const char* key,
	bool& flag, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		std::string chk_key(key);
		double res = 0;
		if (pt_states.find(chk_key) != pt_states.end()) {
			flag = false;
			res = std::get<double>(pt_states[chk_key]);
		}
		else {
			flag = true;
		}
		error->error_code = 0;
		return res;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return 0;
	}
}

EXPORT_API bool HasReachedTarget(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("reachedTarget") != pt_states.end()) {
			flag = std::get<int>(pt_states["reachedTarget"]) == 1;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_has_terminated(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("terminated") != pt_states.end()) {
			flag = std::get<int>(pt_states["terminated"]) == 1;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_has_align_contacted(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("alignContacted") != pt_states.end()) {
			flag = std::get<int>(pt_states["alignContacted"]) == 1;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_push_distance_is_exceeded(flexiv::rdk::Robot* robot,
	double threshold, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("pushDistance") != pt_states.end()) {
			flag = std::get<int>(pt_states["pushDistance"]) > threshold;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_has_check_complete(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("checkComplete") != pt_states.end()) {
			flag = std::get<int>(pt_states["checkComplete"]) == 1;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_is_moving_equal_zero(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("isMoving") != pt_states.end()) {
			flag = std::get<int>(pt_states["isMoving"]) == 0;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_has_mating_finished(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("matingFinished") != pt_states.end()) {
			flag = std::get<int>(pt_states["matingFinished"]) == 1;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API bool current_primitive_has_fasten_state(flexiv::rdk::Robot* robot, FlexivError* error) {
	try {
		auto pt_states = robot->primitive_states();
		bool flag = false;
		if (pt_states.find("fastenState") != pt_states.end()) {
			flag = std::get<int>(pt_states["fastenState"]) == 2;
		}
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void Home(flexiv::rdk::Robot* robot, int jntVelScale, FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["jntVelScale"] = jntVelScale;
		robot->ExecutePrimitive("Home", input_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void	MoveL(flexiv::rdk::Robot* robot,
	W_Coord* target,
	W_Coord* waypoint, int way_len, bool has_way,
	double vel, bool has_vel,
	const char* zoneRadius, bool has_zoneRadius,
	int targetTolerLevel, bool has_targetTolerLevel,
	double acc, bool has_acc,
	double angVel, bool has_angVel,
	double jerk, bool has_jerk,
	double* configOptObj, int config_len, bool has_config,
	bool block_until_started,
	FlexivError* error) {
	if (robot == nullptr) {
		error->error_code = 1;
		const char* error_str = "flexiv robot pointer is nullptr.";
		std::strncpy(error->error_msg, error_str, sizeof(error->error_msg) - 1);
		error->error_msg[sizeof(error->error_msg) - 1] = '\0';
		return;
	}
	std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
	input_params["target"] = ToRdkCoord(*target);
	if (has_way) {
		std::vector<flexiv::rdk::Coord> way_points;
		for (int i = 0; i < way_len; ++i) {
			W_Coord* ptr = waypoint + i;
			way_points.push_back(ToRdkCoord(*ptr));
		}
		input_params["waypoints"] = way_points;
	}
	if (has_vel) input_params["vel"] = vel;
	if (has_zoneRadius) input_params["zoneRadius"] = std::string(zoneRadius);
	if (has_targetTolerLevel) input_params["targetTolerLevel"] = targetTolerLevel;
	if (has_acc) input_params["acc"] = acc;
	if (has_angVel) input_params["angVel"] = angVel;
	if (has_jerk) input_params["jerk"] = jerk;
	if (has_config) {
		if (config_len != 3) {
			error->error_code = 1;
			const char* error_str = "configOptObj parameters size should be equal to 3.";
			std::strncpy(error->error_msg, error_str, sizeof(error->error_msg) - 1);
			error->error_msg[sizeof(error->error_msg) - 1] = '\0';
			return;
		}
		std::vector<double> config(configOptObj, configOptObj + config_len);
		input_params["configOptObj"] = config;
	}
	try {
		robot->ExecutePrimitive("MoveL", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void MoveJ(flexiv::rdk::Robot* robot,
	W_Joint* target,
	W_Joint* waypoint, int way_len, bool has_way,
	int jntVelScale, bool has_jntVelScale,
	const char* zoneRadius, bool has_zoneRadius,
	int targetTolerLevel, bool has_targetTolerLevel,
	bool enableRelativeMove, bool has_relateMove,
	bool block_until_started,
	FlexivError* error) {
	if (robot == nullptr) {
		error->error_code = 1;
		const char* error_str = "flexiv robot pointer is nullptr.";
		std::strncpy(error->error_msg, error_str, sizeof(error->error_msg) - 1);
		error->error_msg[sizeof(error->error_msg) - 1] = '\0';
		return;
	}

	std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
	flexiv::rdk::JPos tgt({ target->A1, target->A2, target->A3, target->A4, target->A5, target->A6, target->A7 },
		{ target->E1, target->E2, target->E3, target->E4, target->E5, target->E6 });
	input_params["target"] = tgt;
	if (has_way) {
		std::vector<flexiv::rdk::JPos> waypoints;
		// std::vector<double> waypoints;
		for (int i = 0; i < way_len; ++i) {
			W_Joint* jnt = waypoint + i;
			waypoints.push_back(flexiv::rdk::JPos({ jnt->A1, jnt->A2, jnt->A3, jnt->A4, jnt->A5, jnt->A6, jnt->A7 },
				{ jnt->E1, jnt->E2, jnt->E3, jnt->E4, jnt->E5, jnt->E6 }));
		}
		input_params["waypoints"] = waypoints;
	}
	if (has_jntVelScale) input_params["jntVelScale"] = jntVelScale;
	if (has_zoneRadius) input_params["zoneRadius"] = zoneRadius;
	if (has_targetTolerLevel) input_params["targetTolerLevel"] = targetTolerLevel;
	if (has_relateMove) input_params["enableRelativeMove"] = has_relateMove ? 1 : 0;
	try {
		robot->ExecutePrimitive("MoveJ", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}

}

EXPORT_API void MoveC(flexiv::rdk::Robot* robot,
	W_Coord* target,
	W_Coord* middlePose,
	double vel, bool has_vel,
	int targetTolerLevel, bool has_targetTolerLevel,
	double acc, bool has_acc,
	double angVel, bool has_angVel,
	double jerk, bool has_jerk,
	double* configOptObj, int config_len, bool has_config,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["target"] = ToRdkCoord(*target);
		input_params["middlePose"] = ToRdkCoord(*middlePose);
		if (has_vel) input_params["vel"] = vel;
		if (has_targetTolerLevel) input_params["targetTolerLevel"] = targetTolerLevel;
		if (has_acc) input_params["acc"] = acc;
		if (has_angVel) input_params["angVel"] = angVel;
		if (has_jerk) input_params["jerk"] = jerk;
		if (has_config) {
			if (config_len != 3) {
				error->error_code = 1;
				const char* error_str = "configOptObj parameters size should be equal to 3.";
				std::strncpy(error->error_msg, error_str, sizeof(error->error_msg) - 1);
				error->error_msg[sizeof(error->error_msg) - 1] = '\0';
				return;
			}
			std::vector<double> config(configOptObj, configOptObj + config_len);
			input_params["configOptObj"] = config;
		}
		robot->ExecutePrimitive("MoveC", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Contact(flexiv::rdk::Robot* robot, const char* contactCoord, bool has_contactCoord,
	double* contactDir, int dir_len, bool has_contactDir,
	double contactVel, bool has_contactVel,
	double maxContactForce, bool has_maxContactForce,
	bool enableFineContact, bool has_enbaleFineContact,
	W_Coord* waypoint, int way_len, bool has_way,
	double* vel, int vel_len, bool has_vel,
	double* acc, int acc_len, bool has_acc,
	const char* zoneRadius[], int count, bool has_zoneRadius,
	double jerk, bool has_jerk,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_contactCoord) input_params["contactCoord"] = std::string(contactCoord);
		if (has_contactDir) {
			std::vector<double> dir(contactDir, contactDir + dir_len);
			input_params["contactDir"] = dir;
		}
		if (has_maxContactForce) input_params["maxContactForce"] = maxContactForce;
		if (has_enbaleFineContact) input_params["enableFineContact"] = enableFineContact ? 1 : 0;
		if (has_way) {
			std::vector<flexiv::rdk::Coord> way_points;
			for (int i = 0; i < way_len; ++i) {
				W_Coord* ptr = waypoint + i;
				way_points.push_back(ToRdkCoord(*ptr));
			}
			input_params["waypoints"] = way_points;
		}
		if (has_vel) {
			std::vector<double> vels(vel, vel + vel_len);
			input_params["vel"] = vels;
		}
		if (has_acc) {
			std::vector<double> accs(acc, acc + acc_len);
			input_params["acc"] = accs;
		}
		if (has_zoneRadius) {
			std::vector<std::string> zrs(zoneRadius, zoneRadius + count);
			input_params["zoneRadius"] = zrs;
		}

		if (has_jerk) input_params["jerk"] = jerk;
		if (input_params.empty()) {
			robot->ExecutePrimitive("Contact", std::map<std::string, flexiv::rdk::FlexivDataTypes> {}, {});
		}
		else {
			robot->ExecutePrimitive("Contact", input_params, {}, block_until_started);
		}
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void ContactAlign(flexiv::rdk::Robot* robot,
	double* contactAxis, int contactAxis_len, bool has_contactAxis,
	double contactVel, bool has_contactVel,
	double contactForce, bool has_contactForce,
	int* alignAxis, int alignAxis_len, bool has_alignAxis,
	double alignVelScale, bool has_alignVelScale,
	double deadbandScale, bool has_deadbandScale,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_contactAxis) {
			std::vector<double> axis(contactAxis, contactAxis + contactAxis_len);
			input_params["contactAxis"] = axis;
		}
		if (has_contactVel) input_params["contactVel"] = contactVel;
		if (has_contactForce) input_params["contactForce"] = contactForce;
		if (has_alignAxis) {
			std::vector<int> align(alignAxis, alignAxis + alignAxis_len);
			input_params["alignAxis"] = align;
		}
		if (has_alignVelScale) input_params["alignVelScale"] = alignVelScale;
		if (has_deadbandScale) input_params["deadbandScale"] = deadbandScale;
		robot->ExecutePrimitive("ContactAlign", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void ForceHybrid(flexiv::rdk::Robot* robot,
	W_Coord* target,
	W_Coord* waypoints_ptr, int waypoints_len, bool has_waypoints,
	W_ForceWrench* wrench_ptr, int wrench_len, bool has_wrench,
	double vel, bool has_vel,
	double acc, bool has_acc,
	const char* zoneRadius, bool has_zoneRadius,
	int targetTolerLevel, bool has_targetTolerLevel,
	W_Coord* forceCoord, bool has_forceCoord,
	int* forceAxis_ptr, int forceAxis_len, bool has_forceAxis,
	double* targetWrench_ptr, int targetWrench_len, bool has_targetWrench,
	double angVel, bool has_angVel,
	double jerk, bool has_jerk,
	double* configOptObj_ptr, int configOptObj_len, bool has_configOptObj,
	double* stiffScale_ptr, int stiffScale_len, bool has_stiffScale,
	int* enableMaxWrench_ptr, int enableMaxWrench_len, bool has_enableMaxWrench,
	double* maxContactWrench_ptr, int maxContactWrench_len, bool has_maxContactWrench,
	double* maxVelForceDir_ptr, int maxVelForceDir_len, bool has_maxVelForceDir,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["target"] = ToRdkCoord(*target);
		if (has_waypoints) {
			std::vector<flexiv::rdk::Coord> way_points;
			for (int i = 0; i < waypoints_len; ++i) {
				W_Coord* ptr = waypoints_ptr + i;
				way_points.push_back(ToRdkCoord(*ptr));
			}
			input_params["waypoints"] = way_points;
		}
		if (has_wrench) {
			std::vector<double> wrench;
			for (int i = 0; i < wrench_len; ++i) {
				W_ForceWrench* ptr = wrench_ptr + i;
				wrench.push_back(ptr->fx);
				wrench.push_back(ptr->fy);
				wrench.push_back(ptr->fz);
				wrench.push_back(ptr->mx);
				wrench.push_back(ptr->my);
				wrench.push_back(ptr->mz);
			}
			input_params["wrench"] = wrench;
		}
		if (has_vel) {
			input_params["vel"] = vel;
		}
		if (has_acc) input_params["acc"] = acc;
		if (has_zoneRadius) input_params["zoneRadius"] = std::string(zoneRadius);
		if (has_targetTolerLevel) input_params["targetTolerLevel"] = targetTolerLevel;
		if (has_forceCoord) input_params["forceCoord"] = ToRdkCoord(*forceCoord);
		if (has_forceAxis) {
			std::vector<int> forceAxis(forceAxis_ptr, forceAxis_ptr + forceAxis_len);
			input_params["forceAxis"] = forceAxis;
		}
		if (has_targetWrench) {
			std::vector<double> targetWrench(targetWrench_ptr, targetWrench_ptr + targetWrench_len);
			input_params["targetWrench"] = targetWrench;
		}
		if (has_angVel) input_params["angVel"] = angVel;
		if (has_jerk) input_params["jerk"] = jerk;
		if (has_configOptObj) {
			std::vector<double> configOptObj(configOptObj_ptr, configOptObj_ptr + configOptObj_len);
			input_params["configOptObj"] = configOptObj;
		}
		if (has_stiffScale) {
			std::vector<double> stiffScale(stiffScale_ptr, stiffScale_ptr + stiffScale_len);
			input_params["stiffScale"] = stiffScale;
		}
		if (has_enableMaxWrench) {
			std::vector<int> enableMaxWrench(enableMaxWrench_ptr, enableMaxWrench_ptr + enableMaxWrench_len);
			input_params["enableMaxWrench"] = enableMaxWrench;
		}
		if (has_maxContactWrench) {
			std::vector<double> maxContactWrench(maxContactWrench_ptr, maxContactWrench_ptr + maxContactWrench_len);
			input_params["maxContactWrench"] = maxContactWrench;
		}
		if (has_maxVelForceDir) {
			std::vector<double> maxVelForceDir(maxVelForceDir_ptr, maxVelForceDir_ptr + maxVelForceDir_len);
			input_params["maxVelForceDir"] = maxVelForceDir;
		}
		robot->ExecutePrimitive("ForceHybrid", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void ForceComp(flexiv::rdk::Robot* robot,
	W_Coord* target,
	W_Coord* waypoints_ptr, int waypoints_len, bool has_waypoints,
	double vel, bool has_vel,
	const char* zoneRadius, bool has_zoneRadius,
	int targetTolerLevel, bool has_targetTolerLevel,
	W_Coord* compCoord, bool has_compCoord,
	double* stiffScale_ptr, int stiffScale_len, bool has_stiffScale,
	int* enableMaxWrench_ptr, int enableMaxWrench_len, bool has_enableMaxWrench,
	double* maxContactWrench_ptr, int maxContactWrench_len, bool has_maxContactWrench,
	double acc, bool has_acc,
	double angVel, bool has_angVel,
	double jerk, bool has_jerk,
	double* configOptObj_ptr, int configOptObj_len, bool has_configOptObj,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["target"] = ToRdkCoord(*target);
		if (has_waypoints) {
			std::vector<flexiv::rdk::Coord> way_points;
			for (int i = 0; i < waypoints_len; ++i) {
				W_Coord* ptr = waypoints_ptr + i;
				way_points.push_back(ToRdkCoord(*ptr));
			}
			input_params["waypoints"] = way_points;
		}
		if (has_vel) input_params["vel"] = vel;
		if (has_zoneRadius) input_params["zoneRadius"] = std::string(zoneRadius);
		if (has_targetTolerLevel) input_params["targetTolerLevel"] = targetTolerLevel;
		if (has_compCoord) input_params["compCoord"] = ToRdkCoord(*compCoord);
		if (has_stiffScale) {
			std::vector<double> stiffScale(stiffScale_ptr, stiffScale_ptr + stiffScale_len);
			input_params["stiffScale"] = stiffScale;
		}
		if (has_enableMaxWrench) {
			std::vector<int> enableMaxWrench(enableMaxWrench_ptr, enableMaxWrench_ptr + enableMaxWrench_len);
			input_params["enableMaxWrench"] = enableMaxWrench;
		}
		if (has_maxContactWrench) {
			std::vector<double> maxContactWrench(maxContactWrench_ptr, maxContactWrench_ptr + maxContactWrench_len);
			input_params["maxContactWrench"] = maxContactWrench;
		}
		if (has_acc) input_params["acc"] = acc;
		if (has_angVel) input_params["angVel"] = angVel;
		if (has_jerk) input_params["jerk"] = jerk;
		if (has_configOptObj) {
			std::vector<double> configOptObj(configOptObj_ptr, configOptObj_ptr + configOptObj_len);
			input_params["configOptObj"] = configOptObj;
		}
		robot->ExecutePrimitive("ForceComp", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SearchHole(flexiv::rdk::Robot* robot,
	double* contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
	double contactForce, bool has_contactForce,
	double* searchAxis_ptr, int searchAxis_len, bool has_searchAxis,
	const char* searchPattern, bool has_searchPattern,
	double spiralRadius, bool has_spiralRadius,
	double zigzagLength, bool has_zigzagLength,
	double zigzagWidth, bool has_zigzagWidth,
	int startDensity, bool has_startDensity,
	int timeFactor, bool has_timeFactor,
	double wiggleRange, bool has_wiggleRange,
	double wigglePeriod, bool has_wigglePeriod,
	int searchImmed, bool has_searchImmed,
	double searchStiffRatio, bool has_searchStiffRatio,
	double maxVelForceDir, bool has_maxVelForceDir,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_contactAxis) {
			std::vector<double> contactAxis(contactAxis_ptr, contactAxis_ptr + contactAxis_len);
			input_params["contactAxis"] = contactAxis;
		}
		if (has_contactForce) input_params["contactForce"] = contactForce;
		if (has_searchAxis) {
			std::vector<double> searchAxis(searchAxis_ptr, searchAxis_ptr + searchAxis_len);
			input_params["searchAxis"] = searchAxis;
		}
		if (has_searchPattern) input_params["searchPattern"] = std::string(searchPattern);
		if (has_spiralRadius) input_params["spiralRadius"] = spiralRadius;
		if (has_zigzagLength) input_params["zigzagLength"] = zigzagLength;
		if (has_zigzagWidth) input_params["zigzagWidth"] = zigzagWidth;
		if (has_startDensity) input_params["startDensity"] = startDensity;
		if (has_timeFactor) input_params["timeFactor"] = timeFactor;
		if (has_wiggleRange) input_params["wiggleRange"] = wiggleRange;
		if (has_wigglePeriod) input_params["wigglePeriod"] = wigglePeriod;
		if (has_searchImmed) input_params["searchImmed"] = searchImmed;
		if (has_searchStiffRatio) input_params["searchStiffRatio"] = searchStiffRatio;
		if (has_maxVelForceDir) input_params["maxVelForceDir"] = maxVelForceDir;
		robot->ExecutePrimitive("SearchHole", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void CheckPiH(flexiv::rdk::Robot* robot,
	double* contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
	double* searchAxis_ptr, int searchAxis_len, bool has_searchAxis,
	double searchRange, bool has_searchRange,
	double searchForce, bool has_searchForce,
	double searchVel, bool has_searchVel,
	int linearSearchOnly, bool has_linearSearchOnly,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_contactAxis) {
			std::vector<double> contactAxis(contactAxis_ptr, contactAxis_ptr + contactAxis_len);
			input_params["contactAxis"] = contactAxis;
		}
		if (has_searchAxis) {
			std::vector<double> searchAxis(searchAxis_ptr, searchAxis_ptr + searchAxis_len);
			input_params["searchAxis"] = searchAxis;
		}
		if (has_searchRange) input_params["searchRange"] = searchRange;
		if (has_searchForce) input_params["searchForce"] = searchForce;
		if (has_searchVel) input_params["searchVel"] = searchVel;
		if (has_linearSearchOnly) input_params["linearSearchOnly"] = linearSearchOnly;
		robot->ExecutePrimitive("CheckPiH", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void InsertComp(flexiv::rdk::Robot* robot,
	const char* insertAxis_ptr,
	int* compAxis_ptr, int compAxis_len, bool has_compAxis,
	double maxContactForce, bool has_maxContactForce,
	double deadbandScale, bool has_deadbandScale,
	double insertVel, bool has_insertVel,
	double compVelScale, bool has_compVelScale,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["insertAxis"] = std::string(insertAxis_ptr);
		if (has_compAxis) {
			std::vector<double> compAxis(compAxis_ptr, compAxis_ptr + compAxis_len);
			input_params["compAxis"] = compAxis;
		}
		if (has_maxContactForce) input_params["maxContactForce"] = maxContactForce;
		if (has_deadbandScale) input_params["deadbandScale"] = deadbandScale;
		if (has_insertVel) input_params["insertVel"] = insertVel;
		if (has_compVelScale) input_params["compVelScale"] = compVelScale;
		robot->ExecutePrimitive("InsertComp", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Mate(flexiv::rdk::Robot* robot,
	int* contactAxis_ptr, int contactAxis_len, bool has_contactAxis,
	double contactForce, bool has_contactForce,
	int* matingAxis_ptr, int matingAxis_len, bool has_matingAxis,
	double slideMatingRange, bool has_slideMatingRange,
	double slideMatingVel, bool has_slideMatingVel,
	double slideMatingAcc, bool has_slideMatingAcc,
	double rotateMatingRange, bool has_rotateMatingRange,
	double rotateMatingVel, bool has_rotateMatingVel,
	double rotateMatingAcc, bool has_rotateMatingAcc,
	int matingTimes, bool has_matingTimes,
	double maxContactDis, bool has_maxContactDis,
	double safetyForce, bool has_safetyForce,
	int* addMatingAxis_ptr, int addMatingAxis_len, bool has_addMatingAxis,
	double addSlideMatingRange, bool has_addSlideMatingRange,
	double addSlideMatingVel, bool has_addSlideMatingVel,
	double addSlideMatingAcc, bool has_addSlideMatingAcc,
	double addRotateMatingRange, bool has_addRotateMatingRange,
	double addRotateMatingVel, bool has_addRotateMatingVel,
	double addRotateMatingAcc, bool has_addRotateMatingAcc,
	double maxVelForceDir, bool has_maxVelForceDir,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_contactAxis) {
			std::vector<int> contactAxis(contactAxis_ptr, contactAxis_ptr + contactAxis_len);
			input_params["contactAxis"] = contactAxis;
		}
		if (has_contactForce) input_params["contactForce"] = contactForce;
		if (has_matingAxis) {
			std::vector<int> matingAxis(matingAxis_ptr, matingAxis_ptr + matingAxis_len);
			input_params["matingAxis"] = matingAxis;
		}
		if (has_slideMatingRange) input_params["slideMatingRange"] = slideMatingRange;
		if (has_slideMatingVel) input_params["slideMatingVel"] = slideMatingVel;
		if (has_slideMatingAcc) input_params["slideMatingAcc"] = slideMatingAcc;
		if (has_rotateMatingRange) input_params["rotateMatingRange"] = rotateMatingRange;
		if (has_rotateMatingVel) input_params["rotateMatingVel"] = rotateMatingVel;
		if (has_rotateMatingAcc) input_params["rotateMatingAcc"] = rotateMatingAcc;
		if (has_matingTimes) input_params["matingTimes"] = matingTimes;
		if (has_maxContactDis) input_params["maxContactDis"] = maxContactDis;
		if (has_safetyForce) input_params["safetyForce"] = safetyForce;
		if (has_addMatingAxis) {
			std::vector<int> addMatingAxis(addMatingAxis_ptr, addMatingAxis_ptr + addMatingAxis_len);
			input_params["addMatingAxis"] = addMatingAxis;
		}
		if (has_addSlideMatingRange) input_params["addSlideMatingRange"] = addSlideMatingRange;
		if (has_addSlideMatingVel) input_params["addSlideMatingVel"] = addSlideMatingVel;
		if (has_addSlideMatingAcc) input_params["addSlideMatingAcc"] = addSlideMatingAcc;
		if (has_addRotateMatingRange) input_params["addRotateMatingRange"] = addRotateMatingRange;
		if (has_addRotateMatingVel) input_params["addRotateMatingVel"] = addRotateMatingVel;
		if (has_addRotateMatingAcc) input_params["addRotateMatingAcc"] = addRotateMatingAcc;
		if (has_maxVelForceDir) input_params["maxVelForceDir"] = maxVelForceDir;
		robot->ExecutePrimitive("Mate", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void FastenScrew(flexiv::rdk::Robot* robot,
	const char* insertDir, bool has_insertDir,
	double maxInsertVel, bool has_maxInsertVel,
	double insertForce, bool has_insertForce,
	double stiffScale, bool has_stiffScale,
	const char* diScrewInHole, bool has_diScrewInHole,
	const char* diFastenFinish, bool has_diFastenFinish,
	const char* diScrewJam, bool has_diScrewJam,
	bool block_until_started,
	FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		if (has_insertDir) input_params["insertDir"] = std::string(insertDir);
		if (has_maxInsertVel) input_params["maxInsertVel"] = maxInsertVel;
		if (has_insertForce) input_params["insertForce"] = insertForce;
		if (has_stiffScale) input_params["stiffScale"] = stiffScale;
		if (has_diScrewInHole) input_params["diScrewInHole"] = std::string(diScrewInHole);
		if (has_diFastenFinish) input_params["diFastenFinish"] = std::string(diFastenFinish);
		if (has_diScrewJam) input_params["diScrewJam"] = std::string(diScrewJam);
		robot->ExecutePrimitive("FastenScrew", input_params, {}, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void zero_ft_sensor(flexiv::rdk::Robot* robot, double dataCollectTime, bool enableStaticCheck,
	bool calibExtraPayload, bool block_until_started, FlexivError* error) {
	try {
		std::map<std::string, flexiv::rdk::FlexivDataTypes> input_params;
		input_params["dataCollectTime"] = dataCollectTime;
		input_params["enableStaticCheck"] = enableStaticCheck;
		input_params["calibExtraPayload"] = calibExtraPayload;
		robot->ExecutePrimitive("ZeroFTSensor", input_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void stream_joint_torque(flexiv::rdk::Robot* robot,
	double* torques_ptr,
	int torques_len,
	bool enable_gravity_comp,
	bool enable_soft_limits,
	FlexivError* error) {
	try {
		std::vector<double> torques(torques_ptr, torques_ptr + torques_len);
		robot->StreamJointTorque(torques, enable_gravity_comp, enable_soft_limits);
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void stream_joint_position(flexiv::rdk::Robot* robot,
	double* positions_ptr, int positions_len,
	double* velocities_ptr, int velocities_len,
	double* accelerations_ptr, int accelerations_len,
	FlexivError* error) {
	try {
		std::vector<double> positions(positions_ptr, positions_ptr + positions_len);
		std::vector<double> velocities(velocities_ptr, velocities_ptr + velocities_len);
		std::vector<double> accelerations(accelerations_ptr, accelerations_ptr + accelerations_len);
		robot->StreamJointPosition(positions, velocities, accelerations);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void send_joint_position(flexiv::rdk::Robot* robot,
	double* positions_ptr, int positions_len,
	double* velocities_ptr, int velocities_len,
	double* accelerations_ptr, int accelerations_len,
	double* max_vel_ptr, int max_vel_len,
	double* max_acc_ptr, int max_acc_len,
	FlexivError* error) {
	try {
		std::vector<double> positions(positions_ptr, positions_ptr + positions_len);
		std::vector<double> velocities(velocities_ptr, velocities_ptr + velocities_len);
		std::vector<double> accelerations(accelerations_ptr, accelerations_ptr + accelerations_len);
		std::vector<double> max_vel(max_vel_ptr, max_vel_ptr + max_vel_len);
		std::vector<double> max_acc(max_acc_ptr, max_acc_ptr + max_acc_len);
		robot->SendJointPosition(positions, velocities, accelerations, max_vel, max_acc);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_joint_impedance(flexiv::rdk::Robot* robot,
	double* K_q_ptr, int K_q_len,
	double* zq_ptr, int zq_len,
	FlexivError* error) {
	try {
		std::vector<double> K_q(K_q_ptr, K_q_ptr + K_q_len);
		std::vector<double> Z_q(zq_ptr, zq_ptr + zq_len);
		robot->SetJointImpedance(K_q, Z_q);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void stream_cartesian_motion_force(flexiv::rdk::Robot* robot,
	double* pose_ptr, int pose_len,
	double* wrench_ptr, int wrench_len,
	double* velocity_ptr, int velocity_len,
	double* acceleration_ptr, int acceleration_len,
	FlexivError* error) {
	try {
		std::array<double, flexiv::rdk::kPoseSize> pose;
		for (int i = 0; i < flexiv::rdk::kPoseSize; ++i) {
			pose[i] = pose_ptr[i];
		}
		std::array<double, flexiv::rdk::kCartDoF> wrench;
		std::array<double, flexiv::rdk::kCartDoF> velocity;
		std::array<double, flexiv::rdk::kCartDoF> acceleration;
		for (int i = 0; i < flexiv::rdk::kCartDoF; ++i) {
			if (wrench_len != 0) wrench[i] = wrench_ptr[i];
			if (velocity_len != 0) velocity[i] = velocity_ptr[i];
			if (acceleration_len != 0) acceleration[i] = acceleration_ptr[i];
		}
		robot->StreamCartesianMotionForce(pose, wrench, velocity, acceleration);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void send_cartesian_motion_force(flexiv::rdk::Robot* robot,
	double* pose_ptr, int pose_len,
	double* wrench_ptr, int wrench_len,
	double max_linear_vel,
	double max_angular_vel,
	double max_linear_acc,
	double max_angular_acc,
	FlexivError* error) {
	try {
		std::array<double, flexiv::rdk::kPoseSize> pose;
		for (int i = 0; i < flexiv::rdk::kPoseSize; ++i) {
			pose[i] = pose_ptr[i];
		}
		std::array<double, flexiv::rdk::kCartDoF> wrench;
		for (int i = 0; i < flexiv::rdk::kCartDoF; ++i) {
			if (wrench_len != 0) wrench[i] = wrench_ptr[i];
		}
		robot->SendCartesianMotionForce(pose, wrench, max_linear_vel,
			max_angular_vel, max_linear_acc, max_angular_acc);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_cartesian_impedance(flexiv::rdk::Robot* robot,
	double* K_x_ptr, int K_x_len,
	double* zx_ptr, int zx_len,
	FlexivError* error) {
	try {
		std::array<double, flexiv::rdk::kCartDoF> K_x;
		std::array<double, flexiv::rdk::kCartDoF> Z_x;
		for (int i = 0; i < flexiv::rdk::kCartDoF; ++i) {
			K_x[i] = K_x_ptr[i];
			Z_x[i] = zx_ptr[i];
		}
		robot->SetCartesianImpedance(K_x, Z_x);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_max_contact_wrench(flexiv::rdk::Robot* robot,
	double* max_wrench_ptr, int max_wrench_len,
	FlexivError* error) {
	try {
		std::array<double, flexiv::rdk::kCartDoF> max_wrench;
		for (int i = 0; i < flexiv::rdk::kCartDoF; ++i) {
			max_wrench[i] = max_wrench_ptr[i];
		}
		robot->SetMaxContactWrench(max_wrench);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_null_space_posture(flexiv::rdk::Robot* robot,
	double* ref_position_ptr, int ref_position_len,
	FlexivError* error) {
	try {
		std::vector<double> ref_positions(ref_position_ptr, ref_position_ptr + ref_position_len);
		robot->SetNullSpacePosture(ref_positions);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_null_space_objectives(flexiv::rdk::Robot* robot,
	double linear_manipulability,
	double angular_manipulability,
	double ref_positions_tracking,
	FlexivError* error) {
	try {
		robot->SetNullSpaceObjectives(linear_manipulability, angular_manipulability, ref_positions_tracking);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_force_control_axis(flexiv::rdk::Robot* robot,
	int* enabled_axes_ptr, int enabled_axes_len,
	double* max_linear_vel_ptr, int max_linear_vel_len,
	FlexivError* error) {
	try {
		std::array<bool, flexiv::rdk::kCartDoF> enabled_axes;
		for (int i = 0; i < flexiv::rdk::kCartDoF; ++i) {
			enabled_axes[i] = (enabled_axes_ptr[i] == 0 ? false : true);
		}
		const int cnt = flexiv::rdk::kCartDoF / 2;
		std::array<double, cnt> max_linear_vel;
		for (int i = 0; i < cnt; ++i) {
			max_linear_vel[i] = max_linear_vel_ptr[i];
		}
		robot->SetForceControlAxis(enabled_axes, max_linear_vel);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_force_control_frame(flexiv::rdk::Robot* robot,
	const char* root_coord,
	double* T_in_root_ptr, int T_in_root_len,
	FlexivError* error) {
	try {
		flexiv::rdk::CoordType coord_type = flexiv::rdk::CoordType::WORLD;
		std::string ct(root_coord);
		if (ct == "WORLD") coord_type = flexiv::rdk::CoordType::WORLD;
		if (ct == "TCP") coord_type = flexiv::rdk::CoordType::TCP;
		std::array<double, flexiv::rdk::kPoseSize> T_in_root;
		for (int i = 0; i < flexiv::rdk::kPoseSize; ++i) {
			T_in_root[i] = T_in_root_ptr[i];
		}
		robot->SetForceControlFrame(coord_type, T_in_root);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void set_passive_force_control(flexiv::rdk::Robot* robot,
	bool is_enable, FlexivError* error) {
	try {
		robot->SetPassiveForceControl(is_enable);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetDigitalOutput(flexiv::rdk::Robot* robot, int idx, bool value, FlexivError* error) {
	try {
		unsigned int port = idx;
		std::vector<unsigned int> port_idx{ port };
		std::vector<bool> values{ value };
		robot->SetDigitalOutputs(port_idx, values);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API bool IsDigitalInputHigh(flexiv::rdk::Robot* robot, int idx, FlexivError* error) {
	try {
		std::array<bool, flexiv::rdk::kIOPorts> value = robot->digital_inputs();
		error->error_code = 0;
		return value[idx];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void execute_plan(flexiv::rdk::Robot* robot, const char* plan_name,
	bool continue_exec, bool block_until_started, FlexivError* error) {
	try {
		robot->ExecutePlan(plan_name, continue_exec, block_until_started);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void pause_plan(flexiv::rdk::Robot* robot, bool pause, FlexivError* error) {
	try {
		robot->PausePlan(pause);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void plan_list(flexiv::rdk::Robot* robot, Pointer* ptr, int* count, FlexivError* error) {
	try {
		std::vector<std::string> plan_vec = robot->plan_list();
		error->error_code = 0;
		*count = plan_vec.size();
		char** plans = new char* [*count];
		for (int i = 0; i < plan_vec.size(); ++i) {
			plans[i] = new char[plan_vec[i].length() + 1];
			std::strcpy(plans[i], plan_vec[i].c_str());
		}
		// std::cout << "C++    plans: " << std::hex << plans << std::endl;
		// std::cout << "C++ plans[0]: " << std::hex << (*plans) << std::endl;  // 指向第一个字符串的指针
		snprintf(ptr->ptr, sizeof(ptr->ptr), "%p", (void*)plans);
		// std::cout << "Address  ptr: " << ptr->ptr << std::endl;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void free_plan_list(Pointer* ptr_val, int count, FlexivError* error) {
	uintptr_t address = std::stoull(ptr_val->ptr, nullptr, 16);
	void* ptr = reinterpret_cast<void*>(address);
	// std::cout << "free: " << ptr << std::endl;
	char** pla = reinterpret_cast<char**>(ptr);
	try {
		for (int i = 0; i < count; ++i) {
			delete[] pla[i];
		}
		delete[] pla;
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

// Flexiv tool operation
EXPORT_API BSTR get_current_tool_name(flexiv::rdk::Tool* tool, FlexivError* error) {
	try {
		std::string curr_name = tool->name();
		error->error_code = 0;
		return _com_util::ConvertStringToBSTR(curr_name.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return SysAllocString(L"");
}

EXPORT_API void tool_list(flexiv::rdk::Tool* tool, Pointer* ptr, int& count, FlexivError* error) {
	try {
		std::vector<std::string> names = tool->list();
		error->error_code = 0;
		count = names.size();
		char** name_lst = new char* [count];
		for (int i = 0; i < names.size(); ++i) {
			name_lst[i] = new char[names[i].length() + 1];
			std::strcpy(name_lst[i], names[i].c_str());
		}
		snprintf(ptr->ptr, sizeof(ptr->ptr), "%p", (void*)name_lst);
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API bool has_tool(flexiv::rdk::Tool* tool, const char* tool_name, FlexivError* error) {
	try {
		bool flag = tool->exist(tool_name);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void switch_tool(flexiv::rdk::Tool* tool, const char* tool_name, FlexivError* error) {
	try {
		tool->Switch(tool_name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void remove_tool(flexiv::rdk::Tool* tool, const char* tool_name, FlexivError* error) {
	try {
		tool->Remove(tool_name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void get_current_tool_params(flexiv::rdk::Tool* tool, W_ToolParams* tool_params, FlexivError* error) {
	try {
		const flexiv::rdk::ToolParams& params = tool->params();
		error->error_code = 0;
		tool_params->mass = params.mass;
		tool_params->CoM_x = params.CoM[0];
		tool_params->CoM_y = params.CoM[1];
		tool_params->CoM_z = params.CoM[2];
		tool_params->Ixx = params.inertia[0];
		tool_params->Iyy = params.inertia[1];
		tool_params->Izz = params.inertia[2];
		tool_params->Ixy = params.inertia[3];
		tool_params->Ixz = params.inertia[4];
		tool_params->Iyz = params.inertia[5];
		tool_params->tcp_x = params.tcp_location[0];
		tool_params->tcp_y = params.tcp_location[0];
		tool_params->tcp_z = params.tcp_location[0];
		tool_params->tcp_qw = params.tcp_location[0];
		tool_params->tcp_qx = params.tcp_location[0];
		tool_params->tcp_qy = params.tcp_location[0];
		tool_params->tcp_qz = params.tcp_location[0];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void get_tool_params(flexiv::rdk::Tool* tool, const char* tool_name, W_ToolParams* tool_params, FlexivError* error) {
	try {
		const flexiv::rdk::ToolParams& params = tool->params(tool_name);
		error->error_code = 0;
		tool_params->mass = params.mass;
		tool_params->CoM_x = params.CoM[0];
		tool_params->CoM_y = params.CoM[1];
		tool_params->CoM_z = params.CoM[2];
		tool_params->Ixx = params.inertia[0];
		tool_params->Iyy = params.inertia[1];
		tool_params->Izz = params.inertia[2];
		tool_params->Ixy = params.inertia[3];
		tool_params->Ixz = params.inertia[4];
		tool_params->Iyz = params.inertia[5];
		tool_params->tcp_x = params.tcp_location[0];
		tool_params->tcp_y = params.tcp_location[0];
		tool_params->tcp_z = params.tcp_location[0];
		tool_params->tcp_qw = params.tcp_location[0];
		tool_params->tcp_qx = params.tcp_location[0];
		tool_params->tcp_qy = params.tcp_location[0];
		tool_params->tcp_qz = params.tcp_location[0];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void add_tool(flexiv::rdk::Tool* tool, const char* tool_name, W_ToolParams* tp, FlexivError* error) {
	try {
		std::string new_tool_name(tool_name);
		flexiv::rdk::ToolParams new_tool_params;
		new_tool_params.mass = tp->mass;
		new_tool_params.CoM = { tp->CoM_x, tp->CoM_y, tp->CoM_z };
		new_tool_params.inertia = { tp->Ixx, tp->Iyy, tp->Izz, tp->Ixy, tp->Ixz, tp->Iyz };
		new_tool_params.tcp_location = { tp->tcp_x, tp->tcp_y, tp->tcp_z, tp->tcp_qw, tp->tcp_qx, tp->tcp_qy, tp->tcp_qz };
		tool->Add(new_tool_name, new_tool_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void update_tool(flexiv::rdk::Tool* tool, const char* tool_name, W_ToolParams* tp, FlexivError* error) {
	try {
		std::string new_tool_name(tool_name);
		flexiv::rdk::ToolParams new_tool_params;
		new_tool_params.mass = tp->mass;
		new_tool_params.CoM = { tp->CoM_x, tp->CoM_y, tp->CoM_z };
		new_tool_params.inertia = { tp->Ixx, tp->Iyy, tp->Izz, tp->Ixy, tp->Ixz, tp->Iyz };
		new_tool_params.tcp_location = { tp->tcp_x, tp->tcp_y, tp->tcp_z, tp->tcp_qw, tp->tcp_qx, tp->tcp_qy, tp->tcp_qz };
		tool->Update(new_tool_name, new_tool_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

// Flexiv Work Coord
EXPORT_API void work_list(flexiv::rdk::WorkCoord* work, Pointer* ptr, int& count, FlexivError* error) {
	try {
		std::vector<std::string> names = work->list();
		error->error_code = 0;
		count = names.size();
		char** name_lst = new char* [count];
		for (int i = 0; i < names.size(); ++i) {
			name_lst[i] = new char[names[i].length() + 1];
			std::strcpy(name_lst[i], names[i].c_str());
		}
		snprintf(ptr->ptr, sizeof(ptr->ptr), "%p", (void*)name_lst);
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API bool has_work_coord(flexiv::rdk::WorkCoord* work_ptr, const char* work_coord_name, FlexivError* error) {
	try {
		bool flag = work_ptr->exist(work_coord_name);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}


EXPORT_API void get_work_coord(flexiv::rdk::WorkCoord* work_ptr, const char* work_coord_name,
	double& x, double& y, double& z, double& qw, double& qx, double& qy, double& qz, FlexivError* error) {
	try {
		std::array<double, 7> w_c = work_ptr->pose(work_coord_name);
		error->error_code = 0;
		x = w_c[0];
		y = w_c[1];
		z = w_c[2];
		qw = w_c[3];
		qx = w_c[4];
		qy = w_c[5];
		qz = w_c[6];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void add_work_coord(flexiv::rdk::WorkCoord* work_ptr, const char* work_coord_name,
	double x, double y, double z, double qw, double qx, double qy, double qz, FlexivError* error) {
	try {
		std::array<double, 7> w_c = { };
		w_c[0] = x;
		w_c[1] = y;
		w_c[2] = z;
		w_c[3] = qw;
		w_c[4] = qx;
		w_c[5] = qy;
		w_c[6] = qz;
		work_ptr->Add(work_coord_name, w_c);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void update_work_coord(flexiv::rdk::WorkCoord* work_ptr, const char* work_coord_name,
	double x, double y, double z, double qw, double qx, double qy, double qz, FlexivError* error) {
	try {
		std::array<double, 7> w_c = { };
		w_c[0] = x;
		w_c[1] = y;
		w_c[2] = z;
		w_c[3] = qw;
		w_c[4] = qx;
		w_c[5] = qy;
		w_c[6] = qz;
		work_ptr->Update(work_coord_name, w_c);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void remove_work_coord(flexiv::rdk::WorkCoord* work_ptr,
	const char* work_coord_name, FlexivError* error) {
	try {
		work_ptr->Remove(work_coord_name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}