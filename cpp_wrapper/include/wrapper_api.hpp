#ifndef WRAPPER_API_HPP_
#define WRAPPER_API_HPP_
#ifdef _WIN32
#define EXPORT_API extern "C" __declspec(dllexport)
#else
#define EXPORT_API extern "C"
#endif
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>
#include <thread>
#include <algorithm>

#include <spdlog\spdlog.h>
#include <flexiv\rdk\data.hpp>
#include <flexiv\rdk\robot.hpp>
#include <flexiv\rdk\utility.hpp>
#include <flexiv\rdk\tool.hpp>
#include <flexiv\rdk\work_coord.hpp>
#include <flexiv\rdk\gripper.hpp>
#include <flexiv\rdk\file_io.hpp>
#include <flexiv\rdk\device.hpp>
#include "json.hpp"
using json = nlohmann::json;
using namespace flexiv::rdk;

struct FlexivError {
	int error_code;
	char error_msg[512];
};

struct WRobotInfo {
	char serial_num[128];
	char software_ver[128];
	char model_name[128];
	char license_type[128];
	int Dof;
	double K_x_nom[kCartDoF];
	double K_q_nom[kSerialJointDoF];
	double q_min[kSerialJointDoF];
	double q_max[kSerialJointDoF];
	double dq_max[kSerialJointDoF];
	double tau_max[kSerialJointDoF];
};

struct WRobotState {
	double q[kSerialJointDoF];
	double theta[kSerialJointDoF];
	double dq[kSerialJointDoF];
	double dtheta[kSerialJointDoF];
	double tau[kSerialJointDoF];
	double tau_des[kSerialJointDoF];
	double tau_dot[kSerialJointDoF];
	double tau_ext[kSerialJointDoF];
	double q_e[kMaxExtAxes];
	double dq_e[kMaxExtAxes];
	double tau_e[kMaxExtAxes];
	double tcp_pose[kPoseSize];
	double tcp_pose_des[kPoseSize];
	double tcp_vel[kCartDoF];
	double flange_pose[kPoseSize];
	double ft_sensor_raw[kCartDoF];
	double ext_wrench_in_tcp[kCartDoF];
	double ext_wrench_in_world[kCartDoF];
	double ext_wrench_in_tcp_raw[kCartDoF];
	double ext_wrench_in_world_raw[kCartDoF];
};

struct WPlanInfo {
	char pt_name[128];
	char node_name[128];
	char node_path[512];
	char node_path_time_period[128];
	char node_path_number[128];
	char assigned_plan_name[128];
	double velocity_scale;
	int waiting_for_step;
};

struct WToolParams {
	double Mass;
	double CoM[3];
	double Intertia[6];
	double TcpLocation[kPoseSize];
};

struct WGripperStates {
	double width;
	double force;
	double max_width;
};

void CopyExceptionMsg(const std::exception& e, FlexivError* error) {
	std::strncpy(error->error_msg, e.what(), sizeof(error->error_msg) - 1);
	error->error_msg[sizeof(error->error_msg) - 1] = '\0';
}

char* CopyInputString(const char* input) {
	if (!input) return nullptr;
	size_t len = std::strlen(input);
	char* buf = new char[len + 1];
	std::strcpy(buf, input);
	return buf;
}

void CopyMsgSrc2Dst(const char* src, char* dst, size_t dstSize) {
	std::strncpy(dst, src, dstSize - 1);
	dst[dstSize - 1] = '\0';
}

namespace flexiv::rdk {
	inline void from_json(const json& j, JPos& pos) {
		j.at("q").get_to(pos.q);
		j.at("q_e").get_to(pos.q_e);
	}

	inline void to_json(json& j, const JPos& pos) {
		j = json{
			{"q", pos.q},
			{"q_e", pos.q_e}
		};
	}

	inline void from_json(const json& j, Coord& coord) {
		j.at("position").get_to(coord.position);
		j.at("orientation").get_to(coord.orientation);
		j.at("ref_frame").get_to(coord.ref_frame);
		j.at("ref_q").get_to(coord.ref_q);
		j.at("ref_q_e").get_to(coord.ref_q_e);
	}

	inline void to_json(json& j, const Coord& coord) {
		j = json{
			{"position", coord.position},
			{"orientation", coord.orientation},
			{"ref_frame", coord.ref_frame},
			{"ref_q", coord.ref_q},
			{"ref_q_e", coord.ref_q_e}
		};
	}
}

using ParserFunc = std::function<FlexivDataTypes(const nlohmann::json&)>;
const std::unordered_map<std::string, ParserFunc> kTypeParsers = {
	{ "Int",          [](const json& j) { return j.get<int>(); } },
	{ "Double",       [](const json& j) { return j.get<double>(); } },
	{ "String",       [](const json& j) { return j.get<std::string>(); } },
	{ "JPos",         [](const json& j) { return j.get<JPos>(); } },
	{ "Coord",        [](const json& j) { return j.get<Coord>(); } },
	{ "VectorInt",    [](const json& j) { return j.get<std::vector<int>>(); } },
	{ "VectorDouble", [](const json& j) { return j.get<std::vector<double>>(); } },
	{ "VectorString", [](const json& j) { return j.get<std::vector<std::string>>(); } },
	{ "VectorJPos",   [](const json& j) { return j.get<std::vector<JPos>>(); } },
	{ "VectorCoord",  [](const json& j) { return j.get<std::vector<Coord>>(); } }
};

FlexivDataTypes parseValue(const std::string& type, const json& value) {
	auto iter = kTypeParsers.find(type);
	if (iter != kTypeParsers.end()) {
		return iter->second(value);
	}
	else {
		throw std::runtime_error("Unsupported type: " + type);
	}
}

std::map<std::string, FlexivDataTypes> parseJsonToParams(const char* json_str) {
	json json_obj = json::parse(json_str);
	std::map<std::string, FlexivDataTypes> params;
	for (auto& [key, val] : json_obj.items()) {
		std::string type = val.at("type").get<std::string>();
		FlexivDataTypes data = parseValue(type, val.at("value"));
		params[key] = std::move(data);
	}
	return params;
}

std::string getTypeName(const FlexivDataTypes& value) {
	return std::visit([](auto&& val) -> std::string {
		using T = std::decay_t<decltype(val)>;
		if constexpr (std::is_same_v<T, int>) return "Int";
		else if constexpr (std::is_same_v<T, double>) return "Double";
		else if constexpr (std::is_same_v<T, std::string>) return "String";
		else if constexpr (std::is_same_v<T, JPos>) return "JPos";
		else if constexpr (std::is_same_v<T, Coord>) return "Coord";
		else if constexpr (std::is_same_v<T, std::vector<int>>) return "VectorInt";
		else if constexpr (std::is_same_v<T, std::vector<double>>) return "VectorDouble";
		else if constexpr (std::is_same_v<T, std::vector<std::string>>) return "VectorString";
		else if constexpr (std::is_same_v<T, std::vector<JPos>>) return "VectorJPos";
		else if constexpr (std::is_same_v<T, std::vector<Coord>>) return "VectorCoord";
		else return "Unknown"; }, value);
}

std::string serializeParams(const std::map<std::string, FlexivDataTypes>& params) {
	json j;
	std::string type_name;
	for (const auto& iter : params) {
		type_name = getTypeName(iter.second);
		std::visit([&](const auto& v) {
			j[iter.first] = {
				{"type", type_name},
				{"value", v}
			}; }, iter.second);
	}
	return j.dump();
}

#endif  // !wrapper_api.hpp