#include "wrapper_api.hpp"

EXPORT_API void FreeString(const char* ptr) {
	delete[] ptr;
}

EXPORT_API void SpdlogInfo(const char* msgs) {
	spdlog::info(std::string(msgs));
}

EXPORT_API void SpdlogWarn(const char* msgs) {
	spdlog::warn(std::string(msgs));
}

EXPORT_API void SpdlogError(const char* msgs) {
	spdlog::error(std::string(msgs));
}

EXPORT_API Robot* CreateFlexivRobot(const char* robot_sn,
	const char** interfaces, int interface_count, int verbose, FlexivError* error) {
	std::vector<std::string> white_list;
	for (int i = 0; i < interface_count; ++i) {
		white_list.push_back(interfaces[i]);
	}
	try {
		flexiv::rdk::Robot* robot = new flexiv::rdk::Robot(robot_sn, white_list, verbose);
		error->error_code = 0;
		return robot;
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

//========================================= ACCESSORS ==========================================
EXPORT_API int IsConnected(Robot* robot) {
	return robot->connected();
}

EXPORT_API void GetInfo(Robot* robot, WRobotInfo* info) {
	const auto& r_info = robot->info();
	CopyMsgSrc2Dst(r_info.serial_num.c_str(), info->serial_num, sizeof(info->serial_num));
	CopyMsgSrc2Dst(r_info.software_ver.c_str(), info->software_ver, sizeof(info->software_ver));
	CopyMsgSrc2Dst(r_info.model_name.c_str(), info->model_name, sizeof(info->model_name));
	CopyMsgSrc2Dst(r_info.license_type.c_str(), info->license_type, sizeof(info->license_type));
	info->Dof = r_info.DoF;
	for (int i = 0; i < kCartDoF; ++i) {
		info->K_x_nom[i] = r_info.K_x_nom[i];
	}
	for (int i = 0; i < kSerialJointDoF; ++i) {
		info->K_q_nom[i] = r_info.K_q_nom[i];
		info->q_min[i] = r_info.q_min[i];
		info->q_max[i] = r_info.q_max[i];
		info->dq_max[i] = r_info.dq_max[i];
		info->tau_max[i] = r_info.tau_max[i];
	}
}

EXPORT_API int GetMode(Robot* robot) {
	switch (robot->mode()) {
	case Mode::UNKNOWN:                    return 0;
	case Mode::IDLE:                       return 1;
	case Mode::RT_JOINT_TORQUE:            return 2;
	case Mode::RT_JOINT_IMPEDANCE:         return 3;
	case Mode::NRT_JOINT_IMPEDANCE:        return 4;
	case Mode::RT_JOINT_POSITION:          return 5;
	case Mode::NRT_JOINT_POSITION:         return 6;
	case Mode::NRT_PLAN_EXECUTION:         return 7;
	case Mode::NRT_PRIMITIVE_EXECUTION:    return 8;
	case Mode::RT_CARTESIAN_MOTION_FORCE:  return 9;
	case Mode::NRT_CARTESIAN_MOTION_FORCE: return 10;
	case Mode::NRT_SUPER_PRIMITIVE:        return 11;
	case Mode::MODES_CNT:                  return 12;
	default:                               return 0;
	}
}

EXPORT_API void GetStates(Robot* robot, WRobotState* robot_state) {
	const RobotStates& states = robot->states();
	for (int i = 0; i < kSerialJointDoF; ++i) {
		robot_state->q[i] = states.q[i];
		robot_state->theta[i] = states.theta[i];
		robot_state->dq[i] = states.dq[i];
		robot_state->dtheta[i] = states.dtheta[i];
		robot_state->tau[i] = states.tau[i];
		robot_state->tau_des[i] = states.tau_des[i];
		robot_state->tau_dot[i] = states.tau_dot[i];
		robot_state->tau_ext[i] = states.tau_ext[i];
	}
	for (int i = 0; i < kPoseSize; ++i) {
		robot_state->tcp_pose[i] = states.tcp_pose[i];
		robot_state->flange_pose[i] = states.flange_pose[i];
	}
	for (int i = 0; i < kCartDoF; ++i) {
		robot_state->tcp_vel[i] = states.tcp_vel[i];
		robot_state->ft_sensor_raw[i] = states.ft_sensor_raw[i];
		robot_state->ext_wrench_in_tcp[i] = states.ext_wrench_in_tcp[i];
		robot_state->ext_wrench_in_world[i] = states.ext_wrench_in_world[i];
		robot_state->ext_wrench_in_tcp_raw[i] = states.ext_wrench_in_tcp_raw[i];
		robot_state->ext_wrench_in_world_raw[i] = states.ext_wrench_in_world_raw[i];
	}
}

EXPORT_API int IsStopped(Robot* robot) {
	return robot->stopped();
}

EXPORT_API int IsOperational(Robot* robot) {
	return robot->operational();
}

EXPORT_API int GetOperationalStatus(Robot* robot) {
	switch (robot->operational_status()) {
	case OperationalStatus::UNKNOWN:            return 0;
	case OperationalStatus::READY:              return 1;
	case OperationalStatus::BOOTING:            return 2;
	case OperationalStatus::ESTOP_NOT_RELEASED: return 3;
	case OperationalStatus::NOT_ENABLED:        return 4;
	case OperationalStatus::RELEASING_BRAKE:    return 5;
	case OperationalStatus::MINOR_FAULT:        return 6;
	case OperationalStatus::CRITICAL_FAULT:     return 7;
	case OperationalStatus::IN_REDUCED_STATE:   return 8;
	case OperationalStatus::IN_RECOVERY_STATE:  return 9;
	case OperationalStatus::IN_MANUAL_MODE:     return 10;
	case OperationalStatus::IN_AUTO_MODE:       return 11;
	default:                                    return 0;
	}
}

EXPORT_API int IsBusy(Robot* robot) {
	return robot->busy();
}

EXPORT_API int IsFault(Robot* robot) {
	return robot->fault();
}

EXPORT_API int IsReduced(Robot* robot) {
	return robot->reduced();
}

EXPORT_API int IsRecovery(Robot* robot) {
	return robot->recovery();
}

EXPORT_API int IsEstopReleased(Robot* robot) {
	return robot->estop_released();
}

EXPORT_API int IsEnablingButtonReleased(Robot* robot) {
	return robot->enabling_button_pressed();
}

EXPORT_API char* GetEventLog(Robot* robot) {
	nlohmann::json j = robot->event_log();
	std::string str = j.dump();
	return CopyInputString(str.c_str());
}

//======================================= SYSTEM CONTROL =======================================
EXPORT_API void Enable(Robot* robot, FlexivError* error) {
	try {
		robot->Enable();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Brake(Robot* robot, int engage, FlexivError* error) {
	try {
		robot->Brake(engage);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SwitchMode(Robot* robot, int mode, FlexivError* error) {
	static const std::array<flexiv::rdk::Mode, 13> modeMap = {
	Mode::UNKNOWN, Mode::IDLE, Mode::RT_JOINT_TORQUE, Mode::RT_JOINT_IMPEDANCE,
	Mode::NRT_JOINT_IMPEDANCE, Mode::RT_JOINT_POSITION, Mode::NRT_JOINT_POSITION,
	Mode::NRT_PLAN_EXECUTION, Mode::NRT_PRIMITIVE_EXECUTION, Mode::RT_CARTESIAN_MOTION_FORCE,
	Mode::NRT_CARTESIAN_MOTION_FORCE, Mode::NRT_SUPER_PRIMITIVE, Mode::MODES_CNT };
	if (mode < 0 || mode >= static_cast<int>(modeMap.size())) {
		error->error_code = 1;
		CopyExceptionMsg(std::runtime_error("Invalid robot mode index"), error);
		return;
	}
	try {
		robot->SwitchMode(modeMap[mode]);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Stop(Robot* robot, FlexivError* error) {
	try {
		robot->Stop();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API int ClearFault(Robot* robot, int timeout_sec, FlexivError* error) {
	try {
		int flag = robot->ClearFault(timeout_sec);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void RunAutoRecovery(Robot* robot, FlexivError* error) {
	try {
		robot->RunAutoRecovery();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetGlobalVariables(Robot* robot,
	const char* globalVars,
	FlexivError* error) {
	try {
		auto vars = parseJsonToParams(globalVars);
		robot->SetGlobalVariables(vars);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API char* GetGlobalVariables(Robot* robot, FlexivError* error) {
	try {
		const auto& globalVars = robot->global_variables();
		std::string json_str = serializeParams(globalVars);
		error->error_code = 0;
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return nullptr;
	}
}

EXPORT_API void LockExternalAxes(Robot* robot, int toggle, FlexivError* error) {
	try {
		robot->LockExternalAxes(toggle);
		error->error_code = 0;
	} 
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//======================================= PLAN EXECUTION =======================================
EXPORT_API void ExecutePlanByIdx(Robot* robot, int idx, int continueExec,
	int blockUntilStarted, FlexivError* error) {
	try {
		robot->ExecutePlan(idx, continueExec, blockUntilStarted);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void ExecutePlanByName(Robot* robot, const char* name, int continueExec,
	int blockUntilStarted, FlexivError* error) {
	try {
		robot->ExecutePlan(std::string(name), continueExec, blockUntilStarted);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void PausePlan(Robot* robot, int pause, FlexivError* error) {
	try {
		robot->PausePlan(pause);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API char* GetPlanList(Robot* robot, FlexivError* error) {
	try {
		const auto& plan_list = robot->plan_list();
		error->error_code = 0;
		std::map<std::string, FlexivDataTypes> tmp;
		tmp["plan_list"] = plan_list;
		std::string json_str = serializeParams(tmp);
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void GetPlanInfo(Robot* robot, WPlanInfo* planInfo, FlexivError* error) {
	try {
		const auto& info = robot->plan_info();
		CopyMsgSrc2Dst(info.pt_name.c_str(), planInfo->pt_name, sizeof(planInfo->pt_name));
		CopyMsgSrc2Dst(info.node_name.c_str(), planInfo->node_name, sizeof(planInfo->node_name));
		CopyMsgSrc2Dst(info.node_path.c_str(), planInfo->node_path, sizeof(planInfo->node_path));
		CopyMsgSrc2Dst(info.node_path_time_period.c_str(), planInfo->node_path_time_period, sizeof(planInfo->node_path_time_period));
		CopyMsgSrc2Dst(info.node_path_number.c_str(), planInfo->node_path_number, sizeof(planInfo->node_path_number));
		CopyMsgSrc2Dst(info.assigned_plan_name.c_str(), planInfo->assigned_plan_name, sizeof(planInfo->assigned_plan_name));
		planInfo->velocity_scale = info.velocity_scale;
		planInfo->waiting_for_step = info.waiting_for_step;
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetBreakpointMode(Robot* robot, int IsEnable, FlexivError* error) {
	try {
		robot->SetBreakpointMode(IsEnable);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void StepBreakpoint(Robot* robot, FlexivError* error) {
	try {
		robot->StepBreakpoint();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//==================================== PRIMITIVE EXECUTION =====================================
EXPORT_API void ExecutePrimitive(Robot* robot,
	const char* primitiveName,
	const char* inputParams,
	int blockUntilStarted,
	FlexivError* error) {
	try {
		auto input_params = parseJsonToParams(inputParams);
		robot->ExecutePrimitive(std::string(primitiveName), input_params, blockUntilStarted);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API char* GetPrimitiveStates(Robot* robot, FlexivError* error) {
	try {
		const auto& primitiveStates = robot->primitive_states();
		std::string json_str = serializeParams(primitiveStates);
		error->error_code = 0;
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return nullptr;
	}
}

//==================================== DIRECT JOINT CONTROL ====================================
EXPORT_API void StreamJointTorque(Robot* robot, const double* pos, int posLen,
	int enableGravityComp, int enableSoftLimits, FlexivError* error) {
	try {
		std::vector<double> positions(pos, pos + posLen);
		robot->StreamJointTorque(positions, enableGravityComp, enableSoftLimits);
		error->error_code = 0;
	}
	catch (std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void StreamJointPosition(Robot* robot, const double* pos, int posLen,
	const double* vel, int velLen, const double* acc, int accLen, FlexivError* error) {
	try {
		std::vector<double> positions(pos, pos + posLen);
		std::vector<double> velocities(vel, vel + velLen);
		std::vector<double> accelerations(acc, acc + accLen);
		robot->StreamJointPosition(positions, velocities, accelerations);
		error->error_code = 0;
	}
	catch (std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SendJointPosition(Robot* robot, const double* pos, int posLen, const double* vel,
	int velLen, const double* acc, int accLen, const double* maxVel, int maxVelLen,
	const double* maxAcc, int maxAccLen, FlexivError* error) {
	try {
		std::vector<double> positions(pos, pos + posLen);
		std::vector<double> velocities(vel, vel + velLen);
		std::vector<double> accelerations(acc, acc + accLen);
		std::vector<double> max_vel(maxVel, maxVel + maxVelLen);
		std::vector<double> max_acc(maxAcc, maxAcc + maxAccLen);
		robot->SendJointPosition(positions, velocities, accelerations,
			max_vel, max_acc);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetJointImpedance(Robot* robot, const double* Kq, int KqLen,
	const double* Zq, int ZqLen, FlexivError* error) {
	try {
		std::vector<double> K_q(Kq, Kq + KqLen);
		std::vector<double> Z_q(Zq, Zq + ZqLen);
		robot->SetJointImpedance(K_q, Z_q);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void StreamCartesianMotionForce(Robot* robot, const double* pose, int poseLen,
	const double* wrench, int wrenchLen, const double* velocity, int velocityLen,
	const double* acceleration, int accelerationLen, FlexivError* error) {
	try {
		if (poseLen != kPoseSize)
			throw std::invalid_argument("StreamCartesianMotionForce pose length must be " + 
				std::to_string(kPoseSize));
		if (wrenchLen != kCartDoF)
			throw std::invalid_argument("StreamCartesianMotionForce wrench length must be " +
				std::to_string(kCartDoF));
		if (velocityLen != kCartDoF)
			throw std::invalid_argument("StreamCartesianMotionForce velocity length must be " + 
				std::to_string(kCartDoF));
		if (accelerationLen != kCartDoF)
			throw std::invalid_argument("StreamCartesianMotionForce acceleration length must be " +
				std::to_string(kCartDoF));
		std::array<double, kPoseSize> poseArr;
		std::copy(pose, pose + kPoseSize, poseArr.begin());
		std::array<double, kCartDoF> wrenchArr{};
		std::copy(wrench, wrench + kCartDoF, wrenchArr.begin());
		std::array<double, kCartDoF> velocityArr{};
		std::copy(velocity, velocity + kCartDoF, velocityArr.begin());
		std::array<double, kCartDoF> accelerationArr{};
		std::copy(acceleration, acceleration + kCartDoF, accelerationArr.begin());
		robot->StreamCartesianMotionForce(poseArr, wrenchArr, velocityArr, accelerationArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SendCartesianMotionForce(Robot* robot, const double* pose, int poseLen,
	const double* wrench, int wrenchLen, double maxLinearVel, double maxAngularVel,
	double maxLinearAcc, double maxAngularAcc, FlexivError* error) {
	try {
		if (poseLen != kPoseSize)
			throw std::invalid_argument("SendCartesianMotionForce pose length must be " +
				std::to_string(kPoseSize));
		if (wrenchLen != kCartDoF)
			throw std::invalid_argument("SendCartesianMotionForce wrench length must be " +
				std::to_string(kCartDoF));
		std::array<double, kPoseSize> poseArr;
		std::copy(pose, pose + kPoseSize, poseArr.begin());
		std::array<double, kCartDoF> wrenchArr{};
		std::copy(wrench, wrench + kCartDoF, wrenchArr.begin());
		robot->SendCartesianMotionForce(poseArr, wrenchArr, maxLinearVel, maxAngularVel,
			maxLinearAcc, maxAngularAcc);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetCartesianImpedance(Robot* robot, const double* Kx, int KxLen,
	const double* Zx, int ZxLen, FlexivError* error) {
	try {
		if (KxLen != kCartDoF)
			throw std::invalid_argument("SetCartesianImpedance Kx length must be " +
				std::to_string(kCartDoF));
		if (ZxLen != kCartDoF)
			throw std::invalid_argument("SetCartesianImpedance Zx length must be " +
				std::to_string(kCartDoF));
		std::array<double, kCartDoF> KxArr;
		std::copy(Kx, Kx + kCartDoF, KxArr.begin());
		std::array<double, kCartDoF> ZxArr{};
		std::copy(Zx, Zx + kCartDoF, ZxArr.begin());
		robot->SetCartesianImpedance(KxArr, ZxArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetMaxContactWrench(Robot* robot, const double* maxWrench, int maxWrenchLen, FlexivError* error) {
	try {
		if (maxWrenchLen != kCartDoF)
			throw std::invalid_argument("SetMaxContactWrench maxWrench length must be " +
				std::to_string(kCartDoF));
		std::array<double, kCartDoF> maxWrenchArr;
		std::copy(maxWrench, maxWrench + maxWrenchLen, maxWrenchArr.begin());
		robot->SetMaxContactWrench(maxWrenchArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetNullSpacePosture(Robot* robot, const double* refPositions, int refPositionsLen, FlexivError* error) {
	try {
		std::vector<double> ref_positions(refPositions, refPositions + refPositionsLen);
		robot->SetNullSpacePosture(ref_positions);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}


EXPORT_API void SetNullSpaceObjectives(Robot* robot, double linearManipulability, double angularManipulability,
	double refPositionsTracking, FlexivError* error) {
	try {
		robot->SetNullSpaceObjectives(linearManipulability, angularManipulability, refPositionsTracking);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetForceControlAxis(Robot* robot, const int* enabledAxes, int enabledAxesLen,
	const double* maxLinearVel, int maxLinearVelLen, FlexivError* error) {
	try {
		if (enabledAxesLen != kCartDoF)
			throw std::invalid_argument("SetForceControlAxis enabledAxes length must be " +
				std::to_string(kCartDoF));
		if (maxLinearVelLen != kCartDoF / 2)
			throw std::invalid_argument("SetForceControlAxis maxLinearVel length must be " +
				std::to_string(kCartDoF / 2));
		std::array<bool, kCartDoF> enabledAxesArr;
		for (int i = 0; i < kCartDoF; ++i)
			enabledAxesArr[i] = *(enabledAxes + i);
		std::array<double, kCartDoF / 2> maxLinearVelArr;
		std::copy(maxLinearVel, maxLinearVel + maxLinearVelLen, maxLinearVelArr.begin());
		robot->SetForceControlAxis(enabledAxesArr, maxLinearVelArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetForceControlFrame(Robot* robot, int rootCoord, const double* TInRobot, int TInRobotLen, FlexivError* error) {
	try {
		if (TInRobotLen != kPoseSize)
			throw std::invalid_argument("SetForceControlFrame TInRobot length must be " +
				std::to_string(kPoseSize));
		CoordType coord_type = rootCoord == 0 ? CoordType::WORLD : CoordType::TCP;
		std::array<double, kPoseSize> T_in_root;
		std::copy(TInRobot, TInRobot + TInRobotLen, T_in_root.begin());
		robot->SetForceControlFrame(coord_type, T_in_root);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetPassiveForceControl(Robot* robot, int IsEnabled, FlexivError* error) {
	try {
		robot->SetPassiveForceControl(IsEnabled);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//======================================== IO CONTROL ========================================
EXPORT_API void SetDigitalOutput(Robot* robot, int idx, int value, FlexivError* error) {
	try {
		unsigned int port = idx;
		std::map<unsigned int, bool> outputs;
		outputs[idx] = value;
		robot->SetDigitalOutputs(outputs);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API int GetDigitalOutput(Robot* robot, int idx) {
	return robot->digital_inputs()[idx];
}

//========================================== TOOL ============================================
EXPORT_API Tool* CreateTool(Robot* robot, FlexivError* error) {
	try {
		Tool* tool = new Tool(*robot);
		error->error_code = 0;
		return tool;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteTool(Tool* tool) {
	delete tool;
}

EXPORT_API char* GetToolNames(Tool* tool, FlexivError* error) {
	try {
		const auto& lst = tool->list();
		error->error_code = 0;
		std::map<std::string, FlexivDataTypes> tmp;
		tmp["tool_list"] = lst;
		std::string json_str = serializeParams(tmp);
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API char* GetToolName(Tool* tool, FlexivError* error) {
	try {
		std::string name = tool->name();
		error->error_code = 0;
		return CopyInputString(name.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API int HasTool(Tool* tool, const char* name, FlexivError* error) {
	try {
		int flag = tool->exist(std::string(name));
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return false;
}

EXPORT_API void GetToolParams(Tool* tool, WToolParams* toolParams, FlexivError* error) {
	try {
		const auto& params = tool->params();
		error->error_code = 0;
		toolParams->Mass = params.mass;
		for (int i = 0; i < 3; ++i) toolParams->CoM[i] = params.CoM[i];
		for (int i = 0; i < 6; ++i) toolParams->Intertia[i] = params.inertia[i];
		for (int i = 0; i < kPoseSize; ++i) toolParams->TcpLocation[i] = params.tcp_location[i];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetToolParamsByName(Tool* tool, const char* name, WToolParams* toolParams, FlexivError* error) {
	try {
		std::string tool_name(name);
		const auto& params = tool->params(tool_name);
		error->error_code = 0;
		toolParams->Mass = params.mass;
		for (int i = 0; i < 3; ++i) toolParams->CoM[i] = params.CoM[i];
		for (int i = 0; i < 6; ++i) toolParams->Intertia[i] = params.inertia[i];
		for (int i = 0; i < kPoseSize; ++i) toolParams->TcpLocation[i] = params.tcp_location[i];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void AddNewTool(Tool* tool, const char* name, WToolParams* toolParams, FlexivError* error) {
	try {
		std::string tool_name(name);
		ToolParams tool_params;
		tool_params.mass = toolParams->Mass;
		for (int i = 0; i < 3; ++i) tool_params.CoM[i] = toolParams->CoM[i];
		for (int i = 0; i < 6; ++i) tool_params.inertia[i] = toolParams->Intertia[i];
		for (int i = 0; i < kPoseSize; ++i) tool_params.tcp_location[i] = toolParams->TcpLocation[i];
		tool->Add(tool_name, tool_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SwitchTool(Tool* tool, const char* name, FlexivError* error) {
	try {
		tool->Switch(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void UpdateTool(Tool* tool, const char* name, WToolParams* toolParams, FlexivError* error) {
	try {
		std::string tool_name(name);
		ToolParams tool_params;
		tool_params.mass = toolParams->Mass;
		for (int i = 0; i < 3; ++i) tool_params.CoM[i] = toolParams->CoM[i];
		for (int i = 0; i < 6; ++i) tool_params.inertia[i] = toolParams->Intertia[i];
		for (int i = 0; i < kPoseSize; ++i) tool_params.tcp_location[i] = toolParams->TcpLocation[i];
		tool->Update(tool_name, tool_params);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void RemoveTool(Tool* tool, const char* name, FlexivError* error) {
	try {
		tool->Remove(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void CalibratePayloadParams(Tool* tool, int toolMounted, WToolParams* toolParams, FlexivError* error) {
	try {
		const auto& params = tool->CalibratePayloadParams(toolMounted);
		error->error_code = 0;
		toolParams->Mass = params.mass;
		for (int i = 0; i < 3; ++i) toolParams->CoM[i] = params.CoM[i];
		for (int i = 0; i < 6; ++i) toolParams->Intertia[i] = params.inertia[i];
		for (int i = 0; i < kPoseSize; ++i) toolParams->TcpLocation[i] = params.tcp_location[i];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//======================================= WORK COORD =========================================
EXPORT_API WorkCoord* CreateWorkCoord(Robot* robot, FlexivError* error) {
	try {
		WorkCoord* work_coord = new WorkCoord(*robot);
		error->error_code = 0;
		return work_coord;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteWorkCoord(WorkCoord* work_coord) {
	delete work_coord;
}

EXPORT_API char* GetWorkCoordNames(WorkCoord* work_coord, FlexivError* error) {
	try {
		const auto& lst = work_coord->list();
		error->error_code = 0;
		std::map<std::string, FlexivDataTypes> tmp;
		tmp["work_coord_list"] = lst;
		std::string json_str = serializeParams(tmp);
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API int HasWorkCoord(WorkCoord* work_coord, const char* name, FlexivError* error) {
	try {
		int flag = work_coord->exist(name);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return 0;
}

EXPORT_API void GetWorkCoordPose(WorkCoord* workCoord, const char* name, double& x, double& y,
	double& z, double& qw, double& qx, double& qy, double& qz, FlexivError* error) {
	try {
		const auto& pose = workCoord->pose(name);
		error->error_code = 0;
		x = pose[0];
		y = pose[1];
		z = pose[2];
		qw = pose[3];
		qx = pose[4];
		qy = pose[5];
		qz = pose[6];
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void AddWorkCoord(WorkCoord* workCoord, const char* name, double* pose, int poseLen, FlexivError* error) {
	try {
		if (poseLen != kPoseSize)
			throw std::invalid_argument("AddWorkCoord pose length must be " +
				std::to_string(kPoseSize));
		std::array<double, kPoseSize> poseArr;
		std::copy(pose, pose + poseLen, poseArr.begin());
		workCoord->Add(name, poseArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void UpdateWorkCoord(WorkCoord* workCoord, const char* name, double* pose, int poseLen, FlexivError* error) {
	try {
		if (poseLen != kPoseSize)
			throw std::invalid_argument("UpdateWorkCoord pose length must be " +
				std::to_string(kPoseSize));
		std::array<double, kPoseSize> poseArr;
		std::copy(pose, pose + poseLen, poseArr.begin());
		workCoord->Update(name, poseArr);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void RemoveWorkCoord(WorkCoord* workCoord, const char* name, FlexivError* error) {
	try {
		workCoord->Remove(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//======================================== GRIPPER ===========================================
EXPORT_API Gripper* CreateGripper(Robot* robot, FlexivError* error) {
	try {
		Gripper* gripper = new Gripper(*robot);
		error->error_code = 0;
		return gripper;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteGripper(Gripper* gripper) {
	delete gripper;
}

EXPORT_API void EnableGripper(Gripper* gripper, const char* name, FlexivError* error) {
	try {
		gripper->Enable(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void DisableGripper(Gripper* gripper, FlexivError* error) {
	try {
		gripper->Disable();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Init(Gripper* gripper, FlexivError* error) {
	try {
		gripper->Init();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Grasp(Gripper* gripper, double force, FlexivError* error) {
	try {
		gripper->Grasp(force);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Move(Gripper* gripper, double width, double velocity, double forceLimit, FlexivError* error) {
	try {
		gripper->Move(width, velocity, forceLimit);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void StopGripper(Gripper* gripper) {
	gripper->Stop();
}

EXPORT_API void GetGripperParams(Gripper* gripper, WGripperParams* param) {
	const auto& gp = gripper->params();
	CopyMsgSrc2Dst(gp.name.c_str(), param->name, sizeof(param->name));
	param->min_width = gp.min_width;
	param->max_width = gp.max_width;
	param->min_vel = gp.min_vel;
	param->max_vel = gp.max_vel;
	param->min_force = gp.min_force;
	param->max_force = gp.max_force;
}

EXPORT_API void GetGripperStates(Gripper* gripper, WGripperStates* states) {
	const auto& st = gripper->states();
	states->width = st.width;
	states->force = st.force;
	states->is_moving = st.is_moving;
}

//======================================== FILE IO ===========================================
EXPORT_API FileIO* CreateFileIO(Robot* robot, FlexivError* error) {
	try {
		FileIO* fileIO = new FileIO(*robot);
		error->error_code = 0;
		return fileIO;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteFileIO(FileIO* fileIO) {
	delete fileIO;
}

EXPORT_API char* GetTrajFilesList(FileIO* fileIO, FlexivError* error) {
	try {
		json j = fileIO->traj_files_list();
		std::string str = j.dump();
		error->error_code = 0;
		return CopyInputString(str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void UploadTrajFile(FileIO* fileIO, const char* fileDir, const char* fileName, FlexivError* error) {
	try {
		fileIO->UploadTrajFile(fileDir, fileName);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API char* DownloadTrajFile(FileIO* fileIO, const char* fileName, FlexivError* error) {
	try {
		std::string str = fileIO->DownloadTrajFile(fileName);
		error->error_code = 0;
		return CopyInputString(str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DownloadTrajFile2(FileIO* fileIO, const char* fileName, const char* saveDir, FlexivError* error) {
	try {
		fileIO->DownloadTrajFile(fileName, saveDir);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//========================================= DEVICE ===========================================
EXPORT_API Device* CreateDevice(Robot* robot, FlexivError* error) {
	try {
		Device* device = new Device(*robot);
		error->error_code = 0;
		return device;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteDevice(Device* device) {
	delete device;
}

EXPORT_API char* GetDevicesList(Device* device, FlexivError* error) {
	try {
		const auto& devices_list = device->list();
		std::string json_str = json(devices_list).dump();
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return nullptr;
	}
}

EXPORT_API int HasDevice(Device* device, const char* name, FlexivError* error) {
	try {
		int flag = device->exist(name);
		error->error_code = 0;
		return flag;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return 0;
	}
}

EXPORT_API char* GetDeviceParams(Device* device, const char* name, FlexivError* error) {
	try {
		const auto& deviceParams = device->params(name);
		std::map<std::string, FlexivDataTypes> ret;
		for (const auto& [key, value] : deviceParams) {
			std::visit([&](auto&& arg) {
				ret[key] = arg;
			}, value);
		}
		std::string json_str = serializeParams(ret);
		error->error_code = 0;
		return CopyInputString(json_str.c_str());
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
		return nullptr;
	}
}

EXPORT_API void EnableDevice(Device* device, const char* name, FlexivError* error) {
	try {
		device->Enable(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void DisableDevice(Device* device, const char* name, FlexivError* error) {
	try {
		device->Disable(name);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SendCommands(Device* device, const char* name, const char* cmds, FlexivError* error) {
	try {
		json json_obj = json::parse(cmds);
		std::map<std::string, std::variant<bool, int, double>> device_cmds;
		for (auto& [key, value] : json_obj.items()) {
			if (value.is_number_integer()) {
				device_cmds[key] = value.get<int>();
			}
			else if (value.is_boolean()) {
				device_cmds[key] = value.get<bool>();
			}
			else if (value.is_number_float()) {
				device_cmds[key] = value.get<double>();
			}
		}
		device->Command(name, device_cmds);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//======================================= MAINTENANCE ========================================
EXPORT_API Maintenance* CreateMaintenance(Robot* robot, FlexivError* error) {
	try {
		Maintenance* maintenancePtr = new Maintenance(*robot);
		error->error_code = 0;
		return maintenancePtr;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteMaintenance(Maintenance* maintenancePtr) {
	delete maintenancePtr;
}

EXPORT_API void CalibrateJointTorqueSensors(Maintenance* maintenancePtr, const double* posture, int postureLen, FlexivError* error) {
	try {
		std::vector<double> cali_posture(posture, posture + postureLen);
		maintenancePtr->CalibrateJointTorqueSensors(cali_posture);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//========================================= SAFETY ============================================
EXPORT_API Safety* CreateSafety(Robot* robot, const char* password, FlexivError* error) {
	try {
		Safety* safetyPtr = new Safety(*robot, std::string(password));
		error->error_code = 0;
		return safetyPtr;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteSafety(Safety* safetyPtr) {
	delete safetyPtr;
}

EXPORT_API void DefaultLimits(Safety* safetyPtr, WSafetyLimits* limits) {
	const auto& val = safetyPtr->default_limits();
	for (int i = 0; i < kSerialJointDoF; ++i) {
		limits->q_min[i] = val.q_min[i];
		limits->q_max[i] = val.q_max[i];
		limits->dq_max_normal[i] = val.dq_max_normal[i];
		limits->dq_max_reduced[i] = val.dq_max_reduced[i];
	}
}

EXPORT_API void CurrentLimits(Safety* safetyPtr, WSafetyLimits* limits) {
	const auto& val = safetyPtr->current_limits();
	for (int i = 0; i < kSerialJointDoF; ++i) {
		limits->q_min[i] = val.q_min[i];
		limits->q_max[i] = val.q_max[i];
		limits->dq_max_normal[i] = val.dq_max_normal[i];
		limits->dq_max_reduced[i] = val.dq_max_reduced[i];
	}
}

EXPORT_API void GetSafetyInputs(Safety* safetyPtr, int* temp) {
	const auto& val = safetyPtr->safety_inputs();
	for (int i = 0; i < kSafetyIOPorts; ++i) {
		temp[i] = val[i];
	}
}

EXPORT_API void SetJointPositionLimits(Safety* safetyPtr, double* minPositions,
	int minLen, double* maxPositions, int maxLen, FlexivError* error) {
	try {
		std::vector<double> minPoses(minPositions, minPositions + minLen);
		std::vector<double> maxPoses(maxPositions, maxPositions + maxLen);
		safetyPtr->SetJointPositionLimits(minPoses, maxPoses);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetJointVelocityNormalLimits(Safety* safetyPtr, double* maxVel,
	int len, FlexivError* error) {
	try {
		std::vector<double> maxVel(maxVel, maxVel + len);
		safetyPtr->SetJointVelocityNormalLimits(maxVel);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void SetJointVelocityReducedLimits(Safety* safetyPtr, double* maxVel,
	int len, FlexivError* error) {
	try {
		std::vector<double> maxVel(maxVel, maxVel + len);
		safetyPtr->SetJointVelocityReducedLimits(maxVel);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

//========================================= MODEL ===========================================
EXPORT_API Model* CreateModel(Robot* robot, double gravityX, double gravityY, 
	double gravityZ, FlexivError* error) {
	try {
		Model* model = new Model(*robot, Eigen::Vector3d(gravityX, gravityY, gravityZ));
		error->error_code = 0;
		return model;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
	return nullptr;
}

EXPORT_API void DeleteModel(Model* model) {
	delete model;
}

EXPORT_API void Reload(Model* model, FlexivError* error) {
	try {
		model->Reload();
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Update(Model* model, double* pos, int posLen, double* vel, int velLen, FlexivError* error) {
	try {
		std::vector<double> positions(pos, pos + posLen);
		std::vector<double> velocities(vel, vel + velLen);
		model->Update(positions, velocities);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetJacobian(Model* model, const char* linkName, double* buffer, int rows, int cols, FlexivError* error) {
	try {
		if (!model || !buffer || !linkName) return;
		Eigen::MatrixXd J = model->J(std::string(linkName));
		if (J.rows() != rows || J.cols() != cols) return;
		std::memcpy(buffer, J.data(), sizeof(double) * rows * rows);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetJacobianDot(Model* model, const char* linkName, double* buffer, int rows, int cols, FlexivError* error) {
	try {
		if (!model || !buffer || !linkName) return;
		Eigen::MatrixXd dJ = model->dJ(std::string(linkName));
		if (dJ.rows() != rows || dJ.cols() != cols) return;
		std::memcpy(buffer, dJ.data(), sizeof(double) * rows * rows);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetMassMatrix(Model* model, double* buffer, int dof, FlexivError* error) {
	try {
		if (!model || !buffer) return;
		Eigen::MatrixXd mass = model->M();
		if (mass.rows() != dof || mass.cols() != dof) return;
		std::memcpy(buffer, mass.data(), sizeof(double) * dof * dof);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetCoriolisCentripetalMatrix(Model* model, double* buffer, int dof, FlexivError* error) {
	try {
		if (!model || !buffer) return;
		Eigen::MatrixXd coriolis = model->C();
		if (coriolis.rows() != dof || coriolis.cols() != dof) return;
		std::memcpy(buffer, coriolis.data(), sizeof(double) * dof * dof);
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void GetGravityForceVector(Model* model, double* buffer, int dof) {
	auto g = model->g();
	if (g.size() != dof) return;
	std::memcpy(buffer, g.data(), sizeof(double) * dof);
}

EXPORT_API void GetCoriolisForceVector(Model* model, double* buffer, int dof) {
	auto c = model->c();
	if (c.size() != dof) return;
	std::memcpy(buffer, c.data(), sizeof(double) * dof);
}

EXPORT_API void SyncURDF(Model* model, char* path, FlexivError* error) {
	try {
		model->SyncURDF(std::string(path));
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void Reachable(Model* model, double* pose, int poseLen, double* seed, int seedLen,
	int freeOri, int* reachable, double* ikSolution, FlexivError* error) {
	try {
		std::array<double, kPoseSize> poseArr;
		for (int i = 0; i < kPoseSize; ++i) poseArr[i] = pose[i];
		std::vector<double> seedVec(seed, seed + seedLen);
		auto result = model->reachable(poseArr, seedVec, freeOri != 0);
		*reachable = result.first ? 1 : 0;
		for (int i = 0; i < kPoseSize; ++i)
			ikSolution[i] = result.second[i];
		error->error_code = 0;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}

EXPORT_API void ConfigurationScore(Model* model, double* tScore, double* oScore, FlexivError* error) {
	try {
		auto score = model->configuration_score();
		*tScore = score.first;
		*oScore = score.second;
	}
	catch (const std::exception& e) {
		error->error_code = 1;
		CopyExceptionMsg(e, error);
	}
}