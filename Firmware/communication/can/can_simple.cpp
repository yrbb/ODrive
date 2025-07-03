#include "can_simple.hpp"
#include <stdio.h>
#include <string>
#include <odrive_main.h>
#include <functional>
#include <fibre/introspection.hpp>

extern Introspectable root_obj;

// 定义静态成员变量 keyMap
const std::map<uint8_t, std::string> CANSimple::keyMap = {
    {0, "axis%d.motor.config.pole_pairs"},                    // 极对数
    {1, "axis%d.motor.config.calibration_current"},           // 校准电流
    {2, "axis%d.motor.config.resistance_calib_max_voltage"},  // 校准电压
    {3, "axis%d.motor.config.motor_type"},                    // 电机类型
    {4, "axis%d.motor.config.requested_current_range"},       // 采样范围
    {5, "axis%d.motor.config.current_control_bandwidth"},     // 电流环带宽
    {6, "axis%d.motor.config.torque_constant"},               // 扭矩常数
    {7, "axis%d.motor.config.current_lim"},                   // 最大电流
    {8, "axis%d.encoder.config.mode"},                        // 编码器模式
    {9, "axis%d.encoder.config.cpr"},                         // 编码器分辨率
    {10, "axis%d.encoder.config.bandwidth"},                  // 编码器带宽
    {11, "axis%d.encoder.config.calib_scan_distance"},        // 编码器校准扫描距离
    {12, "axis%d.controller.config.vel_limit"},               // 最大转速
    {13, "axis%d.controller.config.control_mode"},            // 控制模式
    {14, "axis%d.controller.config.input_mode"},              // 输入模式
    {15, "axis%d.controller.config.pos_gain"},                // 位置环增益
    {16, "axis%d.controller.config.vel_gain"},                // 速度环增益
    {17, "axis%d.controller.config.vel_integrator_gain"},     // 速度环积分增益
    {18, "axis%d.controller.config.vel_ramp_rate"},           // 速度环爬升率
    {19, "axis%d.controller.config.input_filter_bandwidth"},  // 输入滤波带宽
    {20, "axis%d.controller.config.torque_ramp_rate"},        // 扭矩环爬升率
    {21, "axis%d.controller.config.inertia"},                 // 惯量
    {22, "axis%d.trap_traj.config.vel_limit"},                // 梯形轨迹速度限制
    {23, "axis%d.trap_traj.config.accel_limit"},              // 梯形轨迹加速度限制
    {24, "axis%d.trap_traj.config.decel_limit"},              // 梯形轨迹减速度限制
    {25, "axis%d.requested_state"},
};

bool CANSimple::init() {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!renew_subscription(i)) {
            return false;
        }
    }

    return true;
}

bool CANSimple::renew_subscription(size_t i) {
    Axis& axis = axes[i];

    // TODO: remove these two lines (see comment in header)
    node_ids_[i] = axis.config_.can.node_id;
    extended_node_ids_[i] = axis.config_.can.is_extended;

    MsgIdFilterSpecs filter = {
        .id = {},
        .mask = (uint32_t)(0xffffffff << NUM_CMD_ID_BITS)};
    if (axis.config_.can.is_extended) {
        filter.id = (uint32_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    } else {
        filter.id = (uint16_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    }

    if (subscription_handles_[i]) {
        canbus_->unsubscribe(subscription_handles_[i]);
    }

    return canbus_->subscribe(
        filter, [](void* ctx, const can_Message_t& msg) {
            ((CANSimple*)ctx)->handle_can_message(msg);
        },
        this, &subscription_handles_[i]);
}

void CANSimple::handle_can_message(const can_Message_t& msg) {
    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);

    for (auto& axis : axes) {
        if ((axis.config_.can.node_id == nodeID) && (axis.config_.can.is_extended == msg.isExt)) {
            do_command(axis, msg);
            return;
        }
    }
}

void CANSimple::do_command(Axis& axis, const can_Message_t& msg) {
    const uint32_t cmd = get_cmd_id(msg.id);
    axis.watchdog_feed();
    switch (cmd) {
        case MSG_CO_NMT_CTRL:
            break;
        case MSG_CO_HEARTBEAT_CMD:
            break;
        case MSG_ODRIVE_HEARTBEAT:
            // We don't currently do anything to respond to ODrive heartbeat messages
            break;
        case MSG_ODRIVE_ESTOP:
            estop_callback(axis, msg);
            break;
        case MSG_GET_ERROR:
            get_error_callback(axis, msg);
            break;
        case MSG_SET_AXIS_NODE_ID:
            set_axis_nodeid_callback(axis, msg);
            break;
        case MSG_SET_AXIS_REQUESTED_STATE:
            set_axis_requested_state_callback(axis, msg);
            break;
        case MSG_SET_AXIS_STARTUP_CONFIG:
            set_axis_startup_config_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            if (msg.rtr || msg.len == 0)
                get_encoder_estimates_callback(axis);
            break;
        case MSG_GET_ENCODER_COUNT:
            if (msg.rtr || msg.len == 0)
                get_encoder_count_callback(axis);
            break;
        case MSG_SET_INPUT_POS:
            set_input_pos_callback(axis, msg);
            break;
        case MSG_SET_INPUT_VEL:
            set_input_vel_callback(axis, msg);
            break;
        case MSG_SET_INPUT_TORQUE:
            set_input_torque_callback(axis, msg);
            break;
        case MSG_SET_LIMITS:
            set_limits_callback(axis, msg);
            break;
        case MSG_START_ANTICOGGING:
            start_anticogging_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_INERTIA:
            set_traj_inertia_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_ACCEL_LIMITS:
            set_traj_accel_limits_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_VEL_LIMIT:
            set_traj_vel_limit_callback(axis, msg);
            break;
        case MSG_GET_IQ:
            if (msg.rtr || msg.len == 0)
                get_iq_callback(axis);
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            if (msg.rtr || msg.len == 0)
                get_sensorless_estimates_callback(axis);
            break;
        case MSG_RESET_ODRIVE:
            reboot_callback(axis);
            break;
        case MSG_GET_BUS_VOLTAGE_CURRENT:
            if (msg.rtr || msg.len == 0)
                get_bus_voltage_current_callback(axis);
            break;
        case MSG_CLEAR_ERRORS:
            clear_errors_callback(axis, msg);
            break;
        case MSG_SET_LINEAR_COUNT:
            set_linear_count_callback(axis, msg);
            break;
        case MSG_SET_POS_GAIN:
            set_pos_gain_callback(axis, msg);
            break;
        case MSG_SET_VEL_GAINS:
            set_vel_gains_callback(axis, msg);
            break;
        case MSG_GET_ADC_VOLTAGE:
            get_adc_voltage_callback(axis, msg);
            break;
        case MSG_SAVE_CONFIG:
            save_config_callback(axis);
            break;
        case MSG_SET_PARAMS:
            set_params_callback(axis, msg);
            break;
        case MSG_GET_PARAMS:
            get_params_callback(axis, msg);
            break;
        default:
            break;
    }
}

void CANSimple::nmt_callback(const Axis& axis, const can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis& axis, const can_Message_t& msg) {
    axis.error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

bool CANSimple::get_error_callback(const Axis& axis, const can_Message_t& msg) {
    uint8_t errorId = can_getSignal<uint8_t>(msg, 0, 32, true);
    switch (errorId) {
        case ERROR_MOTOR:
            return get_motor_error_callback(axis);
        case ERROR_ENCODER:
            return get_encoder_error_callback(axis);
        case ERROR_CONTROLLER:
            return get_controller_error_callback(axis);
        case ERROR_SENSORLESS:
            return get_sensorless_error_callback(axis);
        default:
            return false;
    }
}

bool CANSimple::get_motor_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, ERROR_MOTOR, 0, 32, true);
    can_setSignal(txmsg, axis.motor_.error_, 32, 72, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, ERROR_ENCODER, 0, 32, true);
    can_setSignal(txmsg, axis.encoder_.error_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, ERROR_SENSORLESS, 0, 32, true);
    can_setSignal(txmsg, axis.sensorless_estimator_.error_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_controller_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, ERROR_CONTROLLER, 0, 32, true);
    can_setSignal(txmsg, axis.controller_.error_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

void CANSimple::set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg) {
    axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg) {
    axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg) {
    // Not Implemented
}

bool CANSimple::get_encoder_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<float>(txmsg, axis.controller_.pos_estimate_linear_src_.any().value_or(0.0f), 0, 32, true);
    can_setSignal<float>(txmsg, axis.controller_.vel_estimate_src_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(axis.sensorless_estimator_.pll_pos_));

    can_setSignal<float>(txmsg, axis.sensorless_estimator_.pll_pos_, 0, 32, true);
    can_setSignal<float>(txmsg, axis.sensorless_estimator_.vel_estimate_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_count_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_COUNT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
    can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
    return canbus_->send_message(txmsg);
}

void CANSimple::set_input_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.set_input_pos_and_steps(can_getSignal<float>(msg, 0, 32, true));
    axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis.controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis& axis, const can_Message_t& msg) {
    Controller::ControlMode const mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(mode);
    axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
    axis.controller_.control_mode_updated();
}

void CANSimple::set_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::start_anticogging_callback(const Axis& axis, const can_Message_t& msg) {
    axis.controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_linear_count_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_pos_gain_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_vel_gains_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

bool CANSimple::get_iq_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_IQ;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    std::optional<float2D> Idq_setpoint = axis.motor_.current_control_.Idq_setpoint_;
    if (!Idq_setpoint.has_value()) {
        Idq_setpoint = {0.0f, 0.0f};
    }
    
    static_assert(sizeof(float) == sizeof(Idq_setpoint->second));
    static_assert(sizeof(float) == sizeof(axis.motor_.current_control_.Iq_measured_));
    can_setSignal<float>(txmsg, Idq_setpoint->second, 0, 32, true);
    can_setSignal<float>(txmsg, axis.motor_.current_control_.Iq_measured_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_bus_voltage_current_callback(const Axis& axis) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_BUS_VOLTAGE_CURRENT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(vbus_voltage));
    static_assert(sizeof(float) == sizeof(ibus_));
    can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);
    can_setSignal<float>(txmsg, ibus_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_adc_voltage_callback(const Axis& axis, const can_Message_t& msg) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ADC_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
    if (gpio_num < GPIO_COUNT) {
        auto voltage = get_adc_voltage(get_gpio(gpio_num));
        can_setSignal<float>(txmsg, voltage, 0, 32, true);
        return canbus_->send_message(txmsg);
    } else {
        return false;
    }
}

void CANSimple::clear_errors_callback(Axis& axis, const can_Message_t& msg) {
    odrv.clear_errors();  // TODO: might want to clear axis errors only
}


void CANSimple::reboot_callback(const Axis& axis) {
    odrv.reboot();
}

void CANSimple::save_config_callback(const Axis& axis) {
    odrv.save_configuration();
}

void CANSimple::set_params_callback(const Axis& axis, const can_Message_t& msg) {
    // uint8_t: paramId, float: value
    uint8_t paramId = can_getSignal<uint8_t>(msg, 0, 32, true);
    auto it = keyMap.find(paramId);
    if (it == keyMap.end()) {
        return;
    }

    float value = can_getSignal<float>(msg, 32, 32, true);

    char name[128];
    std::snprintf(name, sizeof(name), it->second.c_str(), (uint8_t)axis.config_.can.node_id);

    Introspectable property = root_obj.get_child(name, sizeof(name));
    const FloatSettableTypeInfo* type_info = dynamic_cast<const FloatSettableTypeInfo*>(property.get_type_info());
    if (type_info) {
        type_info->set_float(property, value);
    }
}

bool CANSimple::get_params_callback(const Axis& axis, const can_Message_t& msg) {
    uint8_t paramId = can_getSignal<uint8_t>(msg, 0, 32, true);
    auto it = keyMap.find(paramId);
    if (it == keyMap.end()) {
        return false;
    }
        
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_PARAMS;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    char name[128];
    std::snprintf(name, sizeof(name), it->second.c_str(), (uint8_t)axis.config_.can.node_id);

    Introspectable property = root_obj.get_child(name, sizeof(name));
    const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
    if (!type_info) {
        return false;
    }

    char response[10];
    type_info->get_string(property, response, sizeof(response));

    float val = std::stof(response);
    
    can_setSignal<uint8_t>(txmsg, paramId, 0, 8, true);
    can_setSignal<float>(txmsg, val, 8, 32, true);
    return canbus_->send_message(txmsg);
}

uint32_t CANSimple::service_stack() {
    uint32_t nextServiceTime = UINT32_MAX;
    uint32_t now = HAL_GetTick();

    // TODO: remove this polling loop and replace with protocol hook
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool node_id_changed = (axes[i].config_.can.node_id != node_ids_[i]) || (axes[i].config_.can.is_extended != extended_node_ids_[i]);
        if (node_id_changed) {
            renew_subscription(i);
        }
    }

    struct periodic {
        const uint32_t& rate;
        uint32_t& last_time;
        bool (CANSimple::* callback)(const Axis& axis);
    };

    for (auto& axis : axes) {
        std::array<periodic, 10> periodics = {{
            {axis.config_.can.heartbeat_rate_ms, axis.can_.last_heartbeat, &CANSimple::send_heartbeat},
            {axis.config_.can.encoder_rate_ms, axis.can_.last_encoder, &CANSimple::get_encoder_estimates_callback},
            {axis.config_.can.motor_error_rate_ms, axis.can_.last_motor_error, &CANSimple::get_motor_error_callback},
            {axis.config_.can.encoder_error_rate_ms, axis.can_.last_encoder_error, &CANSimple::get_encoder_error_callback},
            {axis.config_.can.controller_error_rate_ms, axis.can_.last_controller_error, &CANSimple::get_controller_error_callback},
            {axis.config_.can.sensorless_error_rate_ms, axis.can_.last_sensorless_error, &CANSimple::get_sensorless_error_callback},
            {axis.config_.can.encoder_count_rate_ms, axis.can_.last_encoder_count, &CANSimple::get_encoder_count_callback},
            {axis.config_.can.iq_rate_ms, axis.can_.last_iq, &CANSimple::get_iq_callback},
            {axis.config_.can.sensorless_rate_ms, axis.can_.last_sensorless, &CANSimple::get_sensorless_estimates_callback},
            {axis.config_.can.bus_vi_rate_ms, axis.can_.last_bus_vi, &CANSimple::get_bus_voltage_current_callback},
        }};

        MEASURE_TIME(axis.task_times_.can_heartbeat) {
            for (auto& msg : periodics) {
                if (msg.rate > 0) {
                    if ((now - msg.last_time) >= msg.rate) {
                        if (std::invoke(msg.callback, this, axis)) {
                            msg.last_time = now;
                        }
                    }

                    int nextAxisService = msg.last_time + msg.rate - now;
                    nextServiceTime = std::min(nextServiceTime, static_cast<uint32_t>(std::max(0, nextAxisService)));
                }
            }
        }
    }

    return nextServiceTime;
}

bool CANSimple::send_heartbeat(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.error_, 0, 32, true);
    can_setSignal(txmsg, uint8_t(axis.current_state_), 32, 8, true);

    // Motor flags
    uint8_t motorFlags = axis.motor_.error_ != 0;

    // Encoder flags
    uint8_t encoderFlags = axis.encoder_.error_ != 0;

    // Controller flags
    uint8_t controllerFlags =axis.controller_.error_ != 0;
    uint8_t trajDone = uint8_t(axis.controller_.trajectory_done_) << 7;
    controllerFlags |= trajDone;

    can_setSignal(txmsg, motorFlags, 40, 8, true);
    can_setSignal(txmsg, encoderFlags, 48, 8, true);
    can_setSignal(txmsg, controllerFlags, 56, 8, true);

    return canbus_->send_message(txmsg);
}
