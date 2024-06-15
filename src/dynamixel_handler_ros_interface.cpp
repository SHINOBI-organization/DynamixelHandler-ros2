#include "dynamixel_handler.hpp"

string update_info(const vector<uint8_t>& id_list, const string& what_updated) {
    char header[99]; 
    sprintf(header, "[%d] servo(s) %s are updated", (int)id_list.size(), what_updated.c_str());
    return id_list_layout(id_list, string(header));
}

void store_goal(
    uint16_t id, double value, DynamixelHandler::GoalValueIndex goal_index, 
    pair<DynamixelHandler::LimitIndex, DynamixelHandler::LimitIndex> lim_index
) {
    const auto& limit = DynamixelHandler::limit_r_[id];
    auto val_max = ( DynamixelHandler::NONE == lim_index.second ) ?  256*2*M_PI : limit[lim_index.second]; //このあたり一般性のない書き方していてキモい
    auto val_min = ( DynamixelHandler::NONE == lim_index.first  ) ? -256*2*M_PI :
                   (        lim_index.first == lim_index.second ) ?   - val_max : limit[lim_index.first];
    DynamixelHandler::goal_w_[id][goal_index] = clamp( value, val_min, val_max );
    DynamixelHandler::is_goal_updated_[id] = true;
    DynamixelHandler::list_write_goal_.insert(goal_index);
}

//* ROS関係

void DynamixelHandler::CallBackDxlCommand(const DynamixelCommand::SharedPtr msg) {
    vector<uint8_t> id_list;
    if ( msg->id_list.empty() || msg->id_list[0]==0xFE) for (auto id : id_set_    ) id_list.push_back(id);
                                                 else for (auto id : msg->id_list) id_list.push_back(id);
    char header[100]; sprintf(header, "Command [%s] \n (id_list=[] or [254] means all IDs)", msg->command.c_str());
    if (varbose_callback_ && msg->command!="") ROS_INFO_STREAM(id_list_layout(id_list, string(header)));
    if (msg->command == "clear_error" || msg->command == "CE")
        for (auto id : id_list) { ClearHardwareError(id); TorqueOn(id);}
    if (msg->command == "torque_on"   || msg->command == "TON") 
        for (auto id : id_list) TorqueOn(id);
    if (msg->command == "torque_off"  || msg->command == "TOFF")
        for (auto id : id_list) TorqueOff(id);
    if (msg->command == "remove_id"   || msg->command == "RMID")
        for (auto id : id_list) id_set_.erase( id );
    if (msg->command == "enable") 
        for (auto id : id_list) WriteTorqueEnable(id, TORQUE_ENABLE);
    if (msg->command == "disable")
        for (auto id : id_list) WriteTorqueEnable(id, TORQUE_DISABLE);
    if (msg->command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);

    CallBackDxlCmd_X_Position(msg->x_pos);
    CallBackDxlCmd_X_Velocity(msg->x_vel);
    CallBackDxlCmd_X_Current(msg->x_cur);
    CallBackDxlCmd_X_CurrentPosition(msg->x_cur_pos);
    CallBackDxlCmd_X_ExtendedPosition(msg->x_ext_pos);
    CallBackDxlCmd_P_Current(msg->p_cur);
    CallBackDxlCmd_P_Velocity(msg->p_vel);
    CallBackDxlCmd_P_Position(msg->p_pos);
    CallBackDxlCmd_P_ExtendedPosition(msg->p_ext_pos);
}

void DynamixelHandler::CallBackDxlCmd_X_Position(const DynamixelCommandXControlPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_POSITION);
        if (has_pos) store_goal( msg.id_list[i], msg.position_deg[i]      *DEG, GOAL_POSITION, {MIN_POSITION_LIMIT, MAX_POSITION_LIMIT} );
        if (has_pv ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL,   {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC,   {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_pos || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Velocity(const DynamixelCommandXControlVelocity& msg) {
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_VELOCITY);
        if (has_vel) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_vel || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (x series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Current(const DynamixelCommandXControlCurrent& msg) {
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i], GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
        if ( is_change_mode || has_cur ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_CurrentPosition(const DynamixelCommandXControlCurrentPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_rot = !msg.id_list.empty() && msg.id_list.size() == msg.rotation.size();
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT_BASE_POSITION);
        const double position_deg = (has_pos ? msg.position_deg[i] : 0.0) + (has_rot ? msg.rotation[i]*360 : 0.0);
        if (has_pos || has_rot) store_goal( msg.id_list[i], position_deg             *DEG, GOAL_POSITION, {NONE              , NONE              } );
        if (has_cur           ) store_goal( msg.id_list[i], msg.current_ma[i]            , GOAL_CURRENT,  {CURRENT_LIMIT     , CURRENT_LIMIT     } );
        if (has_pv            ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL,   {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pa            ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC,   {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_pos || has_cur || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (x series)"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_ExtendedPosition(const DynamixelCommandXControlExtendedPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_rot = !msg.id_list.empty() && msg.id_list.size() == msg.rotation.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT_BASE_POSITION);
        const double position_deg = (has_pos ? msg.position_deg[i] : 0.0) + (has_rot ? msg.rotation[i]*360 : 0.0);
        if (has_pos || has_rot) store_goal( msg.id_list[i], position_deg             *DEG, GOAL_POSITION, {NONE              , NONE              } );
        if (has_pv            ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL,   {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pa            ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC,   {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_pos || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_P_Current(const DynamixelCommandPControlCurrent& msg) {
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i], GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
        if ( is_change_mode || has_cur ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_P_Velocity(const DynamixelCommandPControlVelocity& msg) {
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_VELOCITY);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i]            , GOAL_CURRENT , {CURRENT_LIMIT     , CURRENT_LIMIT     });
        if (has_vel) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT    , VELOCITY_LIMIT    });
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT});
        if ( is_change_mode || (has_cur || has_vel || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (varbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_P_Position(const DynamixelCommandPControlPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_POSITION);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i]            , GOAL_CURRENT , {CURRENT_LIMIT     , CURRENT_LIMIT     });
        if (has_pos) store_goal( msg.id_list[i], msg.position_deg[i]      *DEG, GOAL_POSITION, {MIN_POSITION_LIMIT, MAX_POSITION_LIMIT});
        if (has_vel) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT    , VELOCITY_LIMIT    });
        if (has_pv ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL  , {VELOCITY_LIMIT    , VELOCITY_LIMIT    });
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT});
        if ( is_change_mode || (has_cur || has_pos || has_vel || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (p series)"));
    if (varbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (p series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_P_ExtendedPosition(const DynamixelCommandPControlExtendedPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_rot = !msg.id_list.empty() && msg.id_list.size() == msg.rotation.size();
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_EXTENDED_POSITION);
        const double position_deg = (has_pos ? msg.position_deg[i] : 0.0) + (has_rot ? msg.rotation[i]*360 : 0.0);
        if (has_cur           ) store_goal( msg.id_list[i], msg.current_ma[i]            , GOAL_CURRENT , {CURRENT_LIMIT     , CURRENT_LIMIT} );
        if (has_pos || has_rot) store_goal( msg.id_list[i], position_deg             *DEG, GOAL_POSITION, {NONE              , NONE              } );
        if (has_vel           ) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pv            ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL  , {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pa            ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode && (has_cur || has_pos || has_rot || has_vel || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (p series)"));
    if (varbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (p series)"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlGoal(const DynamixelGoal& msg) {
    const bool has_pwm = !msg.id_list.empty() && msg.id_list.size() == msg.pwm_percent.size();
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        if (has_pwm) store_goal( msg.id_list[i], msg.pwm_percent[i]           , GOAL_PWM     , {PWM_LIMIT         , PWM_LIMIT         });
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i]            , GOAL_CURRENT , {CURRENT_LIMIT     , CURRENT_LIMIT     });
        if (has_vel) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT    , VELOCITY_LIMIT    });
        if (has_pos) store_goal( msg.id_list[i], msg.position_deg[i]      *DEG, GOAL_POSITION, {NONE              , NONE              });
        if (has_pv ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL  , {VELOCITY_LIMIT    , VELOCITY_LIMIT    });
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT});
        if ( has_pwm || has_cur || has_vel || has_pos || has_pv || has_pa ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (varbose_callback_ && has_pwm) ROS_INFO_STREAM(update_info(changed_id_list, "goal_pwm"));
    if (varbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current"));
    if (varbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position"));
    if (varbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity"));
    if (varbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity"));
    if (varbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlGain(const DynamixelGain& msg) {
    static const auto store_gain = [&](const vector<uint16_t>& id_list, const vector<uint16_t>& value_list, GainIndex index) {
        if ( id_list.size() != value_list.size() ) return vector<uint8_t>();
        vector<uint8_t> store_id_list;
        for (size_t i=0; i<id_list.size(); i++) {
            uint8_t id = id_list[i];
            gain_w_[id][index] = value_list[i];
            is_gain_updated_[id] = true;
            list_write_gain_.insert(index);
            store_id_list.push_back(id);
        }
        return store_id_list;
    };
 
    auto store_vi = store_gain(msg.id_list, msg.velocity_i_gain_pulse, VELOCITY_I_GAIN);
    if (varbose_callback_ && !store_vi.empty()) ROS_INFO_STREAM(update_info(store_vi, "velocity i gain"));
    auto store_vp = store_gain(msg.id_list, msg.velocity_p_gain_pulse, VELOCITY_P_GAIN);
    if (varbose_callback_ && !store_vp.empty()) ROS_INFO_STREAM(update_info(store_vp, "velocity p gain"));
    auto store_pd = store_gain(msg.id_list, msg.position_d_gain_pulse, POSITION_D_GAIN);
    if (varbose_callback_ && !store_pd.empty()) ROS_INFO_STREAM(update_info(store_pd, "position d gain"));
    auto store_pi = store_gain(msg.id_list, msg.position_i_gain_pulse, POSITION_I_GAIN);
    if (varbose_callback_ && !store_pi.empty()) ROS_INFO_STREAM(update_info(store_pi, "position i gain"));
    auto store_pp = store_gain(msg.id_list, msg.position_p_gain_pulse, POSITION_P_GAIN);
    if (varbose_callback_ && !store_pp.empty()) ROS_INFO_STREAM(update_info(store_pp, "position p gain"));
    auto store_fa = store_gain(msg.id_list, msg.feedforward_2nd_gain_pulse, FEEDFORWARD_ACC_GAIN);
    if (varbose_callback_ && !store_fa.empty()) ROS_INFO_STREAM(update_info(store_fa, "feedforward 2nd gain"));
    auto store_fv = store_gain(msg.id_list, msg.feedforward_1st_gain_pulse, FEEDFORWARD_VEL_GAIN);
    if (varbose_callback_ && !store_fv.empty()) ROS_INFO_STREAM(update_info(store_fv, "feedforward 1st gain"));

    if ( store_vi.empty() && store_vp.empty() && 
         store_pd.empty() && store_pi.empty() && store_pp.empty() && 
         store_fa.empty() && store_fv.empty() )
        ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlLimit(const DynamixelLimit& msg) {
    static const auto store_limit = [&](const vector<uint16_t>& id_list, const vector<double>& value_list, LimitIndex index, bool is_angle=false) {
        if ( id_list.size() != value_list.size() ) return vector<uint8_t>();
        vector<uint8_t> store_id_list;
        for (size_t i=0; i<id_list.size(); i++) {
            uint8_t id = id_list[i];
            limit_w_[id][index] = is_angle ? deg2rad(value_list[i]) : value_list[i];
            is_limit_updated_[id] = true;
            list_write_limit_.insert(index);
            store_id_list.push_back(id);
        }
        return store_id_list;
    };

    auto store_temp = store_limit(msg.id_list, msg.temperature_limit_deg_c, TEMPERATURE_LIMIT);
    if (varbose_callback_ && !store_temp.empty()) ROS_INFO_STREAM(update_info(store_temp, "temperature limit"));
    auto store_max_v = store_limit(msg.id_list, msg.max_voltage_limit_v, MAX_VOLTAGE_LIMIT);
    if (varbose_callback_ && !store_max_v.empty()) ROS_INFO_STREAM(update_info(store_max_v, "max voltage limit"));
    auto store_min_v = store_limit(msg.id_list, msg.min_voltage_limit_v, MIN_VOLTAGE_LIMIT);
    if (varbose_callback_ && !store_min_v.empty()) ROS_INFO_STREAM(update_info(store_min_v, "min voltage limit"));
    auto store_pwm = store_limit(msg.id_list, msg.pwm_limit_percent, PWM_LIMIT);
    if (varbose_callback_ && !store_pwm.empty()) ROS_INFO_STREAM(update_info(store_pwm, "pwm limit"));
    auto store_cur = store_limit(msg.id_list, msg.current_limit_ma, CURRENT_LIMIT);
    if (varbose_callback_ && !store_cur.empty()) ROS_INFO_STREAM(update_info(store_cur, "current limit"));
    auto store_acc = store_limit(msg.id_list, msg.acceleration_limit_deg_ss, ACCELERATION_LIMIT, true);
    if (varbose_callback_ && !store_acc.empty()) ROS_INFO_STREAM(update_info(store_acc, "acceleration limit"));
    auto store_vel = store_limit(msg.id_list, msg.velocity_limit_deg_s, VELOCITY_LIMIT, true);
    if (varbose_callback_ && !store_vel.empty()) ROS_INFO_STREAM(update_info(store_vel, "velocity limit"));
    auto store_max_p = store_limit(msg.id_list, msg.max_position_limit_deg, MAX_POSITION_LIMIT, true);
    if (varbose_callback_ && !store_max_p.empty()) ROS_INFO_STREAM(update_info(store_max_p, "max position limit"));
    auto store_min_p = store_limit(msg.id_list, msg.min_position_limit_deg, MIN_POSITION_LIMIT, true);

    if ( store_temp.empty() && store_max_v.empty() && store_min_v.empty() && store_pwm.empty() && store_cur.empty() && 
         store_acc.empty() && store_vel.empty() && store_max_p.empty() && store_min_p.empty() )
        ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");

    //* ROMへの書き込みなので即座に実行する。
    bool do_write = false;
    for ( auto id: id_set_ ) if(is_limit_updated_[id]){
        if(tq_mode_[id] == TORQUE_DISABLE) do_write = true;
    }
    if ( do_write ) SyncWriteLimit(list_write_limit_);
    list_write_limit_.clear();
}

double round4(double val) { return round(val*10000.0)/10000.0; }

void DynamixelHandler::BroadcastDxlState(){
    DynamixelState msg;
    msg.stamp = this->get_clock()->now();
    for (const auto& [id, value] : state_r_) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);

        msg.mode.torque_enable.push_back(tq_mode_[id]==TORQUE_ENABLE);
        switch(op_mode_[id]) {
            case OPERATING_MODE_CURRENT:              msg.mode.operating_mode.push_back("current");           break;
            case OPERATING_MODE_VELOCITY:             msg.mode.operating_mode.push_back("velocity");          break;
            case OPERATING_MODE_POSITION:             msg.mode.operating_mode.push_back("position");          break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.mode.operating_mode.push_back("extended_position"); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.mode.operating_mode.push_back("current_position");  break;
        }

        for (auto state : list_read_state_) switch(state) {
            case PRESENT_PWM:          msg.pwm_percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current_ma.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity_deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position_deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory_deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory_deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERTURE:   msg.temperature_deg_c.push_back   (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage_v.push_back     (round4(value[state]    )); break;
        }
    }
    pub_state_->publish(msg);
}

void DynamixelHandler::BroadcastDxlError(){
    DynamixelError msg;
    msg.stamp = this->get_clock()->now();
    for (const auto& [id, error]: hardware_error_) if ( is_in(id, id_set_) ) {
        if (error[INPUT_VOLTAGE     ]) msg.input_voltage.push_back     (id);
        if (error[MOTOR_HALL_SENSOR ]) msg.motor_hall_sensor.push_back (id);
        if (error[OVERHEATING       ]) msg.overheating.push_back       (id);
        if (error[MOTOR_ENCODER     ]) msg.motor_encoder.push_back     (id);
        if (error[ELECTRONICAL_SHOCK]) msg.electronical_shock.push_back(id);
        if (error[OVERLOAD          ]) msg.overload.push_back          (id);
    }
    pub_error_->publish(msg);
}

void DynamixelHandler::BroadcastDxlLimit(){
    DynamixelLimit msg;
    msg.stamp = this->get_clock()->now();
    for (const auto& [id, limit] : limit_r_) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.temperature_limit_deg_c.push_back  (round4(limit[TEMPERATURE_LIMIT ]));
        msg.max_voltage_limit_v.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]));
        msg.min_voltage_limit_v.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]));
        msg.pwm_limit_percent.push_back        (round4(limit[PWM_LIMIT         ]));
        msg.current_limit_ma.push_back         (round4(limit[CURRENT_LIMIT     ]));
        msg.acceleration_limit_deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit_deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit_deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit_deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    pub_limit_->publish(msg);
}

void DynamixelHandler::BroadcastDxlGain(){
    DynamixelGain msg;
    msg.stamp = this->get_clock()->now();
    for ( const auto& [id, gain] : gain_r_ ) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.velocity_i_gain_pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain_pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain_pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain_pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain_pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_2nd_gain_pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_1st_gain_pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    pub_gain_->publish(msg);
}

void DynamixelHandler::BroadcastDxlGoal(){
    DynamixelGoal msg;
    msg.stamp = this->get_clock()->now();
    for ( const auto& [id, goal] : goal_r_ ) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.pwm_percent.push_back       (round4(goal[GOAL_PWM     ]));
        msg.current_ma.push_back        (round4(goal[GOAL_CURRENT ]));
        msg.velocity_deg_s.push_back    (round4(goal[GOAL_VELOCITY]/DEG));
        msg.profile_vel_deg_s.push_back (round4(goal[PROFILE_VEL  ]/DEG));
        msg.profile_acc_deg_ss.push_back(round4(goal[PROFILE_ACC  ]/DEG));
        msg.position_deg.push_back      (round4(goal[GOAL_POSITION]/DEG));
    }
    pub_goal_->publish(msg);
}