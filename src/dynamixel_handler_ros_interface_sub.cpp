#include "dynamixel_handler.hpp"
#include "myUtils/formatting_output.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"

// 角度変換
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
static double deg2rad(double deg){ return deg*DEG; }

template <typename T> string update_info(const vector<T>& id_list, const string& what_updated) {
    char header[99]; 
    sprintf(header, "[%d] servo(s) %s are updated", (int)id_list.size(), what_updated.c_str());
    return id_list_layout(id_list, string(header));
}

void store_goal(
    uint16_t id, double value, DynamixelHandler::GoalIndex goal_index, 
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

void DynamixelHandler::CallbackCmdsX(const DxlCommandsX::SharedPtr msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Common(msg->common);
    CallbackCmd_Status(msg->status);
    CallbackCmd_X_Pwm(msg->pwm_control);
    CallbackCmd_X_Current(msg->current_control);
    CallbackCmd_X_Velocity(msg->velocity_control);
    CallbackCmd_X_Position(msg->position_control);
    CallbackCmd_X_ExtendedPosition(msg->extended_position_control);
    CallbackCmd_X_CurrentBasePosition(msg->current_base_position_control);
    CallbackCmd_Goal  (msg->goal);
    CallbackCmd_Gain  (msg->gain);
    CallbackCmd_Limit (msg->limit);
}
void DynamixelHandler::CallbackCmdsP(const DxlCommandsP::SharedPtr msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Common(msg->common);
    CallbackCmd_Status(msg->status);
    CallbackCmd_P_Pwm(msg->pwm_control);
    CallbackCmd_P_Current(msg->current_control);
    CallbackCmd_P_Velocity(msg->velocity_control);
    CallbackCmd_P_Position(msg->position_control);
    CallbackCmd_P_ExtendedPosition(msg->extended_position_control);
    CallbackCmd_Goal  (msg->goal);
    CallbackCmd_Gain  (msg->gain);
    CallbackCmd_Limit (msg->limit);
}

void DynamixelHandler::CallbackCmd_Common(const DynamixelCommonCmd& msg) {
    if ( msg.command=="" ) return;
    if (verbose_callback_ ) {
        ROS_INFO_STREAM(id_list_layout(msg.id_list, "Common cmd, ID"));
        ROS_INFO(" command [%s] (ID=[] or [254] means all IDs)", msg.command.c_str());
    }
    // msg.id_list内をもとに，処理用のIDリストを作成
    vector<uint16_t> id_list;
    for (auto id : id_set_) if ( msg.id_list.empty() || msg.id_list==vector<uint16_t>{0xFE} || is_in(id, msg.id_list) ) id_list.push_back(id);
    auto st_cmd = DynamixelStatus().set__id_list(id_list);
         if (msg.command == msg.CLEAR_ERROR  || msg.command == "CE"  ) CallbackCmd_Status(st_cmd.set__error (vector<bool>(id_list.size(), false)));
    else if (msg.command == msg.TORQUE_ON    || msg.command == "TON" ) CallbackCmd_Status(st_cmd.set__torque(vector<bool>(id_list.size(),  true)));
    else if (msg.command == msg.TORQUE_OFF   || msg.command == "TOFF") CallbackCmd_Status(st_cmd.set__torque(vector<bool>(id_list.size(), false)));
    else if (msg.command == msg.ADD_ID       || msg.command == "ADID") CallbackCmd_Status(st_cmd.set__ping  (vector<bool>(id_list.size(),  true)));
    else if (msg.command == msg.REMOVE_ID    || msg.command == "RMID") CallbackCmd_Status(st_cmd.set__ping  (vector<bool>(id_list.size(), false)));
    else if (msg.command == msg.RESET_OFFSET ) for (auto id : id_list) {WriteHomingOffset(id, 0);              ROS_INFO("  - set: offset zero, ID [%d]"   , id);}
    else if (msg.command == msg.ENABLE       ) for (auto id : id_list) {WriteTorqueEnable(id, TORQUE_ENABLE);  ROS_INFO("  - set: torque enable, ID [%d]" , id);}
    else if (msg.command == msg.DISABLE      ) for (auto id : id_list) {WriteTorqueEnable(id, TORQUE_DISABLE); ROS_INFO("  - set: torque disable, ID [%d]", id);}
    else if (msg.command == msg.REBOOT       ) for (auto id : id_list) {dyn_comm_.Reboot(id);                  ROS_INFO("  - reboot: ID [%d] ", id);}
    else ROS_WARN("  Invalid command [%s]", msg.command.c_str());
}

void DynamixelHandler::CallbackCmd_Status(const DynamixelStatus& msg) {
    // msg.id_list内のIDの妥当性を確認
    vector<uint16_t> valid_id_list;
    for (auto id : msg.id_list) if ( is_in(id, id_set_) ) valid_id_list.push_back(id);
    if ( valid_id_list.empty() ) return; // 有効なIDがない場合は何もしない
    const bool has_torque = msg.id_list.size() == msg.torque.size();
    const bool has_error  = msg.id_list.size() == msg.error.size() ;
    const bool has_ping   = msg.id_list.size() == msg.ping.size()  ;
    const bool has_mode   = msg.id_list.size() == msg.mode.size()  ;
    if (verbose_callback_ ) {
        ROS_INFO("Status cmd, '%zu' servo(s) are tryed to updated", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID"));
        if ( has_torque ) ROS_INFO("  - updated: torque"); else if ( !msg.torque.empty() ) ROS_WARN("  - skipped: torque (size mismatch)");
        if ( has_error  ) ROS_INFO("  - updated: error "); else if ( !msg.error .empty() ) ROS_WARN("  - skipped: error  (size mismatch)");
        if ( has_ping   ) ROS_INFO("  - updated: ping  "); else if ( !msg.ping  .empty() ) ROS_WARN("  - skipped: ping   (size mismatch)");
        if ( has_mode   ) ROS_INFO("  - updated: mode  "); else if ( !msg.mode  .empty() ) ROS_WARN("  - skipped: mode   (size mismatch)");
    }
    for (size_t i=0; i<msg.id_list.size(); i++) if ( auto ID = msg.id_list[i]; is_in(ID, valid_id_list) ) { // 順番がずれるのでわざとこの書き方をしている．
        if ( has_error  ) { ClearHardwareError(ID); TorqueOn(ID); }
        if ( has_torque ) msg.torque[i] ? TorqueOn(ID)     : TorqueOff(ID)      ;
        if ( has_ping   ) msg.ping[i]   ? addDynamixel(ID) : RemoveDynamixel(ID);
        if ( has_mode   ) {
                 if (msg.mode[i] == msg.CONTROL_PWM                  ) ChangeOperatingMode(ID, OPERATING_MODE_PWM                  );
            else if (msg.mode[i] == msg.CONTROL_CURRENT              ) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT              );
            else if (msg.mode[i] == msg.CONTROL_VELOCITY             ) ChangeOperatingMode(ID, OPERATING_MODE_VELOCITY             );
            else if (msg.mode[i] == msg.CONTROL_POSITION             ) ChangeOperatingMode(ID, OPERATING_MODE_POSITION             );
            else if (msg.mode[i] == msg.CONTROL_EXTENDED_POSITION    ) ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION    );
            else if (msg.mode[i] == msg.CONTROL_CURRENT_BASE_POSITION) 
                                 switch (series_[ID]) { case SERIES_X: ChangeOperatingMode(ID, OPERATING_MODE_CURRENT_BASE_POSITION); break;
                                                        case SERIES_P: ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION    );
                                                                       ROS_WARN("  ID [%d] is P-series, so alternative mode is selected", ID);}
            else ROS_WARN("  Invalid operating mode [%s], please see CallbackCmd_Status.msg definition.", msg.mode[i].c_str());
        }
    } // 各単体関数(ClearHardwareErrorとか)が内部でROS_INFOを出力しているので，ここでは何も出力しない
    if ( verbose_callback_ ) ROS_INFO("==================+==================");
}

void DynamixelHandler::CallbackCmd_X_Pwm(const DynamixelControlXPwm& msg) {
    const bool has_pwm = !msg.id_list.empty() && msg.id_list.size() == msg.pwm_percent.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_PWM);
        if (has_pwm) store_goal( msg.id_list[i], msg.pwm_percent[i], GOAL_PWM, {PWM_LIMIT, PWM_LIMIT} );
        if ( is_change_mode || has_pwm ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_pwm) ROS_INFO_STREAM(update_info(changed_id_list, "goal_pwm (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_X_Position(const DynamixelControlXPosition& msg) {
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
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_X_Velocity(const DynamixelControlXVelocity& msg) {
    const bool has_vel = !msg.id_list.empty() && msg.id_list.size() == msg.velocity_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_VELOCITY);
        if (has_vel) store_goal( msg.id_list[i], msg.velocity_deg_s[i]    *DEG, GOAL_VELOCITY, {VELOCITY_LIMIT, VELOCITY_LIMIT} );
        if (has_pa ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC  , {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_vel || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (x series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_X_Current(const DynamixelControlXCurrent& msg) {
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i], GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
        if ( is_change_mode || has_cur ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_X_CurrentBasePosition(const DynamixelControlXCurrentBasePosition& msg) {
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
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (x series)"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_X_ExtendedPosition(const DynamixelControlXExtendedPosition& msg) {
    const bool has_pos = !msg.id_list.empty() && msg.id_list.size() == msg.position_deg.size();
    const bool has_rot = !msg.id_list.empty() && msg.id_list.size() == msg.rotation.size();
    const bool has_pv  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool has_pa  = !msg.id_list.empty() && msg.id_list.size() == msg.profile_acc_deg_ss.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_X ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_EXTENDED_POSITION);
        const double position_deg = (has_pos ? msg.position_deg[i] : 0.0) + (has_rot ? msg.rotation[i]*360 : 0.0);
        if (has_pos || has_rot) store_goal( msg.id_list[i], position_deg             *DEG, GOAL_POSITION, {NONE              , NONE              } );
        if (has_pv            ) store_goal( msg.id_list[i], msg.profile_vel_deg_s[i] *DEG, PROFILE_VEL,   {VELOCITY_LIMIT    , VELOCITY_LIMIT    } );
        if (has_pa            ) store_goal( msg.id_list[i], msg.profile_acc_deg_ss[i]*DEG, PROFILE_ACC,   {ACCELERATION_LIMIT, ACCELERATION_LIMIT} );
        if ( is_change_mode || (has_pos || has_pv || has_pa) ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (x series)"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (x series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (x series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_P_Pwm(const DynamixelControlPPwm& msg) {
    const bool has_pwm = !msg.id_list.empty() && msg.id_list.size() == msg.pwm_percent.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_PWM);
        if (has_pwm) store_goal( msg.id_list[i], msg.pwm_percent[i], GOAL_PWM, {PWM_LIMIT, PWM_LIMIT} );
        if ( is_change_mode || has_pwm ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_pwm) ROS_INFO_STREAM(update_info(changed_id_list, "goal_pwm (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_P_Current(const DynamixelControlPCurrent& msg) {
    const bool has_cur = !msg.id_list.empty() && msg.id_list.size() == msg.current_ma.size();
    vector<uint8_t> changed_id_list;
    for ( size_t i=0; i<msg.id_list.size(); i++ ) if ( series_[msg.id_list[i]] == SERIES_P ) {
        const bool is_change_mode = ChangeOperatingMode(msg.id_list[i], OPERATING_MODE_CURRENT);
        if (has_cur) store_goal( msg.id_list[i], msg.current_ma[i], GOAL_CURRENT, {CURRENT_LIMIT, CURRENT_LIMIT} );
        if ( is_change_mode || has_cur ) changed_id_list.push_back(msg.id_list[i]);
    }
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_P_Velocity(const DynamixelControlPVelocity& msg) {
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
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (verbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_P_Position(const DynamixelControlPPosition& msg) {
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
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (p series)"));
    if (verbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (p series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_P_ExtendedPosition(const DynamixelControlPExtendedPosition& msg) {
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
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current (p series)"));
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position (p series)"));
    if (verbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity (p series)"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity (p series)"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration (p series)"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_Goal(const DynamixelGoal& msg) {
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
    if (verbose_callback_ && has_pwm) ROS_INFO_STREAM(update_info(changed_id_list, "goal_pwm"));
    if (verbose_callback_ && has_cur) ROS_INFO_STREAM(update_info(changed_id_list, "goal_current"));
    if (verbose_callback_ && has_pos) ROS_INFO_STREAM(update_info(changed_id_list, "goal_position"));
    if (verbose_callback_ && has_vel) ROS_INFO_STREAM(update_info(changed_id_list, "goal_velocity"));
    if (verbose_callback_ && has_pv ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_velocity"));
    if (verbose_callback_ && has_pa ) ROS_INFO_STREAM(update_info(changed_id_list, "profile_acceleration"));
    if ( !msg.id_list.empty() && changed_id_list.empty() ) ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_Gain(const DynamixelGain& msg) {
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
    if (verbose_callback_ && !store_vi.empty()) ROS_INFO_STREAM(update_info(store_vi, "velocity i gain"));
    auto store_vp = store_gain(msg.id_list, msg.velocity_p_gain_pulse, VELOCITY_P_GAIN);
    if (verbose_callback_ && !store_vp.empty()) ROS_INFO_STREAM(update_info(store_vp, "velocity p gain"));
    auto store_pd = store_gain(msg.id_list, msg.position_d_gain_pulse, POSITION_D_GAIN);
    if (verbose_callback_ && !store_pd.empty()) ROS_INFO_STREAM(update_info(store_pd, "position d gain"));
    auto store_pi = store_gain(msg.id_list, msg.position_i_gain_pulse, POSITION_I_GAIN);
    if (verbose_callback_ && !store_pi.empty()) ROS_INFO_STREAM(update_info(store_pi, "position i gain"));
    auto store_pp = store_gain(msg.id_list, msg.position_p_gain_pulse, POSITION_P_GAIN);
    if (verbose_callback_ && !store_pp.empty()) ROS_INFO_STREAM(update_info(store_pp, "position p gain"));
    auto store_fa = store_gain(msg.id_list, msg.feedforward_2nd_gain_pulse, FEEDFORWARD_ACC_GAIN);
    if (verbose_callback_ && !store_fa.empty()) ROS_INFO_STREAM(update_info(store_fa, "feedforward 2nd gain"));
    auto store_fv = store_gain(msg.id_list, msg.feedforward_1st_gain_pulse, FEEDFORWARD_VEL_GAIN);
    if (verbose_callback_ && !store_fv.empty()) ROS_INFO_STREAM(update_info(store_fv, "feedforward 1st gain"));

    if ( store_vi.empty() && store_vp.empty() && 
         store_pd.empty() && store_pi.empty() && store_pp.empty() && 
         store_fa.empty() && store_fv.empty() )
        ROS_ERROR("Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallbackCmd_Limit(const DynamixelLimit& msg) {
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

    auto store_temp = store_limit(msg.id_list, msg.temperature_limit_degc, TEMPERATURE_LIMIT);
    if (verbose_callback_ && !store_temp.empty()) ROS_INFO_STREAM(update_info(store_temp, "temperature limit"));
    auto store_max_v = store_limit(msg.id_list, msg.max_voltage_limit_v, MAX_VOLTAGE_LIMIT);
    if (verbose_callback_ && !store_max_v.empty()) ROS_INFO_STREAM(update_info(store_max_v, "max voltage limit"));
    auto store_min_v = store_limit(msg.id_list, msg.min_voltage_limit_v, MIN_VOLTAGE_LIMIT);
    if (verbose_callback_ && !store_min_v.empty()) ROS_INFO_STREAM(update_info(store_min_v, "min voltage limit"));
    auto store_pwm = store_limit(msg.id_list, msg.pwm_limit_percent, PWM_LIMIT);
    if (verbose_callback_ && !store_pwm.empty()) ROS_INFO_STREAM(update_info(store_pwm, "pwm limit"));
    auto store_cur = store_limit(msg.id_list, msg.current_limit_ma, CURRENT_LIMIT);
    if (verbose_callback_ && !store_cur.empty()) ROS_INFO_STREAM(update_info(store_cur, "current limit"));
    auto store_acc = store_limit(msg.id_list, msg.acceleration_limit_deg_ss, ACCELERATION_LIMIT, true);
    if (verbose_callback_ && !store_acc.empty()) ROS_INFO_STREAM(update_info(store_acc, "acceleration limit"));
    auto store_vel = store_limit(msg.id_list, msg.velocity_limit_deg_s, VELOCITY_LIMIT, true);
    if (verbose_callback_ && !store_vel.empty()) ROS_INFO_STREAM(update_info(store_vel, "velocity limit"));
    auto store_max_p = store_limit(msg.id_list, msg.max_position_limit_deg, MAX_POSITION_LIMIT, true);
    if (verbose_callback_ && !store_max_p.empty()) ROS_INFO_STREAM(update_info(store_max_p, "max position limit"));
    auto store_min_p = store_limit(msg.id_list, msg.min_position_limit_deg, MIN_POSITION_LIMIT, true);

    if ( store_temp.empty() && store_max_v.empty() && store_min_v.empty() && store_pwm.empty() && store_cur.empty() && 
         store_acc.empty() && store_vel.empty() && store_max_p.empty() && store_min_p.empty() )
        ROS_WARN("\nElement size or Dyanmxiel Series is dismatch; skiped callback");
}