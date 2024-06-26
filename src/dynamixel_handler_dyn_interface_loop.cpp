#include "dynamixel_handler.hpp"

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteCommandValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_write_cmd 書き込むコマンドのEnumのリスト
*/
template <> void DynamixelHandler::SyncWriteCommandValues(set<CmdValueIndex> list_write_cmd){
    SyncWriteCommandValues<AddrX>(list_write_cmd);
    SyncWriteCommandValues<AddrP>(list_write_cmd);
}
template <typename Addr> void DynamixelHandler::SyncWriteCommandValues(set<CmdValueIndex> list_write_cmd){
    if ( list_write_cmd.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(list_write_cmd.begin(), list_write_cmd.end()); 
    if ( use_split_write_ ) end = start; // 分割書き込みが有効な場合は書き込む範囲を1つ目のみに制限
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> cmd_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_cmd_vec_map; // id と 書き込むデータのベクタのマップ
    for (CmdValueIndex cmd = *start; cmd <= *end; cmd++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (cmd) {
            case GOAL_PWM      : cmd_addr_list.push_back(Addr::goal_pwm            ); break;
            case GOAL_CURRENT  : cmd_addr_list.push_back(Addr::goal_current        ); break;
            case GOAL_VELOCITY : cmd_addr_list.push_back(Addr::goal_velocity       ); break;
            case PROFILE_ACC   : cmd_addr_list.push_back(Addr::profile_acceleration); break;
            case PROFILE_VEL   : cmd_addr_list.push_back(Addr::profile_velocity    ); break;
            case GOAL_POSITION : cmd_addr_list.push_back(Addr::goal_position       ); break;
            default: /*ここに来たらエラ-*/ exit(1);
        }
        const auto& addr = cmd_addr_list.back();
        for (auto id : id_set_) if ( series_[id]==Addr::series() ) {
            if ( !is_cmd_updated_[id] ) continue;
            id_cmd_vec_map[id].push_back( addr.val2pulse( cmd_values_[id][cmd], model_[id] ) );
        }
    }
    if ( id_cmd_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_cmd_vec_mapの中身を確認
    if ( varbose_write_cmd_ ) {
        char header[100]; sprintf(header, "[%d] servo(s) will be written", (int)id_cmd_vec_map.size());
        auto ss = control_table_layout(width_log_, id_cmd_vec_map, cmd_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(cmd_addr_list, id_cmd_vec_map);
    //*再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    list_write_cmd.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteCommandValues<Addr>(list_write_cmd);
    //*後処理，再帰の終端で実行される, 書き込んだ id のフラグをリセット
    for (const auto& [id, _] : id_cmd_vec_map) is_cmd_updated_[id] = false;
}

template <> void DynamixelHandler::SyncWriteOption_Mode(){
    SyncWriteOption_Mode<AddrX>();
    SyncWriteOption_Mode<AddrP>();
}
template <typename Addr> void DynamixelHandler::SyncWriteOption_Mode(){
    return;
}

template <> void DynamixelHandler::SyncWriteOption_Gain(){
    SyncWriteOption_Gain<AddrX>();
    SyncWriteOption_Gain<AddrP>();
}
template <typename Addr> void DynamixelHandler::SyncWriteOption_Gain(){
    return;
}

template <> void DynamixelHandler::SyncWriteOption_Limit(){
    SyncWriteOption_Limit<AddrX>();
    SyncWriteOption_Limit<AddrP>();
}
template <typename Addr> void DynamixelHandler::SyncWriteOption_Limit(){
    return;
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み取りの成功率, なんで再帰で頑張って実装してるんだろう．．．
*/
template <>double DynamixelHandler::SyncReadStateValues(set<StValueIndex> list_read_state){
    double suc_rate_X = SyncReadStateValues<AddrX>(list_read_state);
    double suc_rate_P = SyncReadStateValues<AddrP>(list_read_state);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadStateValues(set<StValueIndex> list_read_state){
    if ( list_read_state.empty() ) return 1.0; // 空なら即時return
    //* 読み込む範囲のstate_addr_listのインデックスを取得
    auto [start, end] = minmax_element(list_read_state.begin(), list_read_state.end());
    if ( use_split_read_ ) end = start; // 分割読み込みが有効な場合は読み込む範囲を1つ目のみに制限
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> state_addr_list;
    for (StValueIndex st=*start; st<=*end; st++) switch (st) {
        case PRESENT_PWM          : state_addr_list.push_back(Addr::present_pwm          ); break; 
        case PRESENT_CURRENT      : state_addr_list.push_back(Addr::present_current      ); break; 
        case PRESENT_VELOCITY     : state_addr_list.push_back(Addr::present_velocity     ); break;    
        case PRESENT_POSITION     : state_addr_list.push_back(Addr::present_position     ); break;   
        case VELOCITY_TRAJECTORY  : state_addr_list.push_back(Addr::velocity_trajectory  ); break;   
        case POSITION_TRAJECTORY  : state_addr_list.push_back(Addr::position_trajectory  ); break;  
        case PRESENT_INPUT_VOLTAGE: state_addr_list.push_back(Addr::present_input_voltage); break; 
        case PRESENT_TEMPERTURE   : state_addr_list.push_back(Addr::present_temperture   ); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }
    vector<uint8_t> target_id_list;
    for (auto id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return
    // SyncReadでまとめて読み込み
    const auto id_st_vec_map = ( use_fast_read_ ) // fast read を使うかどうか． 途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(state_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (state_addr_list, target_id_list); fflush(stdout);
    const size_t N_total = target_id_list.size();
    const size_t N_suc   = id_st_vec_map.size();
    const bool is_timeout_  = dyn_comm_.timeout_last_read();
    const bool is_comm_err_ = dyn_comm_.comm_error_last_read();
    has_hardware_err_ = dyn_comm_.hardware_error_last_read();
    //* 通信エラーの表示
    if ( varbose_read_st_err_ ) if ( is_timeout_ || is_comm_err_ ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_st_vec_map.find(id) == id_st_vec_map.end() ) failed_id_list.push_back(id);
        char header[99]; sprintf(header, "[%d] servo(s) failed to read", (int)(N_total - N_suc));
        auto ss = id_list_layout(failed_id_list, string(header)+( is_timeout_ ? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    //* id_st_vec_mapの中身を確認
    if ( varbose_read_st_ ) if ( N_suc>0 ) {
        char header[99]; sprintf(header, "[%d] servo(s) are read", (int)N_suc);
        auto ss = control_table_layout(width_log_, id_st_vec_map, state_addr_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( has_hardware_err_ ) ROS_WARN( "Hardware Error are detected");
    }
    //* state_values_に反映
    const int num_state_next = list_read_state.size();
    const int num_state_now = *end-*start+1;
    for (int i = 0; i < num_state_now; i++) {
        const auto addr = state_addr_list[i];
        for (const auto& [id, data_int] : id_st_vec_map)
            state_values_[id][*start+i] = addr.pulse2val( data_int[i], model_[id]);
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, use_split_read_=falseの場合は全て削除されるので,再帰しない
    list_read_state.erase(start, ++end); // 今回読み込んだ範囲を消去
    double suc_rate = SyncReadStateValues<Addr>(list_read_state);
    return       suc_rate       *num_state_next / (num_state_now+num_state_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
         + N_suc/(double)N_total*num_state_now  / (num_state_now+num_state_next);// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadHardwareErrors(){
    double suc_rate_X = SyncReadHardwareErrors<AddrX>();
    double suc_rate_P = SyncReadHardwareErrors<AddrP>();
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / series_.size();
}
template <typename Addr> double DynamixelHandler::SyncReadHardwareErrors(){
    if ( !has_hardware_err_ ) { hardware_error_.clear(); return 1.0; } // 事前にエラーが検出できていない場合は省略

    vector<uint8_t> target_id_list;
    for (int id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return
    
    auto id_error_map =  ( use_fast_read_ ) 
        ? dyn_comm_.SyncRead_fast(Addr::hardware_error_status, target_id_list)
        : dyn_comm_.SyncRead     (Addr::hardware_error_status, target_id_list); fflush(stdout);

    if ( dyn_comm_.timeout_last_read() ) return 0.0; // 読み込み失敗

    //  hardware_error_に反映
    for (const auto& [id, error] : id_error_map ){
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) hardware_error_[id][INPUT_VOLTAGE     ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) hardware_error_[id][MOTOR_HALL_SENSOR ] = true;
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) hardware_error_[id][OVERHEATING       ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) hardware_error_[id][MOTOR_ENCODER     ] = true;
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) hardware_error_[id][ELECTRONICAL_SHOCK] = true;
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) hardware_error_[id][OVERLOAD          ] = true;
    }

    // コンソールへの表示
    if ( varbose_read_hwerr_ ) {
        ROS_WARN( "Hardware error are Checked");
        for (auto id : target_id_list) {
            if (hardware_error_[id][INPUT_VOLTAGE     ]) ROS_ERROR(" * servo id [%d] has INPUT_VOLTAGE error",      id);
            if (hardware_error_[id][MOTOR_HALL_SENSOR ]) ROS_ERROR(" * servo id [%d] has MOTOR_HALL_SENSOR error",  id);
            if (hardware_error_[id][OVERHEATING       ]) ROS_ERROR(" * servo id [%d] has OVERHEATING error",        id);
            if (hardware_error_[id][MOTOR_ENCODER     ]) ROS_ERROR(" * servo id [%d] has MOTOR_ENCODER error",      id);
            if (hardware_error_[id][ELECTRONICAL_SHOCK]) ROS_ERROR(" * servo id [%d] has ELECTRONICAL_SHOCK error", id);
            if (hardware_error_[id][OVERLOAD          ]) ROS_ERROR(" * servo id [%d] has OVERLOAD error",           id);
        }
    }
    return id_error_map.size()/double(target_id_list.size());
}

/**
 * @func SyncReadOption_Mode
 * @brief モードに関する値を読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadOption_Mode(){
    double suc_rate_X = SyncReadOption_Mode<AddrX>();
    double suc_rate_P = SyncReadOption_Mode<AddrP>();
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / series_.size();
}
template <typename Addr> double DynamixelHandler::SyncReadOption_Mode(){
    vector<uint8_t> target_id_list;
    for (int id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_torque_map     = dyn_comm_.SyncRead( Addr::torque_enable,                      target_id_list);
    auto id_dv_op_mode_map = dyn_comm_.SyncRead({Addr::drive_mode, Addr::operating_mode}, target_id_list);  fflush(stdout);

    for ( const auto& [id, toqrue] : id_torque_map ) tq_mode_[id] = toqrue;
    for ( const auto& [id, dv_op]  : id_dv_op_mode_map ) {
        dv_mode_[id] = dv_op[0];
        op_mode_[id] = dv_op[1];
    }
    return 1.0;
}

/**
 * @func SyncReadOption_Gain
 * @brief ゲインをすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadOption_Gain(){
    double suc_rate_X = SyncReadOption_Gain<AddrX>();
    double suc_rate_P = SyncReadOption_Gain<AddrP>();
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / series_.size();
}
template <typename Addr> double DynamixelHandler::SyncReadOption_Gain(){
    OptGainIndex start = VELOCITY_I_GAIN;
    OptGainIndex end   = FEEDFORWARD_VEL_GAIN;
    vector<DynamixelAddress> opt_gain_addr_list;
    for (OptGainIndex g=start; g<=end; g++) switch ( g ) {
        case VELOCITY_I_GAIN     : opt_gain_addr_list.push_back(Addr::velocity_i_gain     ); break;
        case VELOCITY_P_GAIN     : opt_gain_addr_list.push_back(Addr::velocity_p_gain     ); break;
        case POSITION_D_GAIN     : opt_gain_addr_list.push_back(Addr::position_d_gain     ); break;
        case POSITION_I_GAIN     : opt_gain_addr_list.push_back(Addr::position_i_gain     ); break;
        case POSITION_P_GAIN     : opt_gain_addr_list.push_back(Addr::position_p_gain     ); break;
        case FEEDFORWARD_ACC_GAIN: opt_gain_addr_list.push_back(Addr::feedforward_2nd_gain); break;
        case FEEDFORWARD_VEL_GAIN: opt_gain_addr_list.push_back(Addr::feedforward_1st_gain); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_gain_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(opt_gain_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (opt_gain_addr_list, target_id_list);  fflush(stdout);
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_gain_vec_map.find(id) == id_gain_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_gain_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // id_gain_vec_mapの中身を確認
    if ( varbose_read_opt_ ) if ( id_gain_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_gain_vec_map.size());
        auto ss = control_table_layout(width_log_, id_gain_vec_map, opt_gain_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // option_gain_に反映
    for ( size_t opt_gain=0; opt_gain<opt_gain_addr_list.size(); opt_gain++) {
        DynamixelAddress addr = opt_gain_addr_list[opt_gain];
        for (const auto& [id, data_int] : id_gain_vec_map)
            option_gain_[id][opt_gain] = addr.pulse2val( data_int[opt_gain], model_[id] );
    }
    return id_gain_vec_map.size() / (double)target_id_list.size();
}

/**
 * @func SyncReadOption_Limit
 * @brief 制限値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadOption_Limit(){
    double suc_rate_X = SyncReadOption_Limit<AddrX>();
    double suc_rate_P = SyncReadOption_Limit<AddrP>();
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / series_.size();
}
template <typename Addr> double DynamixelHandler::SyncReadOption_Limit(){
    OptLimitIndex start = TEMPERATURE_LIMIT;
    OptLimitIndex end   = MIN_POSITION_LIMIT;
    vector<DynamixelAddress> opt_limit_addr_list;
    for (OptLimitIndex l=start; l<=end; l++) switch ( l ) {
        case TEMPERATURE_LIMIT : opt_limit_addr_list.push_back(Addr::temperature_limit ); break;
        case MAX_VOLTAGE_LIMIT : opt_limit_addr_list.push_back(Addr::max_voltage_limit ); break;
        case MIN_VOLTAGE_LIMIT : opt_limit_addr_list.push_back(Addr::min_voltage_limit ); break;
        case PWM_LIMIT         : opt_limit_addr_list.push_back(Addr::pwm_limit         ); break;
        case CURRENT_LIMIT     : opt_limit_addr_list.push_back(Addr::current_limit     ); break;
        case ACCELERATION_LIMIT: opt_limit_addr_list.push_back(Addr::acceleration_limit); break;
        case VELOCITY_LIMIT    : opt_limit_addr_list.push_back(Addr::velocity_limit    ); break;
        case MAX_POSITION_LIMIT: opt_limit_addr_list.push_back(Addr::max_position_limit); break;
        case MIN_POSITION_LIMIT: opt_limit_addr_list.push_back(Addr::min_position_limit); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_limit_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(opt_limit_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (opt_limit_addr_list, target_id_list); fflush(stdout);
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_limit_vec_map.find(id) == id_limit_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_limit_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // ACCELERATION_LIMITに関してだけ修正を入れる．0はほぼあり得ないかつ0の時profile_accの設定ができないので，適当に大きな値に変更する．
    vector<uint8_t> fixed_id_list;
    if (start <= ACCELERATION_LIMIT && ACCELERATION_LIMIT <= end) 
        for (auto& [id, limit] : id_limit_vec_map) {
            if ( limit[ACCELERATION_LIMIT-start] != 0 ) continue;
            fixed_id_list.push_back(id);
            limit[ACCELERATION_LIMIT-start] = 32767; //  Xシリーズのprofile_accの設定ができる最大値
        }
    // id_limit_vec_mapの中身を確認
    if ( varbose_read_opt_ ) if ( id_limit_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_limit_vec_map.size());
        auto ss = control_table_layout(width_log_, id_limit_vec_map, opt_limit_addr_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( !fixed_id_list.empty() ) {
            char header[100]; sprintf(header,"\n[%d] servo(s)' accelerarion_limit is 0, change to 32767", (int)fixed_id_list.size());
            auto ss = id_list_layout(fixed_id_list, string(header));
            ROS_WARN_STREAM(ss);
        }
    }
    // option_limit_に反映
    for ( size_t opt_lim=0; opt_lim<opt_limit_addr_list.size(); opt_lim++) {
        DynamixelAddress addr = opt_limit_addr_list[opt_lim];
        for (const auto& [id, data_int] : id_limit_vec_map)
            option_limit_[id][opt_lim] = addr.pulse2val( data_int[opt_lim], model_[id] );
    }
    return id_limit_vec_map.size() / (double)target_id_list.size();
}

/**
 * @func SyncReadOption_Goal
 * @brief 目標値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadOption_Goal(){
    double suc_rate_X = SyncReadOption_Goal<AddrX>();
    double suc_rate_P = SyncReadOption_Goal<AddrP>();
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / series_.size();
}
template <typename Addr> double DynamixelHandler::SyncReadOption_Goal() {
    CmdValueIndex start = GOAL_PWM;
    CmdValueIndex end   = GOAL_POSITION;
    vector<DynamixelAddress> opt_goal_addr_list;
    for (CmdValueIndex g=start; g<=end; g++) switch ( g ) {
        case GOAL_PWM      : opt_goal_addr_list.push_back(Addr::goal_pwm      ); break;
        case GOAL_CURRENT  : opt_goal_addr_list.push_back(Addr::goal_current  ); break;
        case GOAL_VELOCITY : opt_goal_addr_list.push_back(Addr::goal_velocity ); break;
        case PROFILE_ACC   : opt_goal_addr_list.push_back(Addr::profile_acceleration); break;
        case PROFILE_VEL   : opt_goal_addr_list.push_back(Addr::profile_velocity    ); break;
        case GOAL_POSITION : opt_goal_addr_list.push_back(Addr::goal_position       ); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set_) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_goal_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(opt_goal_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (opt_goal_addr_list, target_id_list);  fflush(stdout);
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_goal_vec_map.find(id) == id_goal_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_goal_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    if ( varbose_read_opt_ ) if ( id_goal_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_goal_vec_map.size());
        auto ss = control_table_layout(width_log_, id_goal_vec_map, opt_goal_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // option_goal_に反映
    for ( size_t opt_goal=0; opt_goal<opt_goal_addr_list.size(); opt_goal++) {
        DynamixelAddress addr = opt_goal_addr_list[opt_goal];
        for (const auto& [id, data_int] : id_goal_vec_map)
            option_goal_[id][opt_goal] = addr.pulse2val( data_int[opt_goal], model_[id] );
    }
    return id_goal_vec_map.size() / (double)target_id_list.size();
}

// 全てのモータの動作を停止させる．
template <> void DynamixelHandler::SyncStopDynamixels(){
    SyncStopDynamixels<AddrX>();
    SyncStopDynamixels<AddrP>();
} 
template <typename Addr> void DynamixelHandler::SyncStopDynamixels(){
    vector<uint8_t> id_list; 
    for (auto id : id_set_) if ( series_[id]==Addr::series() ) id_list.push_back(id);
    vector<int64_t> offset_pulse(id_list.size(), 0);
    dyn_comm_.SyncWrite(Addr::homing_offset ,id_list, offset_pulse); // マジで謎だが，BusWatchdogを設定するとHomingOffset分だけ回転してしまう...多分ファームrウェアのバグ
    vector<int64_t> bus_watchtime_pulse(id_list.size(), 1);
    dyn_comm_.SyncWrite(Addr::bus_watchdog, id_list, bus_watchtime_pulse);
    ROS_INFO("%s servo will be stopped", Addr::series()==SERIES_X ? "X series" 
                                       : Addr::series()==SERIES_X ? "P series" : "Unknown");
}
