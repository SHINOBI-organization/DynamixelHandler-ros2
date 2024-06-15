#include "dynamixel_handler.hpp"
#include "rclcpp/rclcpp.hpp"

using std::bind;
using std::placeholders::_1;

DynamixelHandler::DynamixelHandler() : Node("dynamixel_handler", rclcpp::NodeOptions()
                                                                  .allow_undeclared_parameters(true)
                                                                  .automatically_declare_parameters_from_overrides(true)) {
    ROS_INFO( "Initializing DynamixelHandler .....");

    // 通信の開始
    int baudrate      ; this->get_parameter_or("baudrate"     , baudrate     , 57600                 );
    int latency_timer ; this->get_parameter_or("latency_timer", latency_timer,    16                 );
    string device_name; this->get_parameter_or("device_name"  , device_name  , string("/dev/ttyUSB0"));

    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer);
    if ( !dyn_comm_.OpenPort() ) { fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        throw std::runtime_error("Initialization failed (device open)");
    } 
    // serial通信のvarbose設定
    bool serial_varbose; get_parameter_or("dyn_comm/varbose", serial_varbose, false);
    dyn_comm_.set_varbose(serial_varbose); fflush(stdout);

    // serial通信のretry設定
    int num_try      ; get_parameter_or("dyn_comm/retry_num"   , num_try      ,  5);
    int msec_interval; get_parameter_or("dyn_comm/inerval_msec", msec_interval, 10);
    dyn_comm_.set_retry_config(num_try, msec_interval); fflush(stdout);

    // main loop の設定
    this->get_parameter_or("loop_rate", loop_rate_, 50u);
    this->get_parameter_or("ratio/read_state"  , ratio_state_pub_,   1u);
    this->get_parameter_or("ratio/read_error"  , ratio_error_pub_, 100u);
    this->get_parameter_or("ratio/read_limit"  , ratio_limit_pub_,   0u);
    this->get_parameter_or("ratio/read_gain"   , ratio_gain_pub_ ,   0u);
    this->get_parameter_or("ratio/read_goal"   , ratio_goal_pub_ ,   0u);
    this->get_parameter_or("ratio/varbose_loop", ratio_mainloop_ , 100u);
    this->get_parameter_or("max_log_width"     , width_log_      ,   7u);
    this->get_parameter_or("use/split_write"    , use_split_write_    , false);
    this->get_parameter_or("use/split_read"     , use_split_read_     , false);
    this->get_parameter_or("use/fast_read"      , use_fast_read_      , true);
    this->get_parameter_or("use/multi_rate_read", use_multi_rate_read_, false);
    this->get_parameter_or("varbose/callback"           , varbose_callback_    , false);
    this->get_parameter_or("varbose/write_commad"       , varbose_write_cmd_   , false);
    this->get_parameter_or("varbose/write_options"      , varbose_write_opt_   , false);
    this->get_parameter_or("varbose/read_state/raw"     , varbose_read_st_     , false);
    this->get_parameter_or("varbose/read_state/err"     , varbose_read_st_err_ , false);
    this->get_parameter_or("varbose/read_options/raw"   , varbose_read_opt_    , false);
    this->get_parameter_or("varbose/read_options/err"   , varbose_read_opt_err_, false);
    this->get_parameter_or("varbose/read_hardware_error", varbose_read_hwerr_  , false);

    // id_listの作成
    int num_expexted; this->get_parameter_or("init/expected_servo_num"     , num_expexted, 0);
    int times_retry ; this->get_parameter_or("init/auto_search_retry_times", times_retry , 5);
    int id_min      ; this->get_parameter_or("init/auto_search_min_id"     , id_min      , 1);
    int id_max      ; this->get_parameter_or("init/auto_search_max_id"     , id_max      , 35);

    if ( num_expexted>0 ) ROS_INFO("Expected number of Dynamixel is [%d]", num_expexted);
    else                  ROS_WARN("\nExpected number of Dynamixel is not set. Free number of Dynamixel is allowed");
    ROS_INFO(" Auto scanning Dynamixel (id range [%d] to [%d]) ...", id_min, id_max);
    auto num_found = ScanDynamixels(id_min, id_max, num_expexted, times_retry);
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        throw std::runtime_error("Initialization failed (no dynamixel found)");
    }
    if( num_expexted>0 && num_expexted!=num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]. please check & retry", num_expexted, num_found);
        throw std::runtime_error("Initialization failed (number of dynamixel is not matched)");
    }
    ROS_INFO("  ... Finish scanning Dynamixel");

    // Subscriber / Publisherの設定
    if ( num_[SERIES_X] > 0 ) {
        sub_cmd_x_pos_  = create_subscription<DynamixelCommandXControlPosition>        ("dynamixel/x_cmd/position",          4, bind(&DynamixelHandler::CallBackDxlCmd_X_Position, this, _1));
        sub_cmd_x_vel_  = create_subscription<DynamixelCommandXControlVelocity>        ("dynamixel/x_cmd/velocity",          4, bind(&DynamixelHandler::CallBackDxlCmd_X_Velocity, this, _1));
        sub_cmd_x_cur_  = create_subscription<DynamixelCommandXControlCurrent>         ("dynamixel/x_cmd/current",           4, bind(&DynamixelHandler::CallBackDxlCmd_X_Current, this, _1));
        sub_cmd_x_cpos_ = create_subscription<DynamixelCommandXControlCurrentPosition> ("dynamixel/x_cmd/current_position",  4, bind(&DynamixelHandler::CallBackDxlCmd_X_CurrentPosition, this, _1));
        sub_cmd_x_epos_ = create_subscription<DynamixelCommandXControlExtendedPosition>("dynamixel/x_cmd/extended_position", 4, bind(&DynamixelHandler::CallBackDxlCmd_X_ExtendedPosition, this, _1));
    }
    if ( num_[SERIES_P] > 0) {
        sub_cmd_p_pos_  = create_subscription<DynamixelCommandPControlPosition>        ("dynamixel/p_cmd/position",          4, bind(&DynamixelHandler::CallBackDxlCmd_P_Position, this, _1));
        sub_cmd_p_vel_  = create_subscription<DynamixelCommandPControlVelocity>        ("dynamixel/p_cmd/velocity",          4, bind(&DynamixelHandler::CallBackDxlCmd_P_Velocity, this, _1));
        sub_cmd_p_cur_  = create_subscription<DynamixelCommandPControlCurrent>         ("dynamixel/p_cmd/current",           4, bind(&DynamixelHandler::CallBackDxlCmd_P_Current, this, _1));
        sub_cmd_p_epos_ = create_subscription<DynamixelCommandPControlExtendedPosition>("dynamixel/p_cmd/extended_position", 4, bind(&DynamixelHandler::CallBackDxlCmd_P_ExtendedPosition, this, _1));
    }
    sub_command_    = create_subscription<DynamixelCommand>("dynamixel/command", 10, bind(&DynamixelHandler::CallBackDxlCommand, this, _1));
    sub_goal_ = create_subscription<DynamixelGoal> ("dynamixel/goal/w", 4, bind(&DynamixelHandler::CallBackDxlGoal, this, _1));
    sub_gain_ = create_subscription<DynamixelGain> ("dynamixel/gain/w", 4, bind(&DynamixelHandler::CallBackDxlGain, this, _1));
    sub_limit_= create_subscription<DynamixelLimit>("dynamixel/limit/w",4, bind(&DynamixelHandler::CallBackDxlLimit, this, _1));

    pub_state_     = create_publisher<DynamixelState>("dynamixel/state", 4);
    pub_error_     = create_publisher<DynamixelError>("dynamixel/error", 4);
    pub_goal_  = create_publisher<DynamixelGoal> ("dynamixel/goal/r",  4);
    pub_gain_  = create_publisher<DynamixelGain> ("dynamixel/gain/r",  4);
    pub_limit_ = create_publisher<DynamixelLimit>("dynamixel/limit/r", 4);

    // 状態のreadの前にやるべき初期化
    double init_pa; this->get_parameter_or("init/profile_acceleration", init_pa, 600.0*DEG);
    double init_pv; this->get_parameter_or("init/profile_velocity"    , init_pv, 100.0*DEG);
    for (auto id : id_set_) {
        WriteBusWatchdog (id, 0.0 );
        WriteProfileAcc(id, init_pa ); //  設定ファイルからとってこれるようにする
        WriteProfileVel(id, init_pv ); //  設定ファイルからとってこれるようにする
    }

    // 最初の一回は全ての情報をread & publish
    ROS_INFO(" Reading present dynamixel status  ...");  
    ROS_INFO(" * state values reading .. "); 
        while ( rclcpp::ok() && SyncReadState( list_read_state_ ) < 1.0-1e-6 ) rsleep(50);                                           
    ROS_INFO(" * limit values reading .. "); 
        while ( rclcpp::ok() && SyncReadLimit( list_read_limit_ ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * gain values reading  .. "); 
        while ( rclcpp::ok() && SyncReadGain ( list_read_gain_  ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * goal values reading  .. "); 
        while ( rclcpp::ok() && SyncReadGoal ( list_read_goal_  ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * hardware error reading .. "); 
        while ( rclcpp::ok() && SyncReadHardwareErrors() < 1.0-1e-6 ) rsleep(50); // 最後にやる
    ROS_INFO(" * mode values reading  ..");
    for (auto id : id_set_) {
        tq_mode_[id] = ReadTorqueEnable(id);
        op_mode_[id] = ReadOperatingMode(id);
        dv_mode_[id] = ReadDriveMode(id);
        limit_w_[id] = limit_r_[id];
        gain_w_[id] = gain_r_[id];
        goal_w_[id] = goal_r_[id];
    }
    ROS_INFO("  ... status reading done");

    BroadcastDxlState();
    BroadcastDxlLimit();
    BroadcastDxlGain();  
    BroadcastDxlGoal();   
    BroadcastDxlError(); 

    for ( auto id : id_set_ ) if ( abs(init_pa - goal_r_[id][PROFILE_ACC]) > 0.1 ) ROS_WARN("\nProfile acceleration is not set correctly [%d], your setting [%f], but [%f]", id, init_pa, goal_r_[id][PROFILE_ACC]);
    for ( auto id : id_set_ ) if ( abs(init_pv - goal_r_[id][PROFILE_VEL]) > 0.1 ) ROS_WARN("\nProfile velocity is not set correctly [%d], your setting [%f], but [%f]", id, init_pv, goal_r_[id][PROFILE_VEL]);

    // 状態のreadの後にやるべき初期化
    ROS_INFO( " Initializing dynamixel state  ...");
    bool do_clean_hwerr; this->get_parameter_or("init/hardware_error_auto_clean", do_clean_hwerr, true);
    bool do_torque_on  ; this->get_parameter_or("init/torque_auto_enable"       , do_torque_on  , true);
    for (auto id : id_set_) {
        if ( do_clean_hwerr ) ClearHardwareError(id); // 現在の状態を変えない
        if ( do_torque_on )   TorqueOn(id);           // 現在の状態を変えない
    } ROS_INFO( "  ... state initialization done ");

    //  readする情報の設定
    if ( !use_multi_rate_read_ ) {
        list_read_state_.clear();
        bool tmp;
        get_parameter_or("read/present_pwm"          , tmp, false); if( tmp ) list_read_state_.insert(PRESENT_PWM          );
        get_parameter_or("read/present_current"      , tmp,  true); if( tmp ) list_read_state_.insert(PRESENT_CURRENT      );
        get_parameter_or("read/present_velocity"     , tmp,  true); if( tmp ) list_read_state_.insert(PRESENT_VELOCITY     );
        get_parameter_or("read/present_position"     , tmp,  true); if( tmp ) list_read_state_.insert(PRESENT_POSITION     );
        get_parameter_or("read/velocity_trajectory"  , tmp, false); if( tmp ) list_read_state_.insert(VELOCITY_TRAJECTORY  );
        get_parameter_or("read/position_trajectory"  , tmp, false); if( tmp ) list_read_state_.insert(POSITION_TRAJECTORY  );
        get_parameter_or("read/present_input_voltage", tmp, false); if( tmp ) list_read_state_.insert(PRESENT_INPUT_VOLTAGE);
        get_parameter_or("read/present_temperature"  , tmp, false); if( tmp ) list_read_state_.insert(PRESENT_TEMPERTURE   );
    } else {
        ratio_state_pub_ = 1; // multi_rate_readの場合は，ratio_state_pub_は1にする
        get_parameter_or("multi_rate_read/ratio/present_pwm"          , multi_rate_read_ratio_pub_[PRESENT_PWM          ],  0u);
        get_parameter_or("multi_rate_read/ratio/present_current"      , multi_rate_read_ratio_pub_[PRESENT_CURRENT      ],  1u);
        get_parameter_or("multi_rate_read/ratio/present_velocity"     , multi_rate_read_ratio_pub_[PRESENT_VELOCITY     ],  1u);
        get_parameter_or("multi_rate_read/ratio/present_position"     , multi_rate_read_ratio_pub_[PRESENT_POSITION     ],  1u);
        get_parameter_or("multi_rate_read/ratio/velocity_trajectory"  , multi_rate_read_ratio_pub_[VELOCITY_TRAJECTORY  ],  0u);
        get_parameter_or("multi_rate_read/ratio/position_trajectory"  , multi_rate_read_ratio_pub_[POSITION_TRAJECTORY  ],  0u);
        get_parameter_or("multi_rate_read/ratio/present_input_voltage", multi_rate_read_ratio_pub_[PRESENT_INPUT_VOLTAGE], 10u);
        get_parameter_or("multi_rate_read/ratio/present_temperature"  , multi_rate_read_ratio_pub_[PRESENT_TEMPERTURE   ], 10u);
    }

    ROS_INFO( "..... DynamixelHandler is initialized");
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

void DynamixelHandler::MainLoop(){
    static int cnt = -1; cnt++;
    static float rtime=0, wtime=0, num_st_suc_p=1, num_st_suc_f=1, num_st_read=1;

/* 処理時間時間の計測 */ auto wstart = system_clock::now();
    //* topicをSubscribe & Dynamixelへ目標角をWrite
    SyncWriteGoal(list_write_goal_);
    list_write_goal_.clear();
    SyncWriteGain(list_write_gain_);
    list_write_gain_.clear();
/* 処理時間時間の計測 */ wtime += duration_cast<microseconds>(system_clock::now()-wstart).count() / 1000.0;

    //* 複数周期でstateをreadする場合の処理
    if ( use_multi_rate_read_ ) {
        static const auto& r = multi_rate_read_ratio_pub_; //長いので省略
        list_read_state_.clear();
        if (r[PRESENT_PWM          ] && cnt % r[PRESENT_PWM          ] == 0) list_read_state_.insert(PRESENT_PWM          );
        if (r[PRESENT_CURRENT      ] && cnt % r[PRESENT_CURRENT      ] == 0) list_read_state_.insert(PRESENT_CURRENT      );
        if (r[PRESENT_VELOCITY     ] && cnt % r[PRESENT_VELOCITY     ] == 0) list_read_state_.insert(PRESENT_VELOCITY     );
        if (r[PRESENT_POSITION     ] && cnt % r[PRESENT_POSITION     ] == 0) list_read_state_.insert(PRESENT_POSITION     );
        if (r[VELOCITY_TRAJECTORY  ] && cnt % r[VELOCITY_TRAJECTORY  ] == 0) list_read_state_.insert(VELOCITY_TRAJECTORY  );
        if (r[POSITION_TRAJECTORY  ] && cnt % r[POSITION_TRAJECTORY  ] == 0) list_read_state_.insert(POSITION_TRAJECTORY  );
        if (r[PRESENT_INPUT_VOLTAGE] && cnt % r[PRESENT_INPUT_VOLTAGE] == 0) list_read_state_.insert(PRESENT_INPUT_VOLTAGE);
        if (r[PRESENT_TEMPERTURE   ] && cnt % r[PRESENT_TEMPERTURE   ] == 0) list_read_state_.insert(PRESENT_TEMPERTURE   );
    }

/* 処理時間時間の計測 */ auto rstart = system_clock::now();
    if ( ratio_state_pub_ && cnt % (100*ratio_state_pub_) == 0 ) CheckDynamixels(); // トルクが入っているか確認
    //* Dynamixelから状態Read & topicをPublish
    if ( !list_read_state_.empty() ) // list_read_state_が空でない場合のみ実行
    if ( ratio_state_pub_ && cnt % ratio_state_pub_ == 0 ) {// ratio_state_pub_の割合で実行
        double rate_suc_st = SyncReadState(list_read_state_);
        num_st_read++;
        num_st_suc_p += rate_suc_st > 0.0;
        num_st_suc_f += rate_suc_st > 1.0-1e-6;
        if ( rate_suc_st>0.0 ) BroadcastDxlState();
    }
    if ( ratio_error_pub_ && cnt % ratio_error_pub_ == 0 ) { // ratio_error_pub_の割合で実行
        double rate_suc_err = SyncReadHardwareErrors();
        if ( rate_suc_err>0.0) BroadcastDxlError();
    }
    if ( ratio_limit_pub_ && cnt % ratio_limit_pub_ == 0 ) { // ratio_limit_pub_
        double rate_suc_lim = SyncReadLimit(list_read_limit_); // 処理を追加する可能性を考えて，変数を別で用意する冗長な書き方をしている．
        if ( rate_suc_lim >0.0 ) BroadcastDxlLimit();
    }
    if ( ratio_gain_pub_ && cnt % ratio_gain_pub_ == 0 ) { // ratio_gain_pub_
        double rate_suc_gain = SyncReadGain(list_read_gain_);
        if ( rate_suc_gain>0.0 ) BroadcastDxlGain();
    }
    if ( ratio_goal_pub_ && cnt % ratio_goal_pub_ == 0 ) { // ratio_goal_pub_
        double rate_suc_goal = SyncReadGoal(list_read_goal_);
        if ( rate_suc_goal>0.0 ) BroadcastDxlGoal();
    }
/* 処理時間時間の計測 */ rtime += duration_cast<microseconds>(system_clock::now()-rstart).count() / 1000.0;

    //* デバック
    if ( ratio_mainloop_ !=0 )
    if ( cnt % ratio_mainloop_ == 0) {
        float partial_suc = 100*num_st_suc_p/num_st_read; 
        float full_suc = 100*num_st_suc_f/num_st_read;
        char msg[100]; sprintf(msg, "Loop [%d]: write=%.2fms read=%.2fms(p/f=%3.0f%%/%3.0f%%)",
                               cnt, wtime/ratio_mainloop_, rtime/num_st_read, partial_suc, full_suc);
        if (full_suc < 80) ROS_ERROR("%s", msg); else if (partial_suc < 99) ROS_WARN("%s", msg); else ROS_INFO("%s", msg);
        wtime = 0.0; /* mainloopで行われてる処理の計測時間を初期化 */
    }
    if ( cnt % max({(int)loop_rate_, (int)ratio_mainloop_, 10}) == 0)
        rtime = num_st_suc_p = num_st_suc_f = num_st_read=0.00001; /* stateのreadの周期で行われてる処理の初期化 */ 

	fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
}

DynamixelHandler::~DynamixelHandler(){
    ROS_INFO( "Terminating DynamixelHandler ...");
    bool do_torque_off; get_parameter_or("term/torque_auto_disable", do_torque_off, true);
    if ( do_torque_off ) for ( auto id : id_set_ ) TorqueOff(id);
    StopDynamixels();
    ROS_INFO( "  ... DynamixelHandler is terminated");
}

#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	/*Initialization*/
    auto node = std::make_shared<DynamixelHandler>();
    /*Mainloop*/
    auto timer_ = node.get()->create_wall_timer(
        1.0s/(node->loop_rate_),
        bind(&DynamixelHandler::MainLoop, node.get())
    ); // 変数に保存する必要あり
    /*Interruption*/
    rclcpp::spin(node);
    /*Termination*/
    node.reset(); // rclcpp::shutdown() の前に呼ぶ必要あり
    rclcpp::shutdown();
    return 0;
}