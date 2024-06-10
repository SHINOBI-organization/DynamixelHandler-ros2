#include "dynamixel_handler.hpp"
#include "rclcpp/rclcpp.hpp"

using std::bind;
using std::placeholders::_1;

DynamixelHandler::DynamixelHandler() : Node("dynamixel_handler") {
    ROS_INFO( "Initializing DynamixelHandler .....");

    // 通信の開始
    this->declare_parameter("baudrate", 57600);
    this->declare_parameter("latency_timer", 16);
    this->declare_parameter("device_name", "/dev/ttyUSB0");
    int baudrate       = get_parameter("baudrate").as_int();
    int latency_timer  = get_parameter("latency_timer").as_int();
    string device_name = get_parameter("device_name").as_string();

    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer); fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
    if ( !dyn_comm_.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        throw std::runtime_error("Initialization failed");
    } 

    // serial通信のvarbose設定
    this->declare_parameter("dyn_comm/varbose", false);
    bool serial_varbose = get_parameter("dyn_comm/varbose").as_bool();
    dyn_comm_.set_varbose(serial_varbose); fflush(stdout);

    // serial通信のretry設定
    this->declare_parameter("dyn_comm/retry_num", 5);
    this->declare_parameter("dyn_comm/inerval_msec", 10);
    int num_try       = get_parameter("dyn_comm/retry_num").as_int();
    int msec_interval = get_parameter("dyn_comm/inerval_msec").as_int();
    dyn_comm_.set_retry_config(num_try, msec_interval); fflush(stdout);

    // main loop の設定
    this->declare_parameter("loop_rate",           50);
    this->declare_parameter("ratio/read_state",     1);
    this->declare_parameter("ratio/read_error",   100);
    this->declare_parameter("ratio/read_limit",    0);
    this->declare_parameter("ratio/read_gain",     0);
    this->declare_parameter("ratio/read_mode",     0);
    this->declare_parameter("ratio/read_goal",     0);
    this->declare_parameter("ratio/varbose_loop", 100);
    this->declare_parameter("max_log_width",        7);
    this->declare_parameter("use/split_write",     false);
    this->declare_parameter("use/split_read",      false);
    this->declare_parameter("use/fast_read",        true);
    this->declare_parameter("use/multi_rate_read", false);
    this->declare_parameter("varbose/callback",           false);
    this->declare_parameter("varbose/write_commad",       false);
    this->declare_parameter("varbose/write_option",       false);
    this->declare_parameter("varbose/read_state/raw",     false);
    this->declare_parameter("varbose/read_state/err",     false);
    this->declare_parameter("varbose/read_options/raw",    false);
    this->declare_parameter("varbose/read_options/err",    false);
    this->declare_parameter("varbose/read_hardware_error",false);
    loop_rate_        = get_parameter("loop_rate"         ).as_int();
    ratio_state_pub_  = get_parameter("ratio/read_state"  ).as_int();
    ratio_error_pub_  = get_parameter("ratio/read_error"  ).as_int();
    ratio_limit_pub_  = get_parameter("ratio/read_limit"  ).as_int();
    ratio_gain_pub_   = get_parameter("ratio/read_gain"   ).as_int();
    ratio_mode_pub_   = get_parameter("ratio/read_mode"   ).as_int();
    ratio_goal_pub_   = get_parameter("ratio/read_goal"   ).as_int();
    ratio_mainloop_   = get_parameter("ratio/varbose_loop").as_int();
    width_log_        = get_parameter("max_log_width"     ).as_int();
    use_split_write_     = get_parameter("use/split_write"    ).as_bool();
    use_split_read_      = get_parameter("use/split_read"     ).as_bool();
    use_fast_read_       = get_parameter("use/fast_read"      ).as_bool();
    use_multi_rate_read_ = get_parameter("use/multi_rate_read").as_bool();
    varbose_callback_     = get_parameter("varbose/callback"           ).as_bool();
    varbose_write_cmd_    = get_parameter("varbose/write_commad"       ).as_bool();
    varbose_write_opt_    = get_parameter("varbose/write_option"       ).as_bool();
    varbose_read_st_      = get_parameter("varbose/read_state/raw"     ).as_bool();
    varbose_read_st_err_  = get_parameter("varbose/read_state/err"     ).as_bool();
    varbose_read_opt_     = get_parameter("varbose/read_options/raw"    ).as_bool();
    varbose_read_opt_err_ = get_parameter("varbose/read_options/err"    ).as_bool();
    varbose_read_hwerr_   = get_parameter("varbose/read_hardware_error").as_bool();

    // id_listの作成
    this->declare_parameter("init/expected_servo_num", 0); // 0のときはチェックしない
    this->declare_parameter("init/auto_search_retry_times", 5);
    this->declare_parameter("init/auto_search_min_id", 1);
    this->declare_parameter("init/auto_search_max_id", 35);
    int num_expexted = get_parameter("init/expected_servo_num").as_int();
    int times_retry = get_parameter("init/auto_search_retry_times").as_int();
    int id_min = get_parameter("init/auto_search_min_id").as_int();
    int id_max = get_parameter("init/auto_search_max_id").as_int();

    ROS_INFO(" Auto scanning Dynamixel (id range [%d] to [%d]) ...", id_min, id_max);
    if ( num_expexted>0 ) ROS_INFO("Expected number of Dynamixel is [%d]", num_expexted);
    else ROS_WARN("Expected number of Dynamixel is not set. Free number of Dynamixel is allowed");
    auto num_found = ScanDynamixels(id_min, id_max, num_expexted, times_retry);
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        throw std::runtime_error("Initialization failed");
    }
    if( num_expexted>0 && num_expexted!=num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]. please check & retry", num_expexted, num_found);
        throw std::runtime_error("Initialization failed");
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
    sub_command_    = create_subscription<DynamixelCommand>("dynamixel/command", 4, bind(&DynamixelHandler::CallBackDxlCommand, this, _1));
    sub_gain_ = create_subscription<DynamixelGain> ("dynamixel/gain/w", 4, bind(&DynamixelHandler::CallBackDxlGain, this, _1));
    sub_mode_ = create_subscription<DynamixelMode> ("dynamixel/mode/w", 4, bind(&DynamixelHandler::CallBackDxlMode, this, _1));
    sub_limit_= create_subscription<DynamixelLimit>("dynamixel/limit/w",4, bind(&DynamixelHandler::CallBackDxlLimit, this, _1));
    sub_goal_ = create_subscription<DynamixelGoal> ("dynamixel/goal/w", 4, bind(&DynamixelHandler::CallBackDxlGoal, this, _1));

    pub_state_     = create_publisher<DynamixelState>("dynamixel/state", 4);
    pub_error_     = create_publisher<DynamixelError>("dynamixel/error", 4);
    pub_gain_  = create_publisher<DynamixelGain> ("dynamixel/gain/r",  4);
    pub_mode_  = create_publisher<DynamixelMode> ("dynamixel/mode/r",  4);
    pub_limit_ = create_publisher<DynamixelLimit>("dynamixel/limit/r", 4);
    pub_goal_  = create_publisher<DynamixelGoal> ("dynamixel/goal/r",  4);


    // 状態のreadの前にやるべき初期化
    this->declare_parameter("init/homing_offset", 0.0);
    this->declare_parameter("init/profile_acceleration", 600.0*DEG);
    this->declare_parameter("init/profile_velocity", 100.0*DEG);
    for (auto id : id_set_) {
        WriteBusWatchdog (id, 0.0 );
        WriteHomingOffset(id, 0.0 ); // 設定ファイルからとってこれるようにする
        WriteProfileAcc(id, 600.0*DEG ); //  設定ファイルからとってこれるようにする
        WriteProfileVel(id, 100.0*DEG ); //  設定ファイルからとってこれるようにする
    }

    // 最初の一回は全ての情報をread & publish
    ROS_INFO( " Reading present dynamixel state  ...");
    // dyn_comm_.set_latency_timer(8);
    while ( rclcpp::ok() && SyncReadHardwareErrors() < 1.0-1e-6 ) rsleep(500); BroadcastDxlError(); ROS_INFO( "  ... error read done ");
    while ( rclcpp::ok() && SyncReadLimit() < 1.0-1e-6 ) rsleep(500); BroadcastDxlLimit(); ROS_INFO( "  ... limit read done ");
    while ( rclcpp::ok() && SyncReadGain()  < 1.0-1e-6 ) rsleep(500); BroadcastDxlGain();  ROS_INFO( "  ... gain read done ");
    while ( rclcpp::ok() && SyncReadMode()  < 1.0-1e-6 ) rsleep(500); BroadcastDxlMode();  ROS_INFO( "  ... mode read done ");
    // while ( ros::ok() && SyncReadConfig()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlConfig();
    // while ( ros::ok() && SyncReadExtra()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlExtra();
    // dyn_comm_.set_latency_timer(latency_timer);
    for (auto id : id_set_) {     // cmd_values_の内部の情報の初期化, cmd_values_は sync read する関数を持ってないので以下の様に手動で．
        op_mode_[id] = ReadOperatingMode(id);
        cmd_values_[id][GOAL_PWM]      = ReadGoalPWM(id);
        cmd_values_[id][GOAL_CURRENT]  = ReadGoalCurrent(id);
        cmd_values_[id][GOAL_VELOCITY] = ReadGoalVelocity(id);
        cmd_values_[id][PROFILE_ACC]   = ReadProfileAcc(id);
        cmd_values_[id][PROFILE_VEL]   = ReadProfileVel(id);
        cmd_values_[id][GOAL_POSITION] = ReadGoalPosition(id);
    } ROS_INFO( "  ... goal read done ");

    // 状態のreadの後にやるべき初期化
    ROS_INFO( " Initializing dynamixel state  ...");
    this->declare_parameter("init/hardware_error_auto_clean", true);
    this->declare_parameter("init/torque_auto_enable", true);
    bool do_clean_hwerr = get_parameter("init/hardware_error_auto_clean").as_bool();
    bool do_torque_on   = get_parameter("init/torque_auto_enable").as_bool();
    for (auto id : id_set_) {
        if ( do_clean_hwerr ) ClearHardwareError(id); // 現在の状態を変えない
        if ( do_torque_on )   TorqueOn(id);           // 現在の状態を変えない
    } ROS_INFO( "  ... state initialization done ");

    //  readする情報の設定
    if ( !use_multi_rate_read_ ) {
        this->declare_parameter("read/present_pwm",           false);
        this->declare_parameter("read/present_current",        true);
        this->declare_parameter("read/present_velocity",       true);
        this->declare_parameter("read/present_position",       true);
        this->declare_parameter("read/velocity_trajectory",   false);
        this->declare_parameter("read/position_trajectory",   false);
        this->declare_parameter("read/present_input_voltage", false);
        this->declare_parameter("read/present_temperature",   false);
        if( get_parameter("read/present_pwm"          ).as_bool() ) list_read_state_.insert(PRESENT_PWM          );
        if( get_parameter("read/present_current"      ).as_bool() ) list_read_state_.insert(PRESENT_CURRENT      );
        if( get_parameter("read/present_velocity"     ).as_bool() ) list_read_state_.insert(PRESENT_VELOCITY     );
        if( get_parameter("read/present_position"     ).as_bool() ) list_read_state_.insert(PRESENT_POSITION     );
        if( get_parameter("read/velocity_trajectory"  ).as_bool() ) list_read_state_.insert(VELOCITY_TRAJECTORY  );
        if( get_parameter("read/position_trajectory"  ).as_bool() ) list_read_state_.insert(POSITION_TRAJECTORY  );
        if( get_parameter("read/present_input_voltage").as_bool() ) list_read_state_.insert(PRESENT_INPUT_VOLTAGE);
        if( get_parameter("read/present_temperature"  ).as_bool() ) list_read_state_.insert(PRESENT_TEMPERTURE   );
    } else {
        ratio_state_pub_ = 1; // multi_rate_readの場合は，ratio_state_pub_は1にする
        this->declare_parameter("multi_rate_read/ratio/present_pwm",            0);
        this->declare_parameter("multi_rate_read/ratio/present_current",        1);
        this->declare_parameter("multi_rate_read/ratio/present_velocity",       1);
        this->declare_parameter("multi_rate_read/ratio/present_position",       1);
        this->declare_parameter("multi_rate_read/ratio/velocity_trajectory",    0);
        this->declare_parameter("multi_rate_read/ratio/position_trajectory",    0);
        this->declare_parameter("multi_rate_read/ratio/present_input_voltage", 10);
        this->declare_parameter("multi_rate_read/ratio/present_temperature",   10);
        multi_rate_read_ratio_pub_[PRESENT_PWM          ] = get_parameter("multi_rate_read/ratio/present_pwm"          ).as_int();
        multi_rate_read_ratio_pub_[PRESENT_CURRENT      ] = get_parameter("multi_rate_read/ratio/present_current"      ).as_int();
        multi_rate_read_ratio_pub_[PRESENT_VELOCITY     ] = get_parameter("multi_rate_read/ratio/present_velocity"     ).as_int();
        multi_rate_read_ratio_pub_[PRESENT_POSITION     ] = get_parameter("multi_rate_read/ratio/present_position"     ).as_int();
        multi_rate_read_ratio_pub_[VELOCITY_TRAJECTORY  ] = get_parameter("multi_rate_read/ratio/velocity_trajectory"  ).as_int();
        multi_rate_read_ratio_pub_[POSITION_TRAJECTORY  ] = get_parameter("multi_rate_read/ratio/position_trajectory"  ).as_int();
        multi_rate_read_ratio_pub_[PRESENT_INPUT_VOLTAGE] = get_parameter("multi_rate_read/ratio/present_input_voltage").as_int();
        multi_rate_read_ratio_pub_[PRESENT_TEMPERTURE   ] = get_parameter("multi_rate_read/ratio/present_temperature"  ).as_int();
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
    SyncWriteCommand(list_write_cmd_);
    list_write_cmd_.clear();
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
        double rate_suc_lim = SyncReadLimit(); // 処理を追加する可能性を考えて，変数を別で用意する冗長な書き方をしている．
        if ( rate_suc_lim >0.0 ) BroadcastDxlLimit();
    }
    if ( ratio_gain_pub_ && cnt % ratio_gain_pub_ == 0 ) { // ratio_gain_pub_
        double rate_suc_gain = SyncReadGain();
        if ( rate_suc_gain>0.0 ) BroadcastDxlGain();
    }
    if ( ratio_mode_pub_ && cnt % ratio_mode_pub_ == 0 ) { // ratio_mode_pub_
        double rate_suc_mode = SyncReadMode();
        if ( rate_suc_mode>0.0 ) BroadcastDxlMode();
    }
    if ( ratio_goal_pub_ && cnt % ratio_goal_pub_ == 0 ) { // ratio_goal_pub_
        double rate_suc_goal = SyncReadGoal();
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
    this->declare_parameter("term/torque_auto_disable", true);
    bool do_torque_off  = get_parameter("term/torque_auto_disable").as_bool();
    if ( do_torque_off ) for ( auto id : id_set_ ) TorqueOff(id);
    SyncStopDynamixels();
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