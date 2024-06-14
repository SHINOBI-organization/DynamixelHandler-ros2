#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "dynamixel_communicator.h"
#include "dynamixel_handler/msg/dynamixel_state.hpp"
#include "dynamixel_handler/msg/dynamixel_error.hpp"
#include "dynamixel_handler/msg/dynamixel_config.hpp"
#include "dynamixel_handler/msg/dynamixel_extra.hpp"
#include "dynamixel_handler/msg/dynamixel_gain.hpp"
#include "dynamixel_handler/msg/dynamixel_limit.hpp"
#include "dynamixel_handler/msg/dynamixel_goal.hpp"
#include "dynamixel_handler/msg/dynamixel_command.hpp"
#include "dynamixel_handler/msg/dynamixel_command_x_control_position.hpp"
#include "dynamixel_handler/msg/dynamixel_command_x_control_velocity.hpp"
#include "dynamixel_handler/msg/dynamixel_command_x_control_current.hpp"
#include "dynamixel_handler/msg/dynamixel_command_x_control_current_position.hpp"
#include "dynamixel_handler/msg/dynamixel_command_x_control_extended_position.hpp"
#include "dynamixel_handler/msg/dynamixel_command_p_control_position.hpp"
#include "dynamixel_handler/msg/dynamixel_command_p_control_velocity.hpp"
#include "dynamixel_handler/msg/dynamixel_command_p_control_current.hpp"
#include "dynamixel_handler/msg/dynamixel_command_p_control_extended_position.hpp"
using namespace dynamixel_handler::msg;

#include <string>
using std::string;
#include <map>
using std::map;
#include <vector>
using std::vector;
#include <array>
using std::array;
#include <set>
using std::set;
#include <utility>
using std::pair;
#include <algorithm>
using std::max_element;
using std::min_element;
using std::clamp;
using std::min;
using std::max;

// 角度変換
static const double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
static double deg2rad(double deg){ return deg*DEG; }
static double rad2deg(double rad){ return rad/DEG; }
// 一定時間待つための関数
static void rsleep(int millisec) { std::this_thread::sleep_for(std::chrono::milliseconds(millisec));}
// enum でインクリメントをするため
template<typename T> T& operator ++ (T& v     ) { v = static_cast<T>(v + 1); return v;}
template<typename T> T  operator ++ (T& v, int) { T p=v; ++v; return p;}
// ROS1 のようにログを出力するためのマクロ
#define ROS_INFO(...)  RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(this->get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(this->get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(this->get_logger(), __VA_ARGS__)
// vectorやsetに値が含まれているかどうかを調べる関数
template <typename T> bool is_in(const T& val, const vector<T>& v) { return std::find(v.begin(), v.end(), val) != v.end(); }
template <typename T> bool is_in(const T& val, const    set<T>& s) { return s.find(val) != s.end(); }

/**
 * DynamixelをROSで動かすためのクラス．本pkgのメインクラス． 
 * 検出したDynamixelに関して，idのみベクトルとして保持し，
 * それ以外の情報はすべて，idをキーとしたmapに保持している．
*/
class DynamixelHandler : public rclcpp::Node {
    public:
        //* ROS 初期設定とメインループ
        DynamixelHandler();  // コンストラクタ, 初期設定を行う
        ~DynamixelHandler(); // デストラクタ,  終了処理を行う
        void MainLoop();     // メインループ

        //* ROS publishを担う関数と subscliber callback関数
        void BroadcastDxlState();
        void BroadcastDxlError();
        // void BroadcastDxlConfig(); // todo
        // void BroadcastDxlExtra();  // todo
        void BroadcastDxlLimit();
        void BroadcastDxlGoal(); 
        void BroadcastDxlGain();
        // void CallBackDxlConfig(const DynamixelConfig& msg); // todo
        // void CallBackDxlExtra (const DynamixelExtra& msg);  // todo
        void CallBackDxlLimit (const DynamixelLimit& msg);
        void CallBackDxlGoal  (const DynamixelGoal& msg); 
        void CallBackDxlGain  (const DynamixelGain& msg); 
        void CallBackDxlCommand               (const DynamixelCommand::SharedPtr msg);
        void CallBackDxlCmd_X_Position        (const DynamixelCommandXControlPosition& msg);
        void CallBackDxlCmd_X_Velocity        (const DynamixelCommandXControlVelocity& msg);
        void CallBackDxlCmd_X_Current         (const DynamixelCommandXControlCurrent& msg);
        void CallBackDxlCmd_X_CurrentPosition (const DynamixelCommandXControlCurrentPosition& msg);
        void CallBackDxlCmd_X_ExtendedPosition(const DynamixelCommandXControlExtendedPosition& msg);
        void CallBackDxlCmd_P_Position        (const DynamixelCommandPControlPosition& msg);
        void CallBackDxlCmd_P_Velocity        (const DynamixelCommandPControlVelocity& msg);
        void CallBackDxlCmd_P_Current         (const DynamixelCommandPControlCurrent& msg);
        void CallBackDxlCmd_P_ExtendedPosition(const DynamixelCommandPControlExtendedPosition& msg);
        //* ROS publisher subscriber instance
        rclcpp::Publisher<DynamixelState>::SharedPtr pub_state_;
        rclcpp::Publisher<DynamixelError>::SharedPtr pub_error_;
        rclcpp::Publisher<DynamixelLimit>::SharedPtr pub_limit_;
        rclcpp::Publisher<DynamixelGain>::SharedPtr  pub_gain_;
        rclcpp::Publisher<DynamixelGoal>::SharedPtr  pub_goal_;
        rclcpp::Subscription<DynamixelCommand>::SharedPtr sub_command_;
        rclcpp::Subscription<DynamixelCommandXControlPosition>::SharedPtr         sub_cmd_x_pos_;
        rclcpp::Subscription<DynamixelCommandXControlVelocity>::SharedPtr         sub_cmd_x_vel_;
        rclcpp::Subscription<DynamixelCommandXControlCurrent>::SharedPtr          sub_cmd_x_cur_;
        rclcpp::Subscription<DynamixelCommandXControlCurrentPosition>::SharedPtr  sub_cmd_x_cpos_;
        rclcpp::Subscription<DynamixelCommandXControlExtendedPosition>::SharedPtr sub_cmd_x_epos_;
        rclcpp::Subscription<DynamixelCommandPControlPosition>::SharedPtr         sub_cmd_p_pos_;
        rclcpp::Subscription<DynamixelCommandPControlVelocity>::SharedPtr         sub_cmd_p_vel_;
        rclcpp::Subscription<DynamixelCommandPControlCurrent>::SharedPtr          sub_cmd_p_cur_;
        rclcpp::Subscription<DynamixelCommandPControlExtendedPosition>::SharedPtr sub_cmd_p_epos_;
        rclcpp::Subscription<DynamixelGain>::SharedPtr  sub_gain_;
        rclcpp::Subscription<DynamixelLimit>::SharedPtr sub_limit_;
        rclcpp::Subscription<DynamixelGoal>::SharedPtr  sub_goal_;
  
        //* 各種のフラグとパラメータ
        unsigned int loop_rate_ = 50;
        unsigned int ratio_state_pub_  = 1;
        unsigned int ratio_error_pub_  = 100; // 0の時は初回のみ
        unsigned int ratio_limit_pub_  = 100; // 0の時は初回のみ
        unsigned int ratio_gain_pub_   = 100; // 0の時は初回のみ
        unsigned int ratio_goal_pub_   = 100; // 0の時は初回のみ
        unsigned int ratio_mainloop_   = 100; // 0の時は初回のみ
        unsigned int width_log_ = 7;
        bool use_split_write_     = false;
        bool use_split_read_      = false;
        bool use_fast_read_       = false;
        bool use_multi_rate_read_ = false;
        bool varbose_callback_  = false;
        bool varbose_write_cmd_ = false;
        bool varbose_write_opt_ = false;
        bool varbose_read_st_      = false;
        bool varbose_read_st_err_  = false;
        bool varbose_read_hwerr_   = false;
        bool varbose_read_opt_     = false;
        bool varbose_read_opt_err_ = false;

        //* Dynamixelとの通信
        DynamixelCommunicator dyn_comm_;

        //* Dynamixelを扱うための変数群 
        enum GoalValueIndex { //　goal_w_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM     ,
            GOAL_CURRENT ,
            GOAL_VELOCITY,
            PROFILE_ACC  ,
            PROFILE_VEL  ,
            GOAL_POSITION,
            /*Indexの最大値*/_num_goal     
        };
        enum StValueIndex { // state_r_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          ,
            PRESENT_CURRENT      ,
            PRESENT_VELOCITY     ,
            PRESENT_POSITION     ,
            VELOCITY_TRAJECTORY  ,
            POSITION_TRAJECTORY  ,
            PRESENT_INPUT_VOLTAGE,
            PRESENT_TEMPERTURE   ,
            /*Indexの最大値*/_num_state
           };
        enum HWErrIndex { // hardware_error_のIndex, サーボが起こしたハードウェアエラー
            INPUT_VOLTAGE     ,
            MOTOR_HALL_SENSOR ,
            OVERHEATING       ,
            MOTOR_ENCODER     ,
            ELECTRONICAL_SHOCK,
            OVERLOAD          ,
            /*Indexの最大値*/_num_hw_err
        };
        enum LimitIndex { // limit_w/r_のIndex, 各種の制限値
            NONE = -1, // Indexには使わない特殊値
            TEMPERATURE_LIMIT ,
            MAX_VOLTAGE_LIMIT ,
            MIN_VOLTAGE_LIMIT ,
            PWM_LIMIT         ,
            CURRENT_LIMIT     ,
            ACCELERATION_LIMIT,
            VELOCITY_LIMIT    ,
            MAX_POSITION_LIMIT,
            MIN_POSITION_LIMIT,
            /*Indexの最大値*/_num_limit
        };
        enum GainIndex { // gain_w/r_のIndex, 各種のゲイン値
            VELOCITY_I_GAIN     ,
            VELOCITY_P_GAIN     ,
            POSITION_D_GAIN     ,
            POSITION_I_GAIN     ,
            POSITION_P_GAIN     ,
            FEEDFORWARD_ACC_GAIN,
            FEEDFORWARD_VEL_GAIN, 
            /*Indexの最大値*/_num_gain           
        };
        // 連結したサーボの基本情報
        set<uint8_t> id_set_; // chained dynamixel id list
        map<uint8_t, uint16_t> model_; // 各dynamixelの id と model のマップ
        map<uint8_t, uint16_t> series_; // 各dynamixelの id と series のマップ
        map<uint8_t, size_t> num_;  // 各dynamixelの series と　個数のマップ 無くても何とかなるけど, 効率を考えて保存する
        // 連結しているサーボの個々の状態を保持するmap
        static inline map<uint8_t, bool> tq_mode_;    // 各dynamixelの id と トルクON/OFF のマップ
        static inline map<uint8_t, uint8_t> op_mode_; // 各dynamixelの id と 制御モード のマップ
        static inline map<uint8_t, uint8_t> dv_mode_; // 各dynamixelの id と ドライブモード のマップ
        static inline map<uint8_t, array<bool,   _num_hw_err>> hardware_error_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身の並びはHWErrIndexに対応する
        static inline map<uint8_t, array<double, _num_state >> state_r_;  // 各dynamixelの id と サーボから読み込んだ状態のマップ
        static inline map<uint8_t, array<double, _num_goal  >> goal_w_; // 各dynamixelの id と サーボへ書き込む指令のマップ
        static inline map<uint8_t, array<double ,_num_goal  >> goal_r_; // 各dynamixelの id と サーボから読み込んだ指令のマップ
        static inline map<uint8_t, array<uint16_t,_num_gain  >> gain_w_;    // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<uint16_t,_num_gain  >> gain_r_;    // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<double, _num_limit >> limit_w_;   // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<double, _num_limit >> limit_r_;   // 各dynamixelの id と サーボ

        // 上記の変数を適切に使うための補助的なフラグ
        static inline map<uint8_t, double> when_op_mode_updated_; // 各dynamixelの id と op_mode_ が更新された時刻のマップ
        static inline map<uint8_t, bool> is_goal_updated_;       // topicのcallbackによって，goal_w_が更新されたかどうかを示すマップ
        static inline map<uint8_t, bool> is_gain_updated_;       // topicのcallbackによって，limit_w_が更新されたかどうかを示すマップ
        static inline map<uint8_t, bool> is_limit_updated_;       // topicのcallbackによって，limit_w_が更新されたかどうかを示すマップ
        static inline bool has_hardware_err_ = false; // 連結しているDynamixelのうち，どれか一つでもハードウェアエラーを起こしているかどうか
        // 各周期で実行するserial通信の内容を決めるためのset
        static inline set<GoalValueIndex> list_write_goal_;
        static inline set<GainIndex>      list_write_gain_;
        static inline set<LimitIndex>     list_write_limit_;
        static inline set<StValueIndex>   list_read_state_ = {PRESENT_POSITION, PRESENT_VELOCITY, PRESENT_CURRENT, PRESENT_PWM, VELOCITY_TRAJECTORY, POSITION_TRAJECTORY, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERTURE};
        static inline set<GoalValueIndex> list_read_goal_  = {GOAL_POSITION, GOAL_VELOCITY, GOAL_CURRENT, GOAL_PWM, PROFILE_ACC, PROFILE_VEL};
        static inline set<GainIndex>      list_read_gain_  = {VELOCITY_I_GAIN, VELOCITY_P_GAIN, POSITION_D_GAIN, POSITION_I_GAIN, POSITION_P_GAIN, FEEDFORWARD_ACC_GAIN, FEEDFORWARD_VEL_GAIN};
        static inline set<LimitIndex>     list_read_limit_ = {TEMPERATURE_LIMIT, MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT, PWM_LIMIT, CURRENT_LIMIT, ACCELERATION_LIMIT, VELOCITY_LIMIT, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT};
        // 複数周期で state を read するために使う．
        static inline array<unsigned int, _num_state>    multi_rate_read_ratio_pub_;

        //* 単体通信を組み合わせた上位機能
        uint8_t ScanDynamixels(uint8_t id_min, uint8_t id_max, uint32_t num_expected, uint32_t time_retry_ms);
        bool ClearHardwareError(uint8_t servo_id);
        bool ChangeOperatingMode(uint8_t servo_id, DynamixelOperatingMode mode);
        bool TorqueOn(uint8_t servo_id);
        bool TorqueOff(uint8_t servo_id);
        //* Dynamixel単体との通信による下位機能
        uint8_t ReadHardwareError(uint8_t servo_id);
        bool    ReadTorqueEnable(uint8_t servo_id);
        double  ReadPresentPWM(uint8_t servo_id);
        double  ReadPresentCurrent(uint8_t servo_id);
        double  ReadPresentVelocity(uint8_t servo_id);
        double  ReadPresentPosition(uint8_t servo_id);
        double  ReadGoalPWM(uint8_t servo_id);
        double  ReadGoalCurrent(uint8_t servo_id);
        double  ReadGoalVelocity(uint8_t servo_id);
        double  ReadGoalPosition(uint8_t servo_id);
        double  ReadProfileAcc(uint8_t servo_id);
        double  ReadProfileVel(uint8_t servo_id);
        double  ReadHomingOffset(uint8_t servo_id);
        uint8_t ReadOperatingMode(uint8_t servo_id);
        uint8_t ReadDriveMode(uint8_t servo_id);
        bool WriteTorqueEnable(uint8_t servo_id, bool enable);
        bool WriteGoalPWM(uint8_t servo_id, double pwm);
        bool WriteGoalCurrent(uint8_t servo_id, double current);
        bool WriteGoalVelocity(uint8_t servo_id, double velocity);
        bool WriteGoalPosition(uint8_t servo_id, double position);
        bool WriteProfileAcc(uint8_t servo_id, double acceleration);
        bool WriteProfileVel(uint8_t servo_id, double velocity);
        bool WriteHomingOffset(uint8_t servo_id, double offset);
        bool WriteOperatingMode(uint8_t servo_id, uint8_t mode);
        bool WriteBusWatchdog(uint8_t servo_id, double time);
        bool WriteGains(uint8_t servo_id, array<uint16_t, _num_gain> gains);
        //* 連結しているDynamixelに一括で読み書きするloopで使用する機能
        template <typename Addr=AddrCommon> void SyncWriteGoal(set<GoalValueIndex> list_write_goal);
        template <typename Addr=AddrCommon> void SyncWriteGain (set<GainIndex> list_write_gain); 
        template <typename Addr=AddrCommon> void SyncWriteLimit(set<LimitIndex> list_write_limit);
        template <typename Addr=AddrCommon> double SyncReadHardwareErrors();
        template <typename Addr=AddrCommon> double SyncReadState(set<StValueIndex> list_read_state);
        template <typename Addr=AddrCommon> double SyncReadGoal(set<GoalValueIndex> list_read_goal);
        template <typename Addr=AddrCommon> double SyncReadGain(set<GainIndex> list_read_gain); 
        template <typename Addr=AddrCommon> double SyncReadLimit(set<LimitIndex> list_read_limit);
        template <typename Addr=AddrCommon> void StopDynamixels();
        template <typename Addr=AddrCommon> void CheckDynamixels();
};

// ちょっとした文字列の整形を行う補助関数

using std::setw;
using std::prev;
using std::next;

static string control_table_layout(int width, const map<uint8_t, vector<int64_t>>& id_data_map, const vector<DynamixelAddress>& dp_list, const string& header=""){
    std::stringstream ss;
    ss << header;
    if (id_data_map.empty()) return ss.str();
    // width 以上のID数がある場合は，再帰させることで，縦に並べる
	width = min(width, (int)id_data_map.size());
    map<uint8_t, vector<int64_t>> first(id_data_map.begin(), prev(id_data_map.end(), id_data_map.size() - width));
    map<uint8_t, vector<int64_t>> second(next(id_data_map.begin(), width), id_data_map.end());
    // 分割した前半を処理
    ss << "\n" << "ADDR|"; 
    for (const auto& [id, data] : first) ss << "  [" << setw(3) << (int)id << "] "; 
    ss << "\n";
    for (size_t i = 0; i < dp_list.size(); ++i) {
        ss << "-" << setw(3) << dp_list[i].address() << "|" ;
        for (const auto& [id, data] : first) ss << std::setfill(' ') << setw(7) << data[i] << " "; 
        ss << "\n";
    }
    // 分割した前半に後半を処理したものを追加する
    return ss.str() + control_table_layout(width, second, dp_list);
}

static string id_list_layout(const vector<uint8_t>& id_list, const string& header=""){
    std::stringstream ss;
    ss << header << "\n";
    ss << " ID : [ "; 
    for ( auto id : id_list ) {
        ss << (int)id; 
        if ( id != id_list.back()) ss << ", ";
    }
    ss << " ]";
    return ss.str();
}

#endif /* DYNAMIXEL_HANDLER_H_ */
