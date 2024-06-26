#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "dynamixel_communicator.h"
#include "dynamixel_handler/msg/dynamixel_state.hpp"
#include "dynamixel_handler/msg/dynamixel_error.hpp"
#include "dynamixel_handler/msg/dynamixel_option_config.hpp"
#include "dynamixel_handler/msg/dynamixel_option_extra.hpp"
#include "dynamixel_handler/msg/dynamixel_option_gain.hpp"
#include "dynamixel_handler/msg/dynamixel_option_limit.hpp"
#include "dynamixel_handler/msg/dynamixel_option_mode.hpp"
#include "dynamixel_handler/msg/dynamixel_option_goal.hpp"
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
        void BroadcastDxlOpt_Config(); // todo
        void BroadcastDxlOpt_Goal(); 
        void BroadcastDxlOpt_Limit();
        void BroadcastDxlOpt_Gain(); 
        void BroadcastDxlOpt_Mode(); 
        void CallBackDxlOpt_Limit (const DynamixelOptionLimit& msg); // todo
        void CallBackDxlOpt_Gain  (const DynamixelOptionGain& msg);  // todo
        void CallBackDxlOpt_Mode  (const DynamixelOptionMode& msg);  // todo
        void CallBackDxlCommand                   (const DynamixelCommand& msg);    // todo
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
        rclcpp::Publisher<DynamixelOptionLimit>::SharedPtr pub_opt_limit_;
        rclcpp::Publisher<DynamixelOptionGain>::SharedPtr pub_opt_gain_;
        rclcpp::Publisher<DynamixelOptionMode>::SharedPtr pub_opt_mode_;
        rclcpp::Publisher<DynamixelOptionGoal>::SharedPtr pub_opt_goal_;
        rclcpp::Subscription<DynamixelCommand>::SharedPtr sub_command_;
        rclcpp::Subscription<DynamixelCommandXControlPosition>::SharedPtr sub_cmd_x_pos_;
        rclcpp::Subscription<DynamixelCommandXControlVelocity>::SharedPtr sub_cmd_x_vel_;
        rclcpp::Subscription<DynamixelCommandXControlCurrent>::SharedPtr sub_cmd_x_cur_;
        rclcpp::Subscription<DynamixelCommandXControlCurrentPosition>::SharedPtr sub_cmd_x_cpos_;
        rclcpp::Subscription<DynamixelCommandXControlExtendedPosition>::SharedPtr sub_cmd_x_epos_;
        rclcpp::Subscription<DynamixelCommandPControlPosition>::SharedPtr sub_cmd_p_pos_;
        rclcpp::Subscription<DynamixelCommandPControlVelocity>::SharedPtr sub_cmd_p_vel_;
        rclcpp::Subscription<DynamixelCommandPControlCurrent>::SharedPtr sub_cmd_p_cur_;
        rclcpp::Subscription<DynamixelCommandPControlExtendedPosition>::SharedPtr sub_cmd_p_epos_;
        rclcpp::Subscription<DynamixelOptionGain>::SharedPtr sub_opt_gain_;
        rclcpp::Subscription<DynamixelOptionMode>::SharedPtr sub_opt_mode_;
        rclcpp::Subscription<DynamixelOptionLimit>::SharedPtr sub_opt_limit_;
  
        //* 各種のフラグとパラメータ
        unsigned int loop_rate_ = 50;
        unsigned int ratio_state_pub_  = 1;
        unsigned int ratio_option_pub_ = 100; // 0の時は初回のみ
        unsigned int ratio_error_pub_  = 100; // 0の時は初回のみ
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
        enum CmdValueIndex { //　cmd_values_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM     ,
            GOAL_CURRENT ,
            GOAL_VELOCITY,
            PROFILE_ACC  ,
            PROFILE_VEL  ,
            GOAL_POSITION,
            /*Indexの最大値*/_num_cmd_value
        };
        enum StValueIndex { // state_values_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          ,
            PRESENT_CURRENT      ,
            PRESENT_VELOCITY     ,
            PRESENT_POSITION     ,
            VELOCITY_TRAJECTORY  ,
            POSITION_TRAJECTORY  ,
            PRESENT_INPUT_VOLTAGE,
            PRESENT_TEMPERTURE   ,
            /*Indexの最大値*/_num_state_value
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
        enum OptLimitIndex { // opt_limit_のIndex, 各種の制限値
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
            /*Indexの最大値*/_num_opt_limit
        };
        enum OptGainIndex { // opt_gain_のIndex, 各種のゲイン値
            VELOCITY_I_GAIN     ,
            VELOCITY_P_GAIN     ,
            POSITION_D_GAIN     ,
            POSITION_I_GAIN     ,
            POSITION_P_GAIN     ,
            FEEDFORWARD_ACC_GAIN,
            FEEDFORWARD_VEL_GAIN, 
            /*Indexの最大値*/_num_opt_gain           
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
        static inline map<uint8_t, array<bool,   _num_hw_err     >> hardware_error_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身の並びはHWErrIndexに対応する
        static inline map<uint8_t, array<double, _num_state_value>> state_values_;   // 各dynamixelの id と サーボから毎周期で読み込むことができる値のマップ, 中身の並びはStValueIndexに対応する
        static inline map<uint8_t, array<double, _num_cmd_value  >> cmd_values_;     // 各dynamixelの id と サーボに毎周期で書き込むことができる値のマップ, 中身の並びはCmdValueIndexに対応する
        static inline map<uint8_t, array<double ,_num_cmd_value  >> option_goal_;    // 各dynamixelの id と サーボの各種制限値のマップ, 中身の並びはCmdValueIndexに対応する
        static inline map<uint8_t, array<double, _num_opt_limit  >> option_limit_;   // 各dynamixelの id と サーボの各種制限値のマップ, 中身の並びはOptLimitIndexに対応する 
        static inline map<uint8_t, array<int64_t,_num_opt_gain   >> option_gain_;    // 各dynamixelの id と サーボの各種制限値のマップ, 中身の並びはOptGainIndexに対応する 
        // 上記の変数を適切に使うための補助的なフラグ
        static inline map<uint8_t, double> when_op_mode_updated_; // 各dynamixelの id と op_mode_ が更新された時刻のマップ
        static inline map<uint8_t, bool> is_cmd_updated_;       // topicのcallbackによって，cmd_valuesが更新されたかどうかを示すマップ
        static inline bool has_hardware_err_ = false; // 連結しているDynamixelのうち，どれか一つでもハードウェアエラーを起こしているかどうか
        // 各周期で実行するserial通信の内容を決めるためのset
        static inline set<CmdValueIndex> list_write_cmd_ ;
        static inline set<StValueIndex>  list_read_state_;
        // 複数周期で state を read するために使う．
        static inline array<unsigned int, _num_state_value> multi_rate_read_ratio_pub_;

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
        bool WriteGains(uint8_t servo_id, array<int64_t, _num_opt_gain> gains);
        //* 連結しているDynamixelに一括で読み書きするloopで使用する機能
        template <typename Addr=AddrCommon> void SyncWriteCommandValues(set<CmdValueIndex> list_wirte_cmd);
        template <typename Addr=AddrCommon> void SyncWriteOption_Mode();  // todo 
        template <typename Addr=AddrCommon> void SyncWriteOption_Gain();  // todo 
        template <typename Addr=AddrCommon> void SyncWriteOption_Limit(); // todo 
        template <typename Addr=AddrCommon> double SyncReadStateValues(set<StValueIndex> list_read_state);
        template <typename Addr=AddrCommon> double SyncReadHardwareErrors();
        template <typename Addr=AddrCommon> double SyncReadOption_Mode(); 
        template <typename Addr=AddrCommon> double SyncReadOption_Gain(); 
        template <typename Addr=AddrCommon> double SyncReadOption_Limit();
        template <typename Addr=AddrCommon> double SyncReadOption_Goal();
        template <typename Addr=AddrCommon> void SyncStopDynamixels();
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
