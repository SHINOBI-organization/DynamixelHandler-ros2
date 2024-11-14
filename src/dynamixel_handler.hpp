#ifndef DYNAMIXEL_HANDLER_H
#define DYNAMIXEL_HANDLER_H

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_communicator.h"

#include "dynamixel_handler/msg/dxl_states.hpp"
#include "dynamixel_handler/msg/dxl_commands_x.hpp"
#include "dynamixel_handler/msg/dxl_commands_p.hpp"

#include "dynamixel_handler/msg/dynamixel_status.hpp"
#include "dynamixel_handler/msg/dynamixel_present.hpp"
#include "dynamixel_handler/msg/dynamixel_goal.hpp"
#include "dynamixel_handler/msg/dynamixel_gain.hpp"
#include "dynamixel_handler/msg/dynamixel_error.hpp"
#include "dynamixel_handler/msg/dynamixel_limit.hpp"
#include "dynamixel_handler/msg/dynamixel_extra.hpp"

#include "dynamixel_handler/msg/dynamixel_debug.hpp"

#include "dynamixel_handler/msg/dynamixel_common_cmd.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_velocity.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler/msg/dynamixel_control_p_velocity.hpp"
#include "dynamixel_handler/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler/msg/dynamixel_control_p_extended_position.hpp"
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
        DynamixelDebug BroadcastDebug();
        DynamixelStatus BroadcastState_Status();
        DynamixelPresent BroadcastState_Present();
        DynamixelGoal BroadcastState_Goal(); 
        DynamixelGain BroadcastState_Gain();
        DynamixelLimit BroadcastState_Limit();
        DynamixelError BroadcastState_Error();
        // void BroadcastStateExtra();  // todo
        void CallbackCmd_Common                (const DynamixelCommonCmd& msg);
        void CallbackCmd_X_Pwm                 (const DynamixelControlXPwm& msg);
        void CallbackCmd_X_Current             (const DynamixelControlXCurrent& msg);
        void CallbackCmd_X_Velocity            (const DynamixelControlXVelocity& msg);
        void CallbackCmd_X_Position            (const DynamixelControlXPosition& msg);
        void CallbackCmd_X_ExtendedPosition    (const DynamixelControlXExtendedPosition& msg);
        void CallbackCmd_X_CurrentBasePosition (const DynamixelControlXCurrentBasePosition& msg);
        void CallbackCmd_P_Pwm             (const DynamixelControlPPwm& msg);
        void CallbackCmd_P_Current         (const DynamixelControlPCurrent& msg);
        void CallbackCmd_P_Velocity        (const DynamixelControlPVelocity& msg);
        void CallbackCmd_P_Position        (const DynamixelControlPPosition& msg);
        void CallbackCmd_P_ExtendedPosition(const DynamixelControlPExtendedPosition& msg);
        void CallbackCmd_Status (const DynamixelStatus& msg); 
        void CallbackCmd_Goal   (const DynamixelGoal& msg); 
        void CallbackCmd_Gain   (const DynamixelGain& msg); 
        void CallbackCmd_Limit  (const DynamixelLimit& msg);
        // void CallbackExtra (const DynamixelExtra& msg);  // todo
        void CallbackCmdsX               (const DxlCommandsX::SharedPtr msg);
        void CallbackCmdsP               (const DxlCommandsP::SharedPtr msg);

        //* ROS publisher subscriber instance
        rclcpp::Publisher<DynamixelDebug>::SharedPtr   pub_debug_;
        rclcpp::Publisher<DynamixelStatus>::SharedPtr  pub_status_;
        rclcpp::Publisher<DynamixelPresent>::SharedPtr pub_present_;
        rclcpp::Publisher<DynamixelGoal>::SharedPtr    pub_goal_;
        rclcpp::Publisher<DynamixelGain>::SharedPtr    pub_gain_;
        rclcpp::Publisher<DynamixelLimit>::SharedPtr   pub_limit_;
        rclcpp::Publisher<DynamixelError>::SharedPtr   pub_error_;
        rclcpp::Publisher<DxlStates>::SharedPtr  pub_dxl_states_;
        rclcpp::Subscription<DynamixelCommonCmd>::SharedPtr sub_common_;
        rclcpp::Subscription<DynamixelControlXPwm>::SharedPtr                 sub_ctrl_x_pwm_;
        rclcpp::Subscription<DynamixelControlXCurrent>::SharedPtr             sub_ctrl_x_cur_;
        rclcpp::Subscription<DynamixelControlXVelocity>::SharedPtr            sub_ctrl_x_vel_;
        rclcpp::Subscription<DynamixelControlXPosition>::SharedPtr            sub_ctrl_x_pos_;
        rclcpp::Subscription<DynamixelControlXExtendedPosition>::SharedPtr    sub_ctrl_x_epos_;
        rclcpp::Subscription<DynamixelControlXCurrentBasePosition>::SharedPtr sub_ctrl_x_cpos_;
        rclcpp::Subscription<DynamixelControlPPwm>::SharedPtr              sub_ctrl_p_pwm_;
        rclcpp::Subscription<DynamixelControlPCurrent>::SharedPtr          sub_ctrl_p_cur_;
        rclcpp::Subscription<DynamixelControlPVelocity>::SharedPtr         sub_ctrl_p_vel_;
        rclcpp::Subscription<DynamixelControlPPosition>::SharedPtr         sub_ctrl_p_pos_;
        rclcpp::Subscription<DynamixelControlPExtendedPosition>::SharedPtr sub_ctrl_p_epos_;
        rclcpp::Subscription<DynamixelStatus>::SharedPtr  sub_status_;
        rclcpp::Subscription<DynamixelGoal>::SharedPtr    sub_goal_;
        rclcpp::Subscription<DynamixelGain>::SharedPtr    sub_gain_;
        rclcpp::Subscription<DynamixelLimit>::SharedPtr   sub_limit_;
        rclcpp::Subscription<DxlCommandsX>::SharedPtr sub_dxl_x_cmds_;
        rclcpp::Subscription<DxlCommandsP>::SharedPtr sub_dxl_p_cmds_;
  
        //* 各種のフラグとパラメータ
        unsigned int  loop_rate_ = 50;
        unsigned int  ratio_mainloop_   = 100; // 0の時は初回のみ
        unsigned int  auto_remove_count_ = 0;
        unsigned int  width_log_ = 7;
        bool use_split_write_     = false;
        bool use_split_read_      = false;
        bool use_fast_read_       = false;
        map<string, bool> verbose_; // 各種のverboseフラグ
                    bool  verbose_callback_ = false;

        //* Dynamixelとの通信
        DynamixelCommunicator dyn_comm_;

        //* Dynamixelを扱うための変数群 
        enum GoalIndex { //　goal_w_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM     ,
            GOAL_CURRENT ,
            GOAL_VELOCITY,
            PROFILE_ACC  ,
            PROFILE_VEL  ,
            GOAL_POSITION,
            /*Indexの最大値*/_num_goal     
        };
        enum PresentIndex { // state_r_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          ,
            PRESENT_CURRENT      ,
            PRESENT_VELOCITY     ,
            PRESENT_POSITION     ,
            VELOCITY_TRAJECTORY  ,
            POSITION_TRAJECTORY  ,
            PRESENT_INPUT_VOLTAGE,
            PRESENT_TEMPERATURE  ,
            /*Indexの最大値*/_num_state
        };
        enum HWErrIndex { // hardware_error_のIndex, サーボが起こしたハードウェアエラー
            INPUT_VOLTAGE    ,
            MOTOR_HALL_SENSOR,
            OVERHEATING      ,
            MOTOR_ENCODER    ,
            ELECTRONICAL_SHOCK ,
            OVERLOAD         ,
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
        map<uint8_t, size_t  > num_;  // 各dynamixelの series と　個数のマップ 無くても何とかなるけど, 効率を考えて保存する
        map<uint8_t, uint64_t> ping_err_; // 各dynamixelの id と 連続でpingに応答しなかった回数のマップ
        // 連結しているサーボの個々の状態を保持するmap
        static inline map<uint8_t, bool> tq_mode_;    // 各dynamixelの id と トルクON/OFF のマップ
        static inline map<uint8_t, uint8_t> op_mode_; // 各dynamixelの id と 制御モード のマップ
        static inline map<uint8_t, uint8_t> dv_mode_; // 各dynamixelの id と ドライブモード のマップ
        static inline map<uint8_t, array<bool,   _num_hw_err>> hardware_error_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身の並びはHWErrIndexに対応する
        static inline map<uint8_t, array<double, _num_state >> state_r_;  // 各dynamixelの id と サーボから読み込んだ状態のマップ
        static inline map<uint8_t, array<double, _num_goal  >> goal_w_; // 各dynamixelの id と サーボへ書き込む指令のマップ
        static inline map<uint8_t, array<double ,_num_goal  >> goal_r_; // 各dynamixelの id と サーボから読み込んだ指令のマップ
        static inline map<uint8_t, array<uint16_t,_num_gain >> gain_w_;    // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<uint16_t,_num_gain >> gain_r_;    // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<double, _num_limit >> limit_w_;   // 各dynamixelの id と サーボ
        static inline map<uint8_t, array<double, _num_limit >> limit_r_;   // 各dynamixelの id と サーボ

        // 上記の変数を適切に使うための補助的なフラグ
        static inline map<uint8_t, double> when_op_mode_updated_; // 各dynamixelの id と op_mode_ が更新された時刻のマップ
        static inline map<uint8_t, bool> is_goal_updated_;       // topicのcallbackによって，goal_w_が更新されたかどうかを示すマップ
        static inline map<uint8_t, bool> is_gain_updated_;       // topicのcallbackによって，limit_w_が更新されたかどうかを示すマップ
        static inline map<uint8_t, bool> is_limit_updated_;       // topicのcallbackによって，limit_w_が更新されたかどうかを示すマップ
        static inline bool has_hardware_err_ = false; // 連結しているDynamixelのうち，どれか一つでもハードウェアエラーを起こしているかどうか
        // 各周期で実行するserial通信の内容を決めるためのset
        static inline set<GoalIndex   > list_write_goal_;
        static inline set<GainIndex   > list_write_gain_;
        static inline set<LimitIndex  > list_write_limit_;
        static inline set<PresentIndex> list_read_present_ = {PRESENT_POSITION, PRESENT_VELOCITY, PRESENT_CURRENT, PRESENT_PWM, VELOCITY_TRAJECTORY, POSITION_TRAJECTORY, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE};
        static inline set<GoalIndex   > list_read_goal_    = {GOAL_POSITION, GOAL_VELOCITY, GOAL_CURRENT, GOAL_PWM, PROFILE_ACC, PROFILE_VEL};
        static inline set<GainIndex   > list_read_gain_    = {VELOCITY_I_GAIN, VELOCITY_P_GAIN, POSITION_D_GAIN, POSITION_I_GAIN, POSITION_P_GAIN, FEEDFORWARD_ACC_GAIN, FEEDFORWARD_VEL_GAIN};
        static inline set<LimitIndex  > list_read_limit_   = {TEMPERATURE_LIMIT, MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT, PWM_LIMIT, CURRENT_LIMIT, ACCELERATION_LIMIT, VELOCITY_LIMIT, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT};
        // read & publish の周期を決めるためのmap
        static inline map<string, unsigned int>       pub_ratio_; // present value以外の周期
        static inline array<unsigned int, _num_state> pub_ratio_present_;

        //* 単体通信を組み合わせた上位機能
        uint8_t ScanDynamixels(uint8_t id_min, uint8_t id_max, uint32_t num_expected, uint32_t time_retry_ms);
        void RemoveDynamixel(uint8_t servo_id);
        bool addDynamixel(uint8_t servo_id);
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
        double  ReadBusWatchdog(uint8_t servo_id);
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
        template <typename Addr=AddrCommon> void SyncWriteGoal(set<GoalIndex> list_write_goal);
        template <typename Addr=AddrCommon> void SyncWriteGain (set<GainIndex> list_write_gain); 
        template <typename Addr=AddrCommon> void SyncWriteLimit(set<LimitIndex> list_write_limit);
        template <typename Addr=AddrCommon> double SyncReadPresent(set<PresentIndex> list_read_state);
        template <typename Addr=AddrCommon> double SyncReadGoal(set<GoalIndex> list_read_goal);
        template <typename Addr=AddrCommon> double SyncReadGain(set<GainIndex> list_read_gain); 
        template <typename Addr=AddrCommon> double SyncReadLimit(set<LimitIndex> list_read_limit);
        template <typename Addr=AddrCommon> double SyncReadHardwareErrors();
        template <typename Addr=AddrCommon> void StopDynamixels();
        template <typename Addr=AddrCommon> void CheckDynamixels();
};

#endif /* DYNAMIXEL_HANDLER_H */
