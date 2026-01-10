#ifndef CTRL_STEP_FW_MOTION_PLANNER_H
#define CTRL_STEP_FW_MOTION_PLANNER_H

#include <cstdint>


/**
 * @brief MotionPlanner 类 — 通用运动规划控制器
 *
 * 该类封装了多种运动控制追踪器，包括：
 *  - 电流跟踪 (CurrentTracker)
 *  - 速度跟踪 (VelocityTracker)
 *  - 位置跟踪 (PositionTracker)
 *  - 位置插补 (PositionInterpolator)
 *  - 轨迹跟踪 (TrajectoryTracker)
 *
 * 每个跟踪器内部都会根据目标值与实际值进行积分、加速度、以及软目标计算，
 * 以实现平滑的运动控制过渡。
 */

class MotionPlanner
{
public:
    MotionPlanner() = default;

    /** 控制频率 (Hz)，用于计算控制周期 */
    const int32_t CONTROL_FREQUENCY = 20000;                    // Hz
    /** 控制周期 (μs)，用于定时任务或时间步计算 */
    const int32_t  CONTROL_PERIOD = 1000000 / CONTROL_FREQUENCY; // uS

    struct Config_t
    {
        int32_t encoderHomeOffset;///< 编码器零点偏移
        int32_t caliCurrent; ///< 校准电流
        int32_t ratedCurrent;///< 额定电流
        int32_t ratedVelocity; ///< 额定转速
        int32_t ratedVelocityAcc; ///< 额定速度加速度
        int32_t ratedCurrentAcc;///< 额定电流加速度
    };

    // ================================
    //  电流跟踪器（Current Tracker）
    // ================================
    class CurrentTracker
    {
    public:
        explicit CurrentTracker(MotionPlanner* _context) :
            context(_context)
        {
        }


        int32_t goCurrent = 0; ///< 电流目标值（soft goal）

        /** 初始化内部状态 */
        void Init();
        /** 设置电流加速度（变化速率） */
        void SetCurrentAcc(int32_t _currentAcc);
        /** 启动新任务，根据实际电流值更新内部状态 */
        void NewTask(int32_t _realCurrent);
        /** 计算电流的软目标值（平滑过渡） */
        void CalcSoftGoal(int32_t _goalCurrent);


    private:
        MotionPlanner* context;///< 上下文指针（访问全局配置）
        int32_t currentAcc = 0; ///< 电流加速度
        int32_t currentIntegral = 0;//< 电流积分值
        int32_t trackCurrent = 0;///< 当前跟踪电流

        /** 内部积分运算函数 */
        void CalcCurrentIntegral(int32_t _current);
    };
    CurrentTracker currentTracker = CurrentTracker(this);///< 电流跟踪器实例

    class VelocityTracker
    {
    public:
        explicit VelocityTracker(MotionPlanner* _context) :
            context(_context)
        {
        }


        int32_t goVelocity = 0;


        void Init();
        void SetVelocityAcc(int32_t _velocityAcc);
        void NewTask(int32_t _realVelocity);
        void CalcSoftGoal(int32_t _goalVelocity);


    private:
        MotionPlanner* context;
        int32_t velocityAcc = 0;
        int32_t velocityIntegral = 0;
        int32_t trackVelocity = 0;


        void CalcVelocityIntegral(int32_t _velocity);
    };
    VelocityTracker velocityTracker = VelocityTracker(this);

    class PositionTracker
    {
    public:
        explicit PositionTracker(MotionPlanner* _context) :
            context(_context)
        {
        }


        int32_t go_location = 0;
        int32_t go_velocity = 0;


        void Init();
        void SetVelocityAcc(int32_t value);
        void NewTask(int32_t real_location, int32_t real_speed);
        void CalcSoftGoal(int32_t _goalPosition);


    private:
        MotionPlanner* context;
        int32_t velocityUpAcc = 0;
        int32_t velocityDownAcc = 0;
        float quickVelocityDownAcc = 0;
        int32_t speedLockingBrake = 0;
        int32_t velocityIntegral = 0;
        int32_t trackVelocity = 0;
        int32_t positionIntegral = 0;
        int32_t trackPosition = 0;


        void CalcVelocityIntegral(int32_t value);
        void CalcPositionIntegral(int32_t value);
    };
    PositionTracker positionTracker = PositionTracker(this);

    class PositionInterpolator
    {
    public:
        explicit PositionInterpolator(MotionPlanner* _context) :
            context(_context)
        {
        }


        int32_t goPosition = 0;
        int32_t goVelocity = 0;


        void Init();
        void NewTask(int32_t _realPosition, int32_t _realVelocity);
        void CalcSoftGoal(int32_t _goalPosition);


    private:
        MotionPlanner* context;
        int32_t recordPosition = 0;
        int32_t recordPositionLast = 0;
        int32_t estPosition = 0;
        int32_t estPositionIntegral = 0;
        int32_t estVelocity = 0;
    };
    PositionInterpolator positionInterpolator = PositionInterpolator(this);

    class TrajectoryTracker
    {
    public:
        explicit TrajectoryTracker(MotionPlanner* _context) :
            context(_context)
        {
        }


        int32_t goPosition = 0;
        int32_t goVelocity = 0;


        void Init(int32_t _updateTimeout);
        void SetSlowDownVelocityAcc(int32_t value);
        void NewTask(int32_t real_location, int32_t real_speed);
        void CalcSoftGoal(int32_t _goalPosition, int32_t _goalVelocity);


    private:
        MotionPlanner* context;
        int32_t velocityDownAcc = 0;
        int32_t dynamicVelocityAcc = 0;
        int32_t updateTime = 0;
        int32_t updateTimeout = 200; // (ms) motion set-points cmd max interval
        bool overtimeFlag = false;
        int32_t recordVelocity = 0;
        int32_t recordPosition = 0;
        int32_t dynamicVelocityAccRemainder = 0;
        int32_t velocityNow = 0;
        int32_t velovityNowRemainder = 0;
        int32_t positionNow = 0;


        void CalcVelocityIntegral(int32_t value);
        void CalcPositionIntegral(int32_t value);
    };
    TrajectoryTracker trajectoryTracker = TrajectoryTracker(this);


    void AttachConfig(Config_t* _config);

private:
    Config_t* config = nullptr;
};


#endif
