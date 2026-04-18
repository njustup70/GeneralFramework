#include "StateCore.hpp"
#include "bsp_dwt.hpp"
#include "stdio.h"
#include "Monitor.hpp"
#include "cmsis_os.h"

/**
 * @brief 为状态块添加状态链接
 */
bool StateBlock::LinkTo(bool *condition, StateBlock& nextState)
{
    if (linkNums < 16)
    {
        links[linkNums].condition = condition;
        links[linkNums].nextState = &nextState;
        linkNums++;
        return true;
    }
    return false;
}


/**
 * @brief 进行状态转移，返回下一个状态ID
 * @return 如果没有状态转移则返回当前状态ID，否则返回下一个状态ID
 */
uint8_t StateBlock::Transition()
{
    for (uint8_t i = 0; i < linkNums; i++)
    {
        if (*(links[i].condition))
        {
            return links[i].nextState->id;
        }
    }
    return id;
}

/**
 * @brief 向状态图中添加状态块
 */
StateBlock& StateGraph::AddState(const char *name)
{
    if (stateNums < 24)
    {
        // 获取目标状态
        StateBlock& targ = states[stateNums];
        
        // 初始化
        targ = StateBlock(name);
        targ.id = stateNums;
        targ.Complete = false;

        // 增加状态数量
        stateNums++;
        return targ;
    }

    // 返回第一个状态，表示添加失败
    return states[0];
}


/**
 * @brief 状态机的简并初始化，一般用于调试
 * @details 只有两个状态：working和end
 */
bool StateGraph::Degenerate(void (*DegenAction)(StateCore *core))
{
    // 固定两个状态

    // 第一个状态：工作状态
    StateBlock& state_work = AddState("working");
    state_work.StateAction = DegenAction;
    state_work.LinkTo(&(states[0].Complete), states[1]);

    // 第二个状态：结束状态
    StateBlock& state_end = AddState("end");
    state_end.StateAction = nullptr; // 什么都不做

    return true;
}


/**
 * @brief 运行状态机
 */
void StateCore::Run()
{
    // 避免空运行
    if (graph_nums < 0 || !_enabled) return;
    
    // 计算时间间隔
    _dt = DWT_GetDeltaTime(&_dwt_tick);

    // 执行 `当前状态图` 的 `对应状态`的 状态函数
    StateGraph& graph = *graphs[at_graph_id];
    StateBlock& state = graph.current_state;

    // 执行当前状态图的全局状态函数
    if (graph.GlobalAction != nullptr) graph.GlobalAction(this);
    
    /**
     * @note 这个写法代表着，一般只有状态函数完全执行完了才会进行状态转移
     * 所以后面应该会加入 在中间打断动作 的机制（确保动作打断是经过作者设计的）
     * @warning 只有非空状态函数才会被执行 
     */
    if (state.StateAction != nullptr) state.StateAction(this);

    // 进行状态转移
    graph.executor_at_id = state.Transition();
    graph.current_state = graph.states[graph.executor_at_id];
}

void StateCore::Enable(uint8_t first_graph)
{
    if (first_graph < graph_nums)
    {
        at_graph_id = first_graph;
        _enabled = true;
    }
}

StateBlock& StateCore::GetCurState()
{
    StateGraph& graph = *graphs[at_graph_id];
    return graph.current_state; 
}


/**
 * @brief 绘制状态机图
 * @details 通过遍历整个状态机，将状态机的状态和状态转换关系以图的形式发送到指定串口上 'Mermaid' 
 * @warning 该函数会阻塞程序运行！！因此禁止在线程中调用，仅做Debug用途
 */
void StateCore::CoreGraph(const StateGraph& graph)
{
    uint8_t buf[60];
    if (Monitor::GetInstance().host_uart.IsValid()) Monitor::GetInstance().host_uart.Transmit((uint8_t *)"StateGraph\n", 11);

    HAL_Delay(10);
    for (uint8_t i = 0; i < graph.stateNums; i++)
    {
        // 发送状态转换关系（mermaid格式）
        for (uint8_t j = 0; j < graph.states[i].linkNums; j++)
        {
            int len = snprintf((char *)buf, 48, "%s --> %s\n", graph.states[i].name, graph.states[i].links[j].nextState->name);
            if (Monitor::GetInstance().host_uart.IsValid()) Monitor::GetInstance().host_uart.Transmit(buf, len);
            HAL_Delay(10);
        }
    }
}


/**
 * @brief 注册状态图
 */
void StateCore::RegistGraph(StateGraph& graph)
{
    if (graph_nums < 4)
    {
        graphs[graph_nums] = &graph;
        graph_nums++;
    }
    else
    {
        Monitor::GetInstance().LogWarning("StateCore: Too much state graph!");
    }
}


namespace Seq
{
    /** 
     * @brief 等待指定时间
     * @param sec 等待时间，单位秒
     * @details 利用osDelay实现
     */
    void Wait(float sec)
    {
        osDelay((uint32_t)(sec * 1000));
    }
    
    /**
     * @brief 等待直到条件满足或超时
     * @param condition 指向布尔条件的引用
     * @param timeout_sec 超时时间，单位秒，默认300秒
     * @details 利用阻塞+让步实现
     */
    void WaitUntil(bool& condition, float timeout_sec)
    {
        uint32_t start_tick = xTaskGetTickCount();
        uint32_t timeout_ticks = (uint32_t)(timeout_sec * 1000);

        // 存储当前任务优先级
        osPriority original_priority = osThreadGetPriority(osThreadGetId());
        // 降低任务优先级，避免死循环占用过多CPU时间
        osThreadSetPriority(osThreadGetId(), osPriorityIdle);

        while (!condition)
        {
            // 检查超时
            uint32_t current_tick = xTaskGetTickCount();
            if ((current_tick - start_tick) >= timeout_ticks)
            {
                break; // 超时退出
            }

            // 让出CPU时间片，避免死循环占用过多资源
            osThreadYield();
        }

        // 恢复任务优先级
        osThreadSetPriority(osThreadGetId(), original_priority);
    }

    namespace _Private
    {
        // 私有代理函数的实现
        void WaitUntil_Impl(CheckFunctionPtr check_func_ptr, void* context, float timeout_sec)
        {
            uint32_t start_tick = xTaskGetTickCount();
            uint32_t timeout_ticks = (uint32_t)(timeout_sec * 1000);

            // 存储当前任务优先级
            osThreadId current_thread_id = osThreadGetId();
            osPriority original_priority = osThreadGetPriority(current_thread_id);
            // 降低任务优先级，避免死循环占用过多CPU时间
            osThreadSetPriority(osThreadGetId(), osPriorityIdle);
            
            // 使用传入的函数指针和上下文来检查条件
            while (!check_func_ptr(context)) 
            {
                // 检查超时
                uint32_t current_tick = xTaskGetTickCount();
                if ((current_tick - start_tick) >= timeout_ticks)
                {
                    break; 
                }

                osThreadYield();
            }

            // 恢复任务优先级
            osThreadSetPriority(current_thread_id, original_priority);
        }
    }
}
