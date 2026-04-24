#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "Lift_Leg.hpp"
#include "Chassis.hpp"

StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();

StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore *core);

void Action_of_MoveTo2(StateCore *core)
{
    StateBlock &st = core->GetCurState();
    st.Complete = false;

    static bool inited = false;
    if (!inited)
    {
        inited = true;
        chas.MoveAt(Vec2(2.0f, 0.0f));
    }

    Seq::WaitUntil(chas.move_ok);

    st.Complete = true;
    inited = false;
    chas.move_ok = false;
}

void Action_of_Rotate180(StateCore *core)
{
    StateBlock &st = core->GetCurState();
    st.Complete = false;

    static bool inited = false;
    if (!inited)
    {
        inited = true;
        chas.RotateAt(PI);
    }

    Seq::WaitUntil(chas.rotate_ok);

    st.Complete = true;
    inited = false;
    chas.rotate_ok = false;
}

void Action_of_MoveTo0(StateCore *core)
{
    StateBlock &st = core->GetCurState();
    st.Complete = false;

    static bool inited = false;
    if (!inited)
    {
        inited = true;
        chas.MoveAt(Vec2(0.0f, 0.0f));
    }

    Seq::WaitUntil(chas.move_ok);

    st.Complete = true;
    inited = false;
    chas.move_ok = false;
}

void Action_of_rotate90(StateCore *core)
{
    StateBlock &st = core->GetCurState();
    st.Complete = false;

    static bool inited = false;
    if (!inited)
    {
        inited = true;
        chas.RotateAt(PI / 2.0f);
    }

    Seq::WaitUntil(chas.rotate_ok);

    st.Complete = true;
    inited = false;
    chas.rotate_ok = false;
}

void OrganizeStates()
{
    // StateBlock &state_move2 = example_graph.AddState("MoveTo2");
    // StateBlock &state_rotate = example_graph.AddState("Rotate180");
    // StateBlock &state_move0 = example_graph.AddState("MoveTo0");
    StateBlock &state_rotate90 = example_graph.AddState("Rotate90");  

    // state_move2.StateAction = Action_of_MoveTo2;
    // state_rotate.StateAction = Action_of_Rotate180;
    // state_move0.StateAction = Action_of_MoveTo0;
    state_rotate90.StateAction = Action_of_rotate90;

//  state_move2.LinkTo(&(state_move2.Complete), state_move0);
//  state_move0.LinkTo(&(state_move0.Complete), state_move2);
    // state_move2.LinkTo(&(state_move2.Complete), state_rotate);
    // state_rotate.LinkTo(&(state_rotate.Complete), state_move0);
    // state_move0.LinkTo(&(state_move0.Complete), state_move2);
}
/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    System.RegistApp(lift_leg);
    System.RegistApp(chas);
    System.SetPositionSource(chas.chas_odom.pos);

    // 使能应用
    lift_leg.Enable();
    chas.Enable();
    // monit.Track(chas.motors[0].motor_adrc.eso.z3);

    // OrganizeStates();

    // 配置状态图为简并模式
    // example_graph.Degenerate(Action_of_Dege);

    // 向状态机核心注册
    core.RegistGraph(example_graph);
    core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore *core)
{
}
