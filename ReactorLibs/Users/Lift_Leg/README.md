# LiftLeg 使用说明

>[!NOTE]
当前的 400、200 数值没有经过严格测量，请根据实际需求调整

## 模块说明

`Lift_Leg` 用于控制前左、前右、后腿三个电机：

- 前腿：`motor_front_left`、`motor_front_right`
- 后腿：`motor_back`（二合一）

## Enable 状态说明

系统里有两层 Enable 状态，需要区分：

- 驱动层 Enable：在 `Start()` 中已调用  
  `motor_front_left.driver.Enable()`、`motor_front_right.driver.Enable()`、`motor_back.driver.Enable()`。
- 功能层 Enable：`_enabled` 默认是 `false`，只有调用 `Lift_Leg::Enable()` 后，`Update()` 才会下发位置控制。

也就是说，默认上电后电机驱动已使能，但腿部动作逻辑默认不工作。  
需要动作时调用 `Enable()`，停止控制时调用 `Disable()`。

## 调车架场景（重点）

如果设备放在调车架上电，腿的机械初始位置不是中点，需要先调用：

`Lift_Leg::From_Bottom()`

该函数会把前后腿偏置设置为“从底部启动”的参考值，避免按中点假设直接控制导致位置不一致。

建议调用顺序：

1. 系统初始化完成（`Start()` 已执行）
2. 调用 `From_Bottom()`
3. 调用 `Enable()`
4. 再调用各类动作接口（如 `Front_LiftAt400()`、`Back_PutDownAt200()` 等）
