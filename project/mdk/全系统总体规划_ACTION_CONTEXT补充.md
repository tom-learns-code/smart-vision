# 《全系统分层视觉与中间层协同总体规划》补充

> 主题：MCU动作上下文反向辅助OpenART视觉  
> 状态：后续完整管线阶段实现，当前纯视觉测试不依赖

## MCU动作上下文辅助视觉

在完整比赛管线中，MCU已经拥有算法层生成的Action序列，并且知道当前正在执行的对象、推动方向和预期目标。MCU可将当前动作上下文发送给OpenART，作为视觉识别的先验信息。

建议新增协议包：

```text
0x05 ACTION_CONTEXT
```

首版只发送当前动作，不发送完整规划路径：

```text
action_id
action_type: MOVE | PUSH_BOX | PUSH_BOMB | WAIT
action_phase: APPROACH | PUSHING | FINISHED | ABORTED
object_id
object_type
push_axis
push_direction
start_cell
target_cell
```

OpenART可据此：

- 提前激活目标对象及其运动走廊ROI。
- 在快速推动未捕获中间帧时，将对象消失与预期目标关联。
- 限制遮挡后的对象位置推算轴和方向。
- 提前建立箱子目标复检或炸弹墙体复检区域。
- 根据当前Action阶段调整车辆、对象和目标区域的识别预算。
- 在车辆视觉短暂丢失时，将短期搜索限制在合理路径走廊。

必须遵循：

```text
MCU动作上下文是识别先验，不是世界状态真值。
视觉必须验证实际发生的结果，不能按规划直接修改地图或删除对象。
```

若视觉结果与动作预期冲突，例如对象未移动、目标未变化、路径受阻或车辆偏离，则OpenART应保持原世界状态并向MCU报告 `LOST / BLOCKED / CONFLICT`，由中间层决定停车、重试或重新规划。

推荐双向信息流：

```text
MCU -> OpenART:
    ACTION_CONTEXT

OpenART -> MCU:
    对象连续位置
    VALID / OCCLUDED / COMPLETED / LOST
    BOX_GOAL_MATCHED / BOX_GOAL_MISMATCHED
    BOMB_EXPLOSION_PENDING / COMMITTED
    观测置信度及冲突状态
```

实施顺序：

1. 先完成不依赖MCU先验的视觉状态机。
2. 打通现有FULL_MAP、POS_UPDATE和自动执行链路。
3. 增加短定长 `ACTION_CONTEXT` 包。
4. OpenART仅用其调整ROI和候选关联，不直接修改识别结果。
5. 完成快速推箱、遮挡、阻挡和规划冲突回归测试后，再将其用于正式比赛。
