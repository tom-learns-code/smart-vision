# 队员B独立调试进度记忆

> 用途：供队员B今晚接手实车调试。本文独立记录队员B的测试习惯、实测结果、建议和临时支线，不覆盖队员A（原用户）的主记忆与既有约定。
>
> 最后维护：2026-07-21 12:05
>
> 当前操作者：队员A。队员B夜间交接已于2026-07-21结束；本文件保留队员B全部实测、建议与候选版本历史，不再作为当前任务入口。当前工作优先读取`design/7100941_完整三图比赛管理器与20小时调试计划.md`和主记忆末尾快照。

## 0. 阅读顺序与上下文边界

新会话优先只读本文。需要追查当前冻结保护时读`design/7100923_上位机位置冻结整格重走保护.md`；需要追查上一版提速与ALIGN恢复时读`design/7100922_实测提速与ALIGN堵转恢复.md`。只有遇到历史映射、协议或算法争议时才读取大文件`design/当前视觉到控制通路接力记忆.md`的相关章节，不要默认整份加载。

上下文维护规则：

1. 本文件只保留当前状态、已确认结论、当前主线、开放支线、下一步和必要的测试摘要。
2. 长串口日志不整段复制，只记录附件路径、固件ID、地图、操作序列、关键指标和结论。
3. 明确区分`已确认`、`推测`、`待验证`，不得把一次现象直接写成工程事实。
4. 新结论追加日期和测试条件；被推翻的结论标记为`已否定`，不静默覆盖。
5. 队员B的习惯和建议只写本文；队员A的习惯和主线保留在原记忆中。
6. 每轮结束主动删除重复描述，将当前快照更新到本文顶部附近，历史细节只保留高价值摘要。

## 1. 不可修改的主线任务

主线由队员A发布并锁定，队员B沿该主线执行，不重新定义目标：

**在不降低当前稳定完赛能力的前提下提速。**

固定推进顺序：

1. 缩短每格之间的停车时间、视觉排空时间和稳定确认时间。
2. 缩短必要位置校准的耗时，同时保持箱子、炸弹和目标点八邻域的强矫正精度。
3. 用实车遥测确认每一步真正的时间瓶颈，不凭主观速度感连续改参数。
4. 普通开放区域稳定后，尝试安全的一次多格运动；物体八邻域和PUSH仍逐格。
5. 固定滞后、卡尔曼/OOSM等延迟视觉融合先做影子估计和离线验证，通过后才可影响正式控制。

不得因为队员B的临时建议改变主线。合理建议可以登记为候选增强或支线，等待队员A复核。

## 2. 主线与支线管理规则

每个新问题必须先分类：

- `MAIN`：直接影响提速主线的测量、修改和验收。
- `BRANCH-BLOCKING`：不解决就无法继续主线，例如通信断联、固件档位错误、主体运动失控。
- `BRANCH-NONBLOCKING`：不妨碍当前提速，例如偶发打印格式、低影响界面问题。

支线必须记录：触发证据、当前假设、最小验证、完成条件、回归主线条件。阻塞支线解决后立即回归主线；非阻塞支线默认延后，不得连续占用多轮实车时间。

当前任务表：

| ID | 类型 | 状态 | 内容 | 完成/回归条件 |
|---|---|---|---|---|
| MAIN-01 | MAIN | 7100938待完整A验收 | 在稳定完赛能力不下降的前提下继续提速；空旷同向FREE点到点，物体八邻域逐格并留距，PUSH逐格且推后只等延迟视觉追帧 | 完整`A`中空旷FREE可`segment>1`，物体邻域FREE有正确`clear`，PUSH为`clear=(0,0)`；PUSH结束后不再出现`ALIGN_ENC_IMU`追加运动，目标格稳定且每轴误差不超过80mm后继续，错误格仍重规划，最终完赛 |
| BRANCH-01 | BRANCH-BLOCKING | 已完成 | 旧μVision窗口曾缓存`CONTROL_DEBUG_ENABLE=1U`并覆盖源码和标准AXF | 2026-07-20 19:48确认窗口已退出、源码为0U、标准与两份模式0固件哈希一致；已回归MAIN-01 |
| BRANCH-02 | BRANCH-BLOCKING | 已完成 | 首轮7100922地图仅识别`box=1 goal=3 bomb=0`，规划返回`WORLD_REJECTED/BAD_COUNTS`，导致E为`NO_PLAN`、N为`NOT_ARMED` | 2026-07-20 21:32重新取得`wall=82 box=3 goal=3 bomb=0`，规划`valid=1 current=1`且`PLAN_CHECK ok=1`；开局对中和第一格均成功，已回归MAIN-01 |
| BRANCH-03 | BRANCH-BLOCKING | 已封存 | 上位机坐标冻结在出发格但位置帧持续更新，旧逻辑无限重跑完整200mm步骤，现实车辆持续前进 | 7100923保护版和回退点完整保留；后续正式版不得恢复“冻结时重复整格运动” |
| BRANCH-04 | BRANCH-BLOCKING | 7100928已修正，待实车验收 | 外边界上位机把真实边线位置夹在向内约40mm；7100927在`(10,10)->(11,10)`把朝外墙的40mm残差继续作为二维ALIGN，最终`STALLED` | 7100928只钳掉非PUSH外边界的朝墙分量，沿边误差仍强矫正；完整A越过该点且出现`wall_snap accepted=1`后关闭支线 |
| BRANCH-05 | BRANCH-BLOCKING | 7100924实测通过 | 目标点推箱曾因`fast_finish=1`在欠程140mm时提前提交，随后`POST_PUSH_REANCHOR`停车 | 取消fast-finish且允许目标点推箱最多180mm视觉补走；7100924日志已以`delta=(0,0)`完成目标点推箱并继续 |
| BRANCH-06 | BRANCH-NONBLOCKING | 7100926修复保留，仍有轻微瑕疵待回访 | 7100925按`E`后开局回格心未继承边界夹持规则，视觉`(1.2,8.0)`被反复向`(1.0,8.0)`校准，车向边线外移动；终态暂停仍每250ms刷遥测 | 7100926已加入开局边界夹持并关闭暂停态刷屏；队员B反馈仍有一些瑕疵，但明确要求先回归提速主线。若7100927的`E`再次造成明显位移，立即恢复为阻塞支线 |
| BRANCH-07 | BRANCH-BLOCKING | 7100929已修正，待实车验收 | 7100928已越过旧边线故障格，但延迟视觉从出发格追到目标格后，可疑位置分支把强矫正次数直接置满；目标格内仅余40mm也立即`GRID_MISMATCH` | 7100929保留剩余强矫正次数；同位置复测应出现沿边40mm ALIGN并继续，完整A不再在`(11,10)->(12,10)`暂停 |
| BRANCH-08 | BRANCH-BLOCKING | 7100930实测未通过，已由BRANCH-09接管 | 7100929暴露“正对外边界驶入边界格”未被夹持、物体附近二维ALIGN扫箱、开局IMU保持物理偏角三项问题 | 7100930的边界夹持、分轴ALIGN和开局视觉航向保留；实测同时暴露航向环符号正反馈及FREE合并过严，统一由7100931修复验收 |
| BRANCH-09 | BRANCH-BLOCKING | 7100931实测失败，符号结论已否定 | 7100930日志中IMU从约3°持续升到430°，曾误判为公共轮速符号相反；FREE路径确因物体/目标八邻域被拆成`segment=1` | 7100931的E-only日志直接证明公共正轮速使IMU航向增加，符号翻转造成开局正反馈；航向符号已在7100932恢复，FREE点到点部分保留 |
| BRANCH-10 | BRANCH-BLOCKING | 7100932实测通过关键安全项 | 7100931按E后位置对中时，IMU从0°升至405°；同时开局边界夹持只改了判定残差，实际短距移动仍重新瞄准边界格心 | 7100932实测E后电机保持锁止、IMU不再发散、左边界不向外移动；随后被BRANCH-11的小角度接口白名单拦截 |
| BRANCH-11 | BRANCH-BLOCKING | 已完成并由BRANCH-12接管 | 7100932在`START_HEADING_WAIT`取得6帧稳定角度并要求约`+4°`微调时，底层只接受45/90/135/180/360°，命令被拒后触发`OBS_ROTATION_FAILED` | 7100933实测不再FAULT并进入`STEP_WAIT`；连续角度接口与非致命回退有效。由于实车小角度纠偏受静摩擦限制且全局视觉角度抖动，后续策略由BRANCH-12接管 |
| BRANCH-12 | BRANCH-BLOCKING | 7100934待完整A验收 | 7100933实测六帧中值偏差仅`2.4°`、未触发物理微转；开局短距对中使IMU由0°偏到约-3.4°，长1.6m FREE按错误的90°坐标基准运行后出现约80/100mm终点误差并`GRID_MISMATCH`。车体硬件无法可靠执行几度微转，全局视觉角度也会在约66.8~101.3°间跳变 | 7100934不再物理微转；用六帧中值建立软件地图/车身参考，IMU只锁当前车身；起点外边界朝墙容差扩大到100mm。完整A中`RUN_HEADING correction`可非零但不出现`START_HEADING_ROTATE`，`FOLLOW used`应为稳健视觉参考而非固定90.0，长FREE横向漂移应明显下降且不再因同类误差暂停 |
| BRANCH-13 | BRANCH-BLOCKING | 7100935几何规则通过，后续故障由BRANCH-14接管 | 车壳偏斜时追格心会非计划擦碰物体；物体邻域需要几何留距且FAST_SAFE不能跨越八邻域 | 7100935日志确认斜邻物体FREE出现正确`clear=(35,0)`、真正PUSH保持`clear=(0,0)`、动作跟随器`DONE/NONE`；几何方向通过，回归条件改由推后确认继续性决定 |
| BRANCH-14 | BRANCH-BLOCKING | 已完成并由BRANCH-15接管 | 7100935推箱已经到`delta=(20,-20)mm`并`DONE/NONE`，随后稳定视觉约`delta=(40,-20)mm`；推后确认仅允许每轴20mm，2秒后误判`REPLAN_REQUIRED` | 7100936实测第一个同位置PUSH后`post_push result=STABLE at=(1.9,8.9)`并继续action11，证明40mm稳定目标格修复有效；已回归后续PUSH链路 |
| BRANCH-15 | BRANCH-BLOCKING | 已完成，由BRANCH-16接管 | 7100936第二个PUSH从稳定视觉`x=1.3`向目标`x=2.0`启动，主体距离按140mm正确计算；运动结束后延迟源格帧`x=1.4`却触发120mm `ALIGN_ENC_IMU`，延迟帧追上时车已到`x=2.8`，导致多推箱并`GRID_MISMATCH` | 7100937完整日志中全部PUSH主体运动后均未再进入`ALIGN_ENC_IMU`；炸弹PUSH已从`POST_PUSH_REANCHOR`继续到后续动作，确认延迟视觉隔离有效 |
| BRANCH-16 | BRANCH-BLOCKING | 7100938待完整A验收 | 7100937最后一次PUSH逻辑车位`(3,9)->(3,8)`，跟随器锁车等待视觉追帧并在`car=(2.7,8.1)`、误差60/20mm时正确结束；上层推后复核仍只允许每轴40mm，2秒后误判`PAUSED_REPLAN` | 7100938把推后稳定目标格容差统一为每轴80mm，仍要求目标格、4个新帧和200ms稳定；同一路径应在该PUSH后继续且不能出现主动追加ALIGN，100mm或错误格仍拒绝 |

## 3. 当前工程快照

### MCU

- 当前待下载版本：`7100938 / PUSH_POST_REANCHOR80`
- 正式模式：`CONTROL_DEBUG_ENABLE=0U`
- 当前标准固件：`project/mdk/Objects/rt1064.axf`
- 固定测试副本：`project/mdk/Objects/rt1064_TEAM_B_7100938_PUSH_POST_REANCHOR80_MODE0.axf`
- 当前标准AXF SHA256：`5C7BB3E2734F52FB5FE344BD35A27C30438AECE5ED9150B52633EFEA336632C2`
- 当前状态：完整保留7100934的软件航向参考、7100935的物体八邻域50/35mm几何留距与PUSH零偏置，以及7100937的PUSH延迟视觉隔离。7100938只把mission层推后稳定目标格容差由每轴40mm改为80mm，使其与跟随层100/120mm成功边界一致；仍要求四舍五入到计划目标格、4个新视觉帧和200ms稳定，100mm或错误格继续拒绝。速度120、PID/PWM、三轮/编码器映射、IMU、规划和两台摄像头均未修改。Keil ARM Compiler 6.16编译`0 Error(s), 0 Warning(s)`，助手未下载MCU。

### 两台摄像头

- 全局摄像头：`oa_id=7100425`，本轮未修改
- 第二摄像头：`fp_id=7200102`，本轮未修改，`FAKE_RECOGNITION_ENABLE=True`
- 全局视觉负责地图和车位；第二摄像头负责箱子图像/目标数字请求式识别

### 回退点

- 稳定完赛基线：`project/_codex_backups/checkpoint_7100920_STABLE_FINISH_BASELINE_20260720_064800`
- 7100922提速完成态：`project/_codex_backups/checkpoint_7100922_ADAPTIVE_VIS_ALIGN_RECOVERY_BUILT_20260720_123855`
- 冻结保护完成态：`project/_codex_backups/checkpoint_7100923_FROZEN_POSE_MOVE_GUARD_BUILT_20260720_220120`
- 7100924目标点推箱完成态：`project/_codex_backups/checkpoint_7100924_GOAL_PUSH_VISUAL_FINISH_BUILT_20260721_000203`
- 当前7100925完成态：`project/_codex_backups/checkpoint_7100925_BOUNDARY_TARGET_CLAMP_BUILT_20260721_002955`
- 7100926修改前备份：`project/_codex_backups/checkpoint_7100925_before_start_edge_fix_20260721_005302`
- 当前7100926完成态：`project/_codex_backups/checkpoint_7100926_START_EDGE_CLAMP_QUIET_BUILT_20260721_005630`
- 7100927修改前备份：`project/_codex_backups/checkpoint_7100926_before_full_straight_fast_20260721_011447`
- 当前7100927完成态：`project/_codex_backups/checkpoint_7100927_FULL_STRAIGHT_FAST_DEFAULT_BUILT_20260721_012238`
- 7100928修改前备份：`project/_codex_backups/pre_7100928_STRICT_EDGE_ALIGN_XTRACK_20260721_015001`
- 7100928中间构建（未含最终在线边界门控）：`project/_codex_backups/checkpoint_7100928_STRICT_EDGE_ALIGN_XTRACK_BUILT_20260721_015251`
- 当前7100928最终完成态：`project/_codex_backups/checkpoint_7100928_STRICT_EDGE_ALIGN_XTRACK_FINAL_BUILT_20260721_015843`
- 7100929修改前备份：`project/_codex_backups/pre_7100929_TARGET_CELL_ALIGN_RECOVERY_20260721_021245`
- 当前7100929完成态：`project/_codex_backups/checkpoint_7100929_TARGET_CELL_ALIGN_RECOVERY_BUILT_20260721_021507`
- 7100930修改前备份：`project/_codex_backups/pre_7100930_SAFE_ALIGN_P2P_20260721_023340`
- 当前7100930完成态：`project/_codex_backups/checkpoint_7100930_SAFE_ALIGN_P2P_HEADING_BUILT_20260721_025308`
- 7100933修改前备份：`project/_codex_backups/pre_7100933_START_HEADING_FINE_ROT_20260721_034651`
- 7100933完成态：`project/_codex_backups/checkpoint_7100933_START_HEADING_FINE_ROT_FALLBACK_BUILT_20260721_035100`
- 7100934修改前备份：`project/_codex_backups/pre_7100934_SOFT_HEADING_REF_EDGE100_20260721_040021`
- 当前7100934完成态：`project/_codex_backups/checkpoint_7100934_SOFT_HEADING_REF_EDGE100_P2P_BUILT_20260721_040425`
- 7100936修改前备份：`project/_codex_backups/pre_7100936_POST_PUSH40_STABLE_20260721_045517`
- 当前7100936完成态：`project/_codex_backups/checkpoint_7100936_POST_PUSH40_STABLE_BUILT_20260721_050206`
- 7100937修改前备份：`project/_codex_backups/pre_7100937_PUSH_DELAY_GUARD_20260721_051824`
- 当前7100937完成态：`project/_codex_backups/checkpoint_7100937_PUSH_DELAY_DRAIN_ONLY_BUILT_20260721_052544`
- 7100938修改前备份：`project/_codex_backups/pre_7100938_POST_PUSH_TOLERANCE_20260721_053822`
- 当前7100938完成态：`project/_codex_backups/checkpoint_7100938_PUSH_POST_REANCHOR80_BUILT_20260721_054300`

### 3.1 7100927普通直线整段提速规则

- 任务归类：`MAIN-01`。目的不是提高单轮速度，而是删掉安全路段每格之间重复的停车、视觉排空和重新起步。
- `A`自动运行且档位为`FAST_SAFE`时，从当前FREE格开始向后检查；只有同方向、连续、环境执行策略完全兼容的步骤才合并。地图16列，理论最大安全直线段为13格/2600mm；考虑格心起点最多120mm视觉容差，底层命令许可上限为2720mm，但逻辑终点仍是原来的第13格。
- 以下任一条件都会在边界前断段：方向变化；FREE动作结束；箱子或炸弹八邻域；目标点八邻域；PUSH；墙体侧向关系改变；单边墙/通道/开放区策略改变；通道出入口等重锚节点。
- 合并后的整段仍由编码器完成距离主体运动、IMU锁航向，并保留单边墙/通道原有的横向策略；开放区多格段结束时执行一次强重锚。
- 箱子、炸弹和目标点附近仍为`BOX_NEAR_PRECISE`，所有PUSH仍为`PUSH_PRECISE`；均保持`segment=1`和严格20/30mm到位逻辑，不允许被快速段跨越。
- 7100930起，`N`不受快速档位影响，执行当前同方向且上下文兼容的完整安全直线点到点；`G`永远只执行一格，用于下载后的首格方向、停车和测量校验。`A`在`FAST_SAFE`下自动合并安全直线，在`STANDARD`下仍逐格。
- 未修改：移动速度/PID/PWM、三电机与编码器映射、IMU、CPR4096、里程比例0.915、F50/R30/B47/L20制动提前量、规划器输出、PUSH算法、两台摄像头与通信协议。
- 静态验证：任务层和跟随器均限制最长13格，底层距离允许13格名义距离加120mm起点容差；FREE动作最多50个航点，容量足够；Keil编译0错误0警告。
- 实车验收：重新上电确认7100927；`1 -> 等MAP_END -> 2 -> D -> 等PLAN_CHECK ok=1 -> B -> 3 -> E -> 等STEP_WAIT -> S -> N -> 等STEP_WAIT -> A`。正常长直线应看到`profile=FAST_SAFE`且`segment=2..13`；物体附近和PUSH必须回到`segment=1 strict=1`。
- 立即停止条件：`E`阶段车出现明显位移；长段跨过转弯或箱子/炸弹邻域；PUSH显示`segment>1`；长段末端明显不减速或冲过目标；任何明显失控。发生时按`X`并保留完整日志。

### 3.2 7100927首轮地图计数不合法，尚未进入提速实测

- 实测固件确认正确：`7100927 / FULL_STRAIGHT_FAST_DEFAULT`，全局视觉在线，车位有效，快速档位默认为`FAST_SAFE`。
- 本次FULL_MAP为`wall=89 box=2 goal=3 bomb=3`。第9行是`M09 #----########--#`，相较该地图正常形态缺少一个`$`箱子。
- `2 -> D`返回`PLAN_SUMMARY valid=0 current=0 status=WORLD_REJECTED world=BAD_COUNTS solver=BAD_ARGUMENT`，表示在路径算法启动前就被世界模型计数校验拒绝；与快速直线合并逻辑无关，也没有驱动车辆。
- 下一步只重新采图，不修改代码：`B -> 1 -> 等MAP_END`，直到确认`box=3 goal=3 bomb=3`且缺失箱子恢复，再执行`2 -> D`。不得在`BAD_COUNTS`状态下继续`3 -> E/N/A`。

### 3.3 7100927有效地图实测与7100928边界矫正修复

- 日志：`C:\Users\L\.codex\attachments\6a12b5a9-d9e4-4a5f-b9b5-371d09e14a6f\pasted-text.txt`。地图计数`wall=89 box=3 goal=3 bomb=3`，规划`valid=1`且`PLAN_CHECK ok=1`，共28个Action、37个整格步骤。
- 快速整段规则本身按设计工作：只出现两段2格`OPEN_FAST`；所有箱子邻域与PUSH均保持`segment=1 strict=1`，没有跨越物体八邻域。因此本次边线停车不是多格合并越界。
- 真正故障位于`action=12/28 sub=3/4 step=21/37`，FREE移动`(10,10)->(11,10)`。视觉最终为`car=(11.0,9.8)`、目标`(11.0,10.0)`，只剩朝地图下外墙方向40mm；旧逻辑却启动斜向短距ALIGN，最终`fault=STALLED`。
- 用户“没有感到强矫正”的反馈有日志依据：运行中287组遥测仅约10组`xtrk on=1`；严格物体邻域常见的20mm横偏正好落在旧`<=20mm`死区内，而且前300ms默认不启动纠偏。旧强矫正主要发生在每格结束后，不等于持续在线纠偏。
- 7100928只做三项局部修正：外边界同目标格、非PUSH、朝墙残差不超过60mm时，允许在point已DONE或视觉复查锁车状态下钳掉墙法向分量，沿边误差最大仍只允许120mm并继续矫正；严格物体邻域视觉横偏死区改为10mm，非外边界处超过死区时允许起步即在线纠偏；四条外边界内侧格出现朝外墙且不超过60mm的在线横偏时直接抑制该墙法向纠偏，避免主体运动阶段反复把车推向墙。PUSH、内场墙、跨格位置、远离墙的偏差均不放行。
- 未修改：速度、PID/PWM、三轮映射、IMU、CPR4096、里程比例0.915、F50/R30/B47/L20、规划器、快速段合并边界、两台摄像头及协议。
- 下一次完整流程：`下载rt1064.axf -> 完全断电重上电 -> 确认7100928/STRICT_EDGE_ALIGN_XTRACK -> 1 -> 等FULL_MAP received/MAP_END并核对box=3 goal=3 bomb=3 -> 2 -> D -> 等PLAN_SUMMARY valid=1 current=1和PLAN_CHECK ok=1 -> B -> 3 -> E -> 等STEP_WAIT -> S -> N -> 等STEP_WAIT -> A`。重点观察：非外边界物体邻域20mm横偏应更早出现`xtrk on=1`；到`(11,10)`这类外边界附近，朝墙20～40mm残差的`xtrk on=0`是`EDGE_XTRACK_GUARD60`主动保护的正常现象，停车复查应出现`wall_snap candidate=1 accepted=1`并继续运行，不得再启动朝墙的斜向ALIGN。

### 3.4 7100928实测与7100929目标格残差恢复

- 日志：`C:\Users\L\.codex\attachments\7a87f185-b3d1-4ac5-87d7-d548ed8aba7e\pasted-text.txt`。地图`wall=89 box=3 goal=3 bomb=3`，规划`valid=1`、`PLAN_CHECK ok=1`，共25个Action、37格。
- 7100928已经越过上一轮必卡的`(10,10)->(11,10)`，未再出现朝外墙斜向ALIGN或`STALLED`，因此`EDGE_XTRACK_GUARD60`方向正确。
- 在线横偏并非未运行：本次共有26组`xtrk on=1`，严格区域20mm偏差通常产生约`corr=+/-8`；IMU相对航向大多保持约`+/-2deg`。与7100927约10组激活相比已有明显改善，暂不继续放大增益。
- 新停止点为`action=13/25 sub=4/8 step=25/37`，FREE移动`(11,10)->(12,10)`。视觉延迟从`11.1 -> 11.5 -> 11.8`追入目标格，最终`car=(11.8,9.9)`，目标`(12.0,10.0)`。边界法向20mm应被钳掉，只需沿边补走40mm。
- 根因是可疑位置确认目标格的分支先执行`follower_align_attempts = follower_align_max_attempts()`，随后严格模式看到40mm残差但已无矫正机会，只能进入`POSE_REPLAN / GRID_MISMATCH`。这不是电机堵转、规划错误或多格合并越界。
- 7100929删除该预先耗尽动作，保留已有矫正次数；目标格内且不超过120mm的残差按原逻辑补走，跨格、PUSH异常、超过120mm和三次仍不到位仍保持原重规划边界。
- 下一次完整流程：`下载rt1064.axf -> 完全断电重上电 -> 确认7100929/TARGET_CELL_ALIGN_RECOVERY -> 1 -> 等MAP_END并核对box=3 goal=3 bomb=3 -> 2 -> D -> 等valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等STEP_WAIT -> S -> N -> 等STEP_WAIT -> A`。在`(11,10)->(12,10)`附近，预期`pose_suspect`确认目标格后出现`align=1`、`err=(40,0)`一类沿边补走并继续；不应再直接`GRID_MISMATCH`。

### 3.5 7100929实测：边界外推、斜向强矫正与航向基准

- 日志：`C:\Users\L\.codex\attachments\0b55e844-5d41-466c-ad59-c742819447de\pasted-text.txt`。本轮属于`BRANCH-08 / BRANCH-BLOCKING`，暂时阻塞提速主线实车复测；7100929本轮未再修改。
- 确定故障点为`action=11/50 sub=0/2 step=17/75`的FREE移动`(2,9)->(1,9)`，并非PUSH。视觉最终稳定夹在`car=(1.3,9.0)`，逻辑目标为`(1.0,9.0)`，残差`delta=(-60,0)mm`恰好朝左外边界。
- 7100929按原设计保留了3次强矫正机会，因此反复执行`map_dir=251.6/288.4deg`、`speed=100`的斜向ALIGN；视觉Y在`8.9/9.1`间抖动，使运动方向左右摆动，但X始终被上位机夹在1.3，最终`rebase=4`并进入`POSE_REPLAN / GRID_MISMATCH`。这精确解释了“边线总往外跑”。
- 代码级缺口已确认：现有`follower_apply_target_boundary_clamp()`只在“沿外墙平行移动”时按`wall_axis_mask`钳掉墙法向分量；当前是横向正对左边界驶入`x=1`，边界误差属于运动方向而非横向墙轴，因此未进入夹持。
- 车身偏斜不是单纯PID力度不足。`action_follower_begin_mission_heading()`仅用视觉角度确定最近的0/90/180/270象限，随后把武装瞬间`imu963ra_yaw_angle`直接保存为全程航向目标；它没有把物理车身先转到量化方向。因此车若在按`E`时已偏8至12度，IMU会很好地保持这份偏角。日志中IMU相对误差多数仅约正负2.5度，而视觉角常见`77.9~85.9deg`、参考仍为90度，符合“控制器稳定保持了可能偏着的基准”。
- 物体八邻域当前强矫正直接对`dx/dy`做`atan2`合成，任一轴20mm视觉抖动都会把本应纯横/纯纵的短移变成斜移；车体包络会扫过相邻箱子，存在用户已观察到的碰箱改位风险。`ALIGN_RECHECK`阶段速度为0但航向保持仍可输出数百PWM，也可能造成轻微摆动。
- 拟定修复顺序：P0扩展外边界夹持，使目标坐标位于`x/y=1`或`W/H-2`时，无论从哪个方向进入，只要同目标格、非PUSH、已停车、航向稳定且朝墙残差不超过60mm，就钳掉对应分量；P0将物体八邻域强矫正改为分轴、安全方向优先的短移，次轴不超过20至30mm时归零，禁止任意斜向扫过物体；P1在任务武装阶段使用多帧视觉角确定象限，再由IMU完成一次实际转正并重新捕获航向目标。视觉角只做低频绝对校准，不进入连续航向反馈。
- 当前结论足以停止重复实车试验：7100929在同类左/右/上/下边界上会可预测地重复外推。下一步应先完成局部代码修复和静态检查，再生成新识别数后执行完整菜单验收。

## 4. 7100923正在验证什么

7100921的实车日志已确认：允许自适应停车45次，但实际提前停车0次；最终在严格物体邻域短距ALIGN中堵转并被旧逻辑直接升级为任务硬故障。

7100922只做以下变化：

- 静止判断只看三轮编码器速度与IMU角速度，不再看位置环误差。
- 普通非严格、非切换步骤使用`700ms排空 + 4帧 + 150ms稳定`。
- 物体/目标八邻域和切换步骤仍使用`1000ms排空 + 6帧 + 200ms稳定`及20/30mm严格误差。
- 短距ALIGN的STALL改为锁车、等待新视觉、重算并重试/重规划。
- 主体200mm运动STALL仍为硬故障。

7100923在上述行为之上只增加一条安全边界：编码器已完成一格，而新位置帧仍稳定停留在出发格时，电机保持锁定并额外等待6帧；仍冻结则`REPLAN_REQUIRED`，不再重跑完整一格。`RUN_VIS`新增`from_wait=current/limit`。

没有修改PID、速度、PWM、电机/编码器映射、旋转、CPR4096、里程比例0.915或F50/R30/B47/L20制动提前量。

## 5. 队员B第一轮完整操作

### 5.1 下载与上电确认

1. 下载标准固件`project/mdk/Objects/rt1064.axf`。固定交接固件`rt1064_TEAM_B_7100923_MODE0.axf`仅作为同哈希备用。
2. 完整断电后重新上电。
3. 串口必须看到：

```text
#MCU_BOOT id=7100923 stage=FROZEN_POSE_MOVE_GUARD
```

如果ID不是7100923，停止测试，不要继续按菜单。

### 5.2 请求地图并计算

完整菜单序列：

```text
1
-> 等待 FULL_MAP received 和 MAP_END
-> 2
-> D
-> 等待 PLAN_SUMMARY valid=1 current=1
-> 确认 PLAN_CHECK ok=1
-> B
-> 3
-> E
-> 等待开局格心复核完成并进入 STEP_WAIT
```

各步骤含义：

- `1`：MCU请求全局摄像头发送一帧完整地图。
- `2 -> D`：进入规划菜单并按详细模式计算/打印路径，不会驱动车。
- `B`：返回主菜单。
- `3 -> E`：进入运行菜单并武装当前计划；系统先验证视觉位置并将车重锚到起始格中心。
- `STEP_WAIT`：当前动作已准备好，但电机仍等待`N`或`A`命令。

### 5.3 高效率批量验证

先按一次`N`检查第一格方向、停车和坐标是否合理。第一格正常且没有`FAULT`后，直接按`A`连续跑完，不再机械地逐格重复`N`。整个A过程应随时准备按`X`急停。

一次完整A应同时覆盖：普通开放区、墙边/走廊、物体八邻域、PUSH和短距ALIGN。这比每轮只测一格更有信息量。

重点遥测：

- 普通开放步骤：`adaptive=1/1`，`frames=.../4`，`drain`约700ms。
- 严格步骤：`adaptive=0/0`，`frames=.../6`，`drain`约1000ms。
- ALIGN堵转恢复：`stall_retry=1`后等待重试或重规划，不应立即`MISSION FAULT/STALLED`。
- 主体运动真实堵转：仍应硬停，这是正常保护。

## 6. 队员B每轮反馈模板

每次测试尽量一次提供完整信息：

```text
固件ID：
地图名称/实际地图：
重新上电：是/否
完整菜单序列：
执行N还是A：
车的实际表现：方向、偏移、停顿、是否卡住：
停止时的位置/动作：
是否主动按X/S：
串口日志附件路径：
队员B的判断或建议：
```

不要只说“停了”或“变慢了”；至少说明当时是在普通移动、转向、靠近箱子、PUSH还是视觉矫正。

## 7. 每次修改后的强制汇报

助手每次结束必须主动给出：

1. MCU是否修改、当前识别数、是否已编译、是否已下载。
2. 全局摄像头是否修改、当前`oa_id`、是否已同步设备。
3. 第二摄像头是否修改、当前`fp_id`、伪识别状态、是否已同步设备。
4. 本轮属于主线还是哪一条支线。
5. 修改目的、没有修改的安全边界和预期变化。
6. 从重新上电开始的完整菜单操作。
7. 每个菜单动作的含义、正常输出和立即停止条件。
8. 下一步具体要做什么。
9. 记忆文件和备份是否已经更新。

## 8. 队员B习惯与建议记录

当前尚无已确认的个人调试习惯。后续只记录队员B明确表现出的偏好，不根据一次操作推断。

合理建议记录格式：

```text
日期 / 建议内容 / 适用场景 / 支持证据 / 风险 / 是否采纳 / 是否需要队员A复核
```

## 9. 当前下一步

### 2026-07-20 22:02 BRANCH-03冻结坐标危险重走

- 原始日志：`C:\Users\L\.codex\attachments\dc7c91f2-47c7-4f25-b85e-87f6d9c2e0e7\pasted-text.txt`
- 已确认：故障发生于`S021 (10,10)->(11,10)`，上下文`BOX_NEAR_PRECISE strict=1`。
- 已确认：上位机位置始终为`(10.1,10.0)`，但`pos`持续递增且`age`仅约5至275ms；这是内容冻结，不是通信断联。
- 已确认：旧逻辑每次完成编码器运动后都将稳定的出发格坐标解释为“尚未移动”，`rebase=3->4->5`并重复执行整格，导致现实车辆持续前进。
- 已完成：7100923删除整格重跑；额外等待一组6帧，仍冻结则锁车并进入`PAUSED_REPLAN`。速度、PID、正常确认参数和两台摄像头均未修改。
- 待验证：正常视觉下完整A不受影响；冻结再次出现时，必须看到`from_wait=1/1`且车辆不再重复移动。

下一步从重新上电开始：`1 -> 等待MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> N -> 等待STEP_WAIT -> A`。运行A时随时准备按`X`；若同一目标下上位机位置不变，先观察车辆是否已锁定，看到`from_wait=1/1`后应进入`PAUSED_REPLAN / REPLAN_REQUIRED`。任何重复整格移动、明显偏离、靠近物体仍持续运动或6秒以上无安全锁定，立即按`X`并回传完整日志。

### 2026-07-20 22:26 队员B临时回归7100920稳定完赛基线

- 队员B要求实测队员A已经确认稳定的“箱子、炸弹及目标点八邻域强矫正”版本。
- 已从重点封存检查点`project/_codex_backups/checkpoint_7100920_STABLE_FINISH_BASELINE_20260720_064800`恢复MCU源码、模式0 Keil工程配置和正式AXF。
- 当前MCU源码与封存基线逐文件SHA256一致；正式`project/mdk/Objects/rt1064.axf`与封存AXF一致，SHA256=`0DB2D2AAE7A7B70A3D1447EAE47AF9785C38971900A38A36372DE225ACBCE905`。
- 固定测试副本：`project/mdk/Objects/rt1064_TEAM_B_7100920_STABLE_MODE0.axf`。
- 启动识别必须为`id=7100920 stage=OBJECT_NEAR_START_REANCHOR`；配置串应包含`OBJECT8_STRICT20_30_ATTEMPT3`。
- 恢复前的7100923源码与AXF已完整保存在`project/_codex_backups/checkpoint_7100923_before_restore_7100920_STABLE_20260720_222646`；7100923原完成态检查点仍保留。
- 全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`且伪识别开启。
- 本轮属于`BRANCH-BLOCKING`对照实测，不改变队员A确定的提速主线。实测结束后依据结果决定继续留在稳定基线，或恢复后续版本处理冻结视觉。
- 注意：7100920不包含7100923的上位机坐标冻结整格重走保护。若上位机坐标明显冻结而实车仍继续移动，立即按`X`，不要等待自动处理。

完整实测流程：`重新下载rt1064.axf -> 完全断电重上电 -> 确认7100920 -> 1 -> 核对MAP_END且box=goal -> 2 -> D -> 确认valid=1/current=1与PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> N -> 等待STEP_WAIT -> A`。

### 2026-07-20 22:xx 7100920首轮回归日志结论

- 日志：`C:\Users\L\.codex\attachments\eb29233c-8a7a-4b37-9eed-5b3972042c2d\pasted-text.txt`。
- 固件确认正确：`id=7100920 stage=OBJECT_NEAR_START_REANCHOR`。
- 地图合法，`box=3 goal=3`；规划成功，`PLAN_SUMMARY valid=1`、`world=OK`、`solver=OK`、`PLAN_CHECK ok=1`。
- 开局格心重锚成功，任务进入`STEP_WAIT`；随后实际收到的是`A`，直接进入自动运行，没有先执行一次`N`。
- 第一格`(1,8)->(2,8)`正确进入`BOX_NEAR_PRECISE`，上下文`box=1 bomb=1 object=1 strict=1`，证明队员A的物体八邻域强矫正代码已经生效。
- 主体编码器运动约3.25秒完成，随后正常进入`HOLD_500MS -> VIS_DRAIN -> VIS_STABLE`；视觉最终到达约`(2.2,8.1)`，相对目标残差约`(-40,-20)mm`，需要进一步短距ALIGN。
- 阻塞原因不是规划、控制器故障或内部自动锁车：严格确认要求6个新位置帧，但`RUN_VIS pos`停在`174`，确认进度停在`1/6`。链路总包仍新鲜，只是位置内容/位置帧停止更新，属于上位机位置冻结。
- 最终`X`由PC发送，任务被人工急停；日志没有`FAULT`、`REPLAN`或程序自行停止。
- 结论：7100920稳定强矫正本身工作正常，但它会在位置帧冻结时无限等待，因此无法单独解决“上位机卡住仍要完赛”的主问题。下一版应以7100920为稳定控制基线，只叠加位置帧冻结检测、编码器+IMU逻辑格续跑和视觉恢复重接管，不引入7100921/7100922的提速实验。

### 2026-07-20 23:15 7100920第二轮回归日志结论

- 日志：`C:\Users\L\.codex\attachments\f1be3612-2d04-48c5-beff-3a9a4082288e\pasted-text.txt`。
- 固件、地图和规划均正确：`id=7100920`，两次地图均为`wall=89 box=3 goal=3 bomb=3`，规划`valid=1`、`world=OK`、`solver=OK`、`PLAN_CHECK ok=1`。
- 开局从视觉`(1.2,8.2)`向逻辑格心`(1,8)`对中，最终约为`(1.1,8.0)`；随后实际操作为直接按`A`，首格仍为`(1,8)->(2,8)`。
- 首格正确进入`BOX_NEAR_PRECISE`，`near box=1 bomb=1 object=1 strict=1`；200mm主体编码器运动完成，未出现`STALLED`、`TIMEOUT`、电机失控或航向环故障。
- 严格视觉残差在三次短距ALIGN中约为`(+60,0) -> (-40,-20) -> (+40,+20)mm`，最终视觉约`(2.2,8.0)`，相对目标残差`(-40,0)mm`。这体现出全局坐标20至40mm级量化/抖动，不是车又执行了一整格。
- 每次严格确认要求6个新位置帧，但最后仅到`2/6`，`RUN_VIS pos`停在`180`；总链路`age`仍较小，因此仍是位置内容/位置帧冻结，不是UART断联。
- 三次ALIGN用尽后，程序在约12.34秒主动进入`MISSION state=PAUSED_REPLAN result=REPLAN_REQUIRED`和`FOLLOW fault=GRID_MISMATCH`，电机保持`LOCKED`。本轮不是人工X，也不是程序卡死；它是7100920原有严格门控的预期安全停机。
- 已确认结论：7100920强矫正控制底座本身可靠，但“20/30mm严格阈值 + 6个新位置帧”无法容忍当前上位机位置冻结。继续重复实车测试不会增加信息量。
- 下一版建议：继续以7100920为唯一控制基线；位置帧真实更新时保留最多3次强矫正；若200mm编码器主体运动已完成且位置帧进度在约600至1000ms内不再变化，则停止重复ALIGN，以编码器+IMU和逻辑格提交当前步并进入视觉隔离；视觉恢复且连续3至6帧匹配逻辑格后再恢复强矫正。视觉恢复后若跨格冲突，只在安全非物体格等待/重规划，不在箱子或炸弹旁追逐异常坐标。

### 2026-07-20 23:xx 7100920第三轮长程日志结论

- 日志：`C:\Users\L\.codex\attachments\f12cfc4d-ca8d-440e-b08a-d95a771eda32\pasted-text.txt`；日志从已运行中的`action=3 step=6`开始，固件仍为7100920。
- 本轮不是一启动就失败。位置帧正常更新期间，程序连续完成多次`PUSH_PRECISE`、普通FREE和POST_PUSH_REANCHOR；至少运行到`action=10 step=16`，并在`action=9`把箱子从`(6,9)`推到目标`(6,10)`，日志显示`goal=1 consumed=1`。这进一步证明路径、编码器主体运动、IMU锁航向和物体邻域强矫正均可正常工作。
- 最终阻塞点为`action=10 wp=2/5 step=16`，逻辑移动`(4,9)->(3,9)`。车已到视觉约`(3.1,9.0)`，目标为`(3.0,9.0)`，残差仅`(-20,0)mm`，主体运动已经结束且电机为`LOCKED`。
- 该步仍处于`BOX_NEAR_PRECISE strict=1`，要求6个新位置帧；确认只到`1/6`，随后`RUN_VIS pos`长期固定为`1003`，但总链路`age`仍在20至290ms间变化。再次确认是位置帧/内容冻结，而不是UART断联。
- 与第二轮不同：本次残差已经处于20/30mm严格容差内，所以没有必要启动新的ALIGN，也就没有ALIGN尝试次数可耗尽；旧逻辑因而一直停在`WAIT_VIS_STABLE`，日志末尾已等待约19秒，既不继续也不进入`PAUSED_REPLAN`。
- 已确认设计缺口：冻结看门狗必须放在通用`WAIT_VIS_STABLE`确认层，不能只依赖ALIGN尝试次数。对“主体运动完成、车已锁定、残差在容差内、位置帧计数不再增加”的情况，应在约600至1000ms后提交逻辑格并进入视觉隔离，而不是无限等待。
- PID日志中的非零`pwm`是锁车前/控制器内部缓存的诊断值；`motor=LOCKED`、编码器为0且车体没有继续移动，不能据此判断电机仍在输出。
### 2026-07-20 23:xx 7100920第四轮长程日志：卡点与“强矫正不正”分析

- 日志：`C:\Users\L\.codex\attachments\2f1bb435-7a94-4fa7-92a2-7f733291b252\pasted-text.txt`；MCU仍为`7100920 / OBJECT_NEAR_START_REANCHOR`。
- 本轮跑到`action=14 step=23`的PUSH，逻辑目标为`(3,8)`/`(600,1600)mm`；车的最后视觉位置为`(2.9,8.0)`，残差仅`(20,0)mm`。
- 最终停车原因仍是位置帧停止推进：`RUN_VIS pos=1034`长期不变，确认进度固定在`2/6`，随后任务进入`PAUSED_REPLAN / REPLAN_REQUIRED`，电机是`LOCKED`。不是PID正反馈、轮子堵转或路径算法失败。
- 424条遥测中，全局视觉原始角度`vtheta=63.4~118.3deg`，平均`86.2deg`，说明车标记的视觉角度确实存在明显抖动/偏差。但控制中`used=90.0deg`始终不变，并由IMU锁定航向，所以原始`vtheta`抖动不会直接成为强矫正的运动方向。
- 强矫正当X、Y残差同时非零时，本来就会沿合成向量斜向移动；例如`delta=(40,-20)mm`对应约`map_dir=63.4deg`。因此肉眼看到“没有横平竖直”不能单独证明摄像头歪了。
- 日志中纯水平段的Y串扰和纯垂直段的X串扰大多约`0.1格/20mm`，暂无证据支持“全局摄像头坐标轴明显旋转”是主因。摄像头被碰后的轻微倾斜仍可能存在，需要用静态+手动沿格线平移的对照试验区分。
- 若摄像头坐标轴对齐而通电运动仍系统性斜走，下一嫌疑项是三轮实际增益/负载不一致：IMU可以锁车身航向，但不能单独保证平移向量绝对正确。
- 本轮未修改MCU、全局摄像头或第二摄像头代码。主阻塞仍是通用`WAIT_VIS_STABLE`缺少位置帧进度看门狗。
### 2026-07-20 23:xx 位置帧“停止推进”的代码级边界与不改代码鉴别流程

- 全局摄像头`oa_id=7100425`的主循环目标每25ms调用`send_pos_update()`。即使当帧找不到车，仍会发送带`car_valid=0`的完整`0x12 POS_UPDATE`，并递增`frame_id`。
- MCU的`vision_link_parse_pos_update()`对校验和格式合法的有效/无效车位包都会递增`pos_packets`。因此“车不动”或“摄像头短暂找不到车”都不应使`pos_packets`停止。
- 7100920的`POINT_FUSION_NO_FRAME_TICKS=100`，即VIS_DRAIN结束后500ms没有新的可解析位置帧就走fallback。本轮`1027 -> 1029 -> 1030 -> 1034`后触发fallback，说明存在至少一个超过500ms的合法位置包空窗；停车后遥测中的`pos=1034/age=175ms`是point/follower终态快照，不能单独证明停车后UART仍在收包。
- 当前日志不包含故障瞬间的`rx_bytes/checksum_errors/format_errors/ring_drops`连续差分，所以还不能在“摄像头主循环/UART停发”、“线路干扰导致包损坏”和“MCU环形缓冲/解析来不及”之间定性。
- 不改代码鉴别流程：下次任务停止后不要重置，立即执行`S -> B -> 4 -> 等待2秒 -> 4`，保留两份STATUS。`rx`/`pos`都增加表示只是短时空窗后自愈；`rx`增加但`pos`不增且`err`增加表示包损坏/解析失败；`rx`不增且`age`持续增加表示摄像头发送或物理UART中断；`drop`增加则表示MCU环形缓冲溢出。

### 2026-07-20 7100920目标点推箱后的主动暂停根因

- 日志：`C:\Users\L\.codex\attachments\a97e143c-e1e2-4732-9e0e-462dd90f4259\pasted-text.txt`；固件仍为`7100920 / OBJECT_NEAR_START_REANCHOR`。
- 前序多次PUSH均完整经过`ENC_IMU_MOVE -> HOLD_500MS -> VIS_DRAIN -> VIS_STABLE/WAIT_VIS_STABLE`，并正常完成，说明本轮不是整条控制链、PID或推箱逻辑普遍失效。
- 停车发生在`action=9/51, step=13/78`：车计划从`(6,8)`到`(6,9)`，箱子从`(6,9)`推到目标`(6,10)`。日志中的`goal=1 consumed=1`只表示算法按计划把该箱子和目标从逻辑地图中消耗，并不是物理视觉复核已经证明箱子到位。
- 此步遥测明确为`fast=1`。源码`mission_manager.c`对“PUSH且箱子目标格为goal”设置`fast_finish=1`；`point_test.c`在停车保持500ms后直接将该步标为DONE，跳过常规视觉排空、稳定帧确认和短距ALIGN。
- 视觉最终位置仅约`(6.0,8.3)`，而车目标是`(6.0,9.0)`，尚差`140mm`。该误差超过当前`abs_accept=120mm`，随后`POST_PUSH_REANCHOR`判定目标格不匹配，任务主动进入`PAUSED_REPLAN / REPLAN_REQUIRED`并锁住电机。因此不是机械卡死、PID故障或漏按N/A。
- 最可能的物理原因是目标点推箱负载下轮胎打滑、箱体阻力或接触压缩，使编码器已走完而车体实际只推进约60mm；直接放宽140mm门限会让车在约0.7格偏差下继续运行，不可取。
- 后续修改方向：取消“目标点PUSH无条件fast-finish”。改为短确认分支：停车后等待1至2个新位置帧；若残差小则快速提交，若沿推箱方向仍差较多则在严格距离上限内补走并复核；若位置帧暂时缺失再采用有界的编码器+IMU回退。保留避免基地停留过久触发上位机清图的原设计目标，但不能跳过物理到位检查。
- 本轮只完成日志和源码分析，未修改MCU、全局摄像头或第二摄像头代码。

### 2026-07-21 7100924目标点推箱必须物理到位

- 队员B明确要求不能因目标点推箱误差而停住，首要目标是继续推完全部箱子。本轮属于`BRANCH-BLOCKING`完赛阻塞修复，不改变队员A规定的提速主线。
- 修改前备份：`project/_codex_backups/checkpoint_7100920_before_goal_push_finish_20260720_235633`；备份AXF SHA256=`7EC736ED79D524E1B64808C1C5E4F25ED4880118DAC1B699486089C496A3B045`。
- 新MCU识别数：`7100924 / GOAL_PUSH_VISUAL_FINISH`，`CONTROL_DEBUG_ENABLE=0U`。
- 修复内容：取消“PUSH且目标为goal”时无条件启用`fast_finish`。目标点推箱现在仍执行常规视觉排空、稳定确认和严格到位检查；普通步最大补走仍为120mm，仅目标点推箱允许最大180mm补走。实测日志中的140mm残差因此会进入ALIGN补完，不再直接`PAUSED_REPLAN`。
- 没有放宽严格完成标准：目标点推箱最终仍需满足物体邻域的20mm轴向/30mm合成误差标准，随后才提交箱子、执行`POST_PUSH_REANCHOR`并继续后续动作。PID、速度120、PWM、电机/编码器映射、IMU、CPR4096、里程比例0.915、F50/R30/B47/L20制动量、普通推箱和普通移动均未修改。
- Keil ARM Compiler 6.16已完整编译：`0 Error(s), 0 Warning(s)`；正式固件为`project/mdk/Objects/rt1064.axf`。固定副本为`project/mdk/Objects/rt1064_TEAM_B_7100924_GOAL_PUSH_VISUAL_FINISH_MODE0.axf`。
- 完成态检查点：`project/_codex_backups/checkpoint_7100924_GOAL_PUSH_VISUAL_FINISH_BUILT_20260721_000203`。
- 全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`且伪识别开启。
- 下一轮完整操作：`下载rt1064.axf -> 完全断电重上电 -> 确认7100924 -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1和PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> A`。
- 目标点推箱处的正常遥测特征：`goal=1`，`RUN_VIS fast=0 max=180mm`；若有明显欠程，应出现`HOLD_500MS -> VIS_DRAIN -> VIS_STABLE -> ALIGN_MOVE`，完成后进入`POST_PUSH_REANCHOR`并继续，而不是立即`PAUSED_REPLAN`。任何`fast=1`、同一位置持续锁车或任务转为`FAULT/PAUSED_REPLAN`都应立刻保留日志并报告。

### 2026-07-21 7100925外边界视觉夹持兼容

- 日志：`C:\Users\L\.codex\attachments\2c7cfcbc-e490-41d2-abae-9aad104331d9\pasted-text.txt`。本轮属于`BRANCH-BLOCKING`完赛阻塞修复，完成后立即回归完整A验证。
- 7100924已经证明上一轮目标点推箱修复有效：`(6,8)->(6,9)`且箱子`(6,9)->(6,10)`的`goal=1`步骤为`fast=0`，完成视觉补走后以`delta=(0,0)`进入`POST_PUSH_REANCHOR`并继续；因此“第二个箱子没推完”不是本轮根因。
- 真正锁停发生在后续自由移动`action=11 wp=1/2`：车沿左外边界从`(1,9)`到`(1,8)`，全局视觉连续稳定报告`(1.2,8.0)`，即只在朝墙法向固定偏差40mm。三次ALIGN无法改变该上位机边界夹持值，最后按原设计进入`POSE_REPLAN / GRID_MISMATCH`。
- 修改前备份：`project/_codex_backups/checkpoint_7100924_before_boundary_clamp_20260721_002003`。
- 新MCU识别数：`7100925 / BOUNDARY_TARGET_CLAMP`，`CONTROL_DEBUG_ENABLE=0U`。
- 新规则只在以下条件同时满足时生效：单格、严格位置模式、非PUSH、视觉四舍五入后已经在目标格、目标位于地图外边界内侧一格、存在对应墙轴、编码器步骤DONE、IMU相对航向误差不超过15度、沿运动方向误差已经满足严格20mm、向墙法向误差不超过60mm。此时只把向墙误差归零，并按逻辑目标格重锚。
- 安全边界：PUSH/炸弹交互因`interaction_locked=1`绝不使用该容差；远离墙方向的偏差不放过；内场墙不放过；跨格、沿运动方向误差、超过60mm误差不放过。PID、速度、PWM、编码器/电机映射、IMU、CPR4096、里程比例0.915、制动参数和两台摄像头均未修改。
- Keil ARM Compiler 6.16编译结果：`0 Error(s), 0 Warning(s)`。AXF SHA256=`5833AE26BA49542981BBDD35A6691511801075663AAF3DB1C3987B01D7FD6674`。固定副本：`project/mdk/Objects/rt1064_TEAM_B_7100925_BOUNDARY_TARGET_CLAMP_MODE0.axf`。完成态检查点：`project/_codex_backups/checkpoint_7100925_BOUNDARY_TARGET_CLAMP_BUILT_20260721_002955`。
- 全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`且伪识别开启。
- 下一轮从重新上电开始：`下载rt1064.axf -> 完全断电重上电 -> 确认7100925/BOUNDARY_TARGET_CLAMP -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> A`。
- 本轮验收重点：第二个箱子目标点推送仍应`fast=0`并完整到位；到左边界`(1,8)`附近应出现`wall_snap candidate=1 accepted=1 cross=-40`一类记录，任务继续而不是`POSE_REPLAN`。若出现PUSH步骤也被`wall_snap accepted=1`、沿边界偏差超过60mm仍放行或再次停住，立即按`X`并保留完整日志。

### 2026-07-21 7100926开局边界回格心与终态静默

- 日志：`C:\Users\L\.codex\attachments\421224ec-dc2e-4987-bc39-133db11ac21c\pasted-text.txt`；固件为`7100925 / BOUNDARY_TARGET_CLAMP`。
- 已确认：`E`没有启动正式路径，而是先执行`START_POSE_WAIT -> START_POSE_ALIGN`。逻辑起点为`(1,8)`，全局视觉在左边界稳定量化为约`(1.2,8.0)`；7100925的普通移动边界兼容依赖步骤墙轴上下文，开局合成回格心没有该上下文，因此三次向左短距ALIGN使实车越过边线。
- 已确认：三次失败后任务进入`PAUSED_REPLAN / REPLAN_REQUIRED`且`motor=LOCKED`；之后串口持续输出是因为`pc_console`仍对`MISSION_PAUSED`执行250ms周期遥测，不表示电机仍在运行。
- 7100926仅做两项修复：开局目标位于四条外边界内侧第一格时，同目标格、朝墙法向且不超过60mm的视觉残差归零，不再启动向墙外的短距ALIGN；从周期遥测活动态中移除`MISSION_PAUSED`，仍保留状态变化时的一次最终快照和手动`S`查询。
- 未修改：正常一步移动的7100925边界规则、PUSH/炸弹交互、PID、速度、PWM、电机/编码器映射、IMU、CPR4096、里程比例0.915、制动参数、规划器及两台摄像头协议。
- 修改前备份：`project/_codex_backups/checkpoint_7100925_before_start_edge_fix_20260721_005302`。完成态：`project/_codex_backups/checkpoint_7100926_START_EDGE_CLAMP_QUIET_BUILT_20260721_005630`。Keil编译`0 Error(s), 0 Warning(s)`；AXF SHA256=`EBAFDCAF008551EBD680856AD8CC0BF1739E079561DF499D239215B4F2CCA1E3`。
- E-only验收流程：`下载rt1064.axf -> 完全断电重上电 -> 确认7100926/START_EDGE_CLAMP_QUIET -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT`。本轮到此停止，不按`N/A`。
- E-only通过标准：`E`后电机始终锁定；允许短暂`START_POSE_WAIT`，但不应出现`START_POSE_ALIGN`、`ALIGN_ENC_IMU`或车体位移；最终进入`STEP_WAIT`。若车有任何移动，立即按`X`或断电并回传日志。

### 2026-07-21 7100930边界/分轴/航向修复与点到点执行

- 任务归类：`BRANCH-08 / BRANCH-BLOCKING`局部安全修复，同时推进`MAIN-01`点到点提速。修改前完整备份：`project/_codex_backups/pre_7100930_SAFE_ALIGN_P2P_20260721_023340`。
- MCU：`7100930 / SAFE_ALIGN_P2P_HEADING`，`CONTROL_DEBUG_ENABLE=0U`。标准固件为`project/mdk/Objects/rt1064.axf`，AXF SHA256=`0AC5E2A0FDA4960FAEC5E4695B346F8D204BF273B491AFEAF90347094EDC8F0C`；Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`。完成态：`project/_codex_backups/checkpoint_7100930_SAFE_ALIGN_P2P_HEADING_BUILT_20260721_025308`。
- 外边界：停车复查时，只要目标格本身位于`x=1/x=14/y=1/y=10`外边界内侧、视觉已在目标格、非PUSH、残差朝墙且不超过60mm，就钳掉朝墙分量；不再要求运动方向与墙平行。因此`(2,9)->(1,9)`且视觉夹为`x=1.3`时，不再向`x=1.0`外侧反复ALIGN。远离墙、超过60mm、跨格、PUSH均不放行。
- 物体邻域：`BOX_NEAR_PRECISE/PUSH_PRECISE`的停车短距ALIGN不再执行二维合成斜线。次轴不超过20mm时直接抑制；两轴都明显时只执行一条轴，获得新视觉帧后再决定第二条轴。PUSH优先沿计划推送轴补足主体到位，避免车身斜扫箱子。
- 开局航向：`E`完成格心位置对中后，新增`START_HEADING_WAIT`采集6个新视觉角度帧。若同一象限、角度跨度不超过10°且相对最近0/90/180/270方向偏差在3°~12°，以速度60低速物理转正一次，随后再采6帧并以当前IMU建立全程航向锁；视觉角度不稳定时最多等5秒，不依据抖动角盲转。遥测新增`RUN_HEADING`。
- 点到点：`N`改为执行当前同方向、连续、执行上下文完全兼容的安全直线段，一次最多13格；即使切到STANDARD，`N`仍是点到点。新增`G`严格只执行一格。`A`在FAST_SAFE下自动合并，在STANDARD下逐格。箱子/炸弹/目标点八邻域、PUSH、方向变化、墙体关系变化和通道转场都会截断合并，继续`segment=1 strict=1`。
- 未修改：移动速度120、PID/PWM、三轮与编码器映射、IMU参数、CPR4096、里程比例0.915、F50/R30/B47/L20制动提前量、规划器路径、PUSH世界模型、两台摄像头代码与协议。
- 局部优先验收流程：`下载rt1064.axf -> 完全断电重上电 -> 确认7100930/SAFE_ALIGN_P2P_HEADING -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1与PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待START_HEADING_WAIT/可选START_HEADING_ROTATE/最终STEP_WAIT -> S -> G -> 等待STEP_WAIT -> N -> 等待STEP_WAIT -> S`。
- 局部通过标准：`E`若初始物理偏角3°~12°，允许一次低速原地转正，但不可平移；若本来已正则不转。`G`必须`segment=1`。`N`在安全直线应为`segment=2..13`，进入物体邻域前必须停段。边界停车复查不得出现朝墙的斜向ALIGN；物体附近若两轴都有误差，连续两次ALIGN必须分别为单轴。
- 局部通过后完整验收：不重新规划可直接在RUN菜单按`A`继续剩余任务。要求所有PUSH为`segment=1 strict=1`、目标点推箱仍为`fast=0`并进入`POST_PUSH_REANCHOR`、最终`MISSION_COMPLETE`。异常立即按`X`，回传从`#MCU_BOOT`到停车后的完整日志。
- 摄像头：全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`，伪识别开关开启。

### 2026-07-21 7100931航向负反馈与FREE整段点到点

- 实测日志：`C:\Users\L\.codex\attachments\3f568fc2-4492-4f7d-898c-d1a2d69562e6\pasted-text.txt`。日志证明大偏移不是先由视觉矫正引起：在`action=2 (3,8)->(2,8)`中，IMU航向从约`3°`连续增加到`430°`，航向误差从`-10.5°`扩大到`-399.1°`，控制输出长期饱和为`-50`；三个轮子公共负目标使当前硬件的IMU航向继续增大，构成明确正反馈。等待阶段仍施加该错误公共轮速，所以车在`HOLD/VIS_DRAIN`中继续自转，随后视觉位置和角度才出现大跳变。
- 7100931把`YAW_HOLD_ROT_SIGN`由`+1`改为`-1`，运动与静止航向环都使用`normalize_angle_180(target-now)`。同样条件下，当前航向高于目标时输出应转为公共正轮速，促使误差缩小；跨360°时也始终走最短恢复方向。
- 7100930仍逐格的原因已确认：合并条件要求每个小格都`merge_eligible`且墙体/物体上下文完全一致，箱子、炸弹、目标点八邻域会直接禁止合并。日志所跑路线长期位于重叠八邻域，因此即使按`A`也一直`segment=1`。
- 7100931改为：`N`或FAST_SAFE下的`A`把同一`ACTION_FREE_MOVE`内、连续且同方向的全部子步一次合并，最多13格；不再因中间格的物体八邻域或墙体上下文变化停车。整段终点重新生成环境上下文，`segment>1`强制终点重锚；终点邻近物体/目标时仍用`BOX_NEAR_PRECISE strict=1`，多格终点也可使用外边界朝墙残差夹持。转弯、PUSH、OBSERVE和P1/P2阶段边界仍自然截断，PUSH始终逐格。
- 修改前备份：`project/_codex_backups/pre_7100931_YAW_NEGFB_FREE_P2P_20260721_030905`。完成态检查点：`project/_codex_backups/checkpoint_7100931_YAW_NEGFB_FREE_ACTION_P2P_BUILT_20260721_031621`。
- MCU：`7100931 / YAW_NEGFB_FREE_ACTION_P2P`，正式AXF为`project/mdk/Objects/rt1064.axf`，固定副本为`project/mdk/Objects/rt1064_TEAM_B_7100931_YAW_NEGFB_FREE_ACTION_P2P_MODE0.axf`，SHA256=`4419B7687B8E1AFA2FEFBD36AEB47E3384ACA5E0299B90DA6DA4921FDC143F36`。Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`；助手未下载到MCU。
- 全局摄像头未修改，仍为`oa_id=7100425`。第二摄像头未修改，仍为`fp_id=7200102`且`FAKE_RECOGNITION_ENABLE=True`。
- 下一步完整流程：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100931 stage=YAW_NEGFB_FREE_ACTION_P2P -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1与PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> A`。本轮不先按`G`，直接完整A提高测试效率。
- 验收判据：同向FREE多格应直接出现`MISSION_EXEC segment=2..13`并一次抵达该action终点，中间不回到`STEP_WAIT/HOLD/VIS_DRAIN`；PUSH应保持`segment=1 strict=1`。`RUN_CTRL yaw`在受扰后必须表现为`abs(err)`下降，不能再次出现`now`单向增加数百度且`corr`同向饱和。若仍有明显自转或航向误差持续扩大，立即按`X`并回传从该FREE开始前到停止后的日志。

### 2026-07-21 7100932撤销错误航向符号并落实开局边界夹持

- 7100931 E-only日志：`C:\Users\L\.codex\attachments\0ff21e3e-9a22-4bb6-8ea3-7205ee2865a6\pasted-text.txt`。按E后先进入`START_POSE_ALIGN`，初始IMU约0°；当航向偏到+5°时，7100931输出公共`+49.7`，随后公共轮速保持+50，IMU依次增至39°、83°、120°、156°、187°，最终约405°。这直接证明当前硬件公共正轮速会增大IMU航向，7100931的`YAW_HOLD_ROT_SIGN=-1`是正反馈，上一轮符号推断被实测否定。
- 7100932恢复`YAW_HOLD_ROT_SIGN=+1`。保留`normalize_angle_180(target-now)`，因此航向仍使用最短角，不再出现累计误差超过360°的问题。
- 同一日志还证明开局边界夹持未真正传入执行器：逻辑起点`(1,9)`，视觉约`(1.3,8.5)`，应只沿Y移动100mm并忽略朝左外墙的60mm误差；旧代码虽然在mission层把X残差钳为0，但`action_follower_start_pose_reanchor()`又按格心重新计算`dx=-60mm`，实际给出斜向`map_dir=211°`。7100932在follower建立短距目标时直接钳住边界轴，并把该轴目标设置为当前视觉坐标，确保不会重新产生向墙分量。
- FREE整段点到点修改没有撤回：同向FREE action在`N/A`下仍一次执行到终点；PUSH保持单格。
- 修改前备份：`project/_codex_backups/pre_7100932_YAW_SIGN_REVERT_20260721_032848`。完成态：`project/_codex_backups/checkpoint_7100932_YAW_REVERT_START_EDGE_FREE_P2P_BUILT_20260721_033355`。
- MCU：`7100932 / YAW_REVERT_START_EDGE_FREE_P2P`，AXF SHA256=`4F97FE996908245D001376CBED7853D8B7839A31F561F5F2A90C94529D5C9445`，Keil构建`0 Error(s), 0 Warning(s)`，助手未下载。全局摄像头未修改，`oa_id=7100425`；第二摄像头未修改，`fp_id=7200102`且伪识别开启。
- 下一步只做E-only：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认7100932/YAW_REVERT_START_EDGE_FREE_P2P -> 1 -> 等待MAP_END并核对元素 -> 2 -> D -> 等待valid=1/current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S`。不要按`N/A`。
- E-only通过标准：允许为了回格心做一次有界平移，但不能连续旋转；左边界60mm朝墙残差不得产生向外运动；IMU航向误差应收敛，最终`MISSION state=STEP_WAIT`。通过后下一轮直接在RUN菜单按`A`验证FREE点到点。

### 2026-07-21 7100933起步小角度旋转接口修复与非致命回退

- 实测日志：`C:\Users\L\.codex\attachments\6a08bbf0-f391-4015-8447-8ff6a8dfb8bb\pasted-text.txt`。固件确认为7100932，地图规划`valid=1/current=1`且`PLAN_CHECK ok=1`。按E后没有再发生7100931的连续自转，也没有产生左边界向墙外移动，证明7100932的航向符号回退与实际边界夹持有效。
- 新故障发生在`START_HEADING_WAIT`第6个新视觉角度帧：样本约85.2/85.6/90.0°，中值要求朝90°做约`+4°`修正，随后立即`FAULT/OBS_ROTATION_FAILED`。代码检查确认`point_test_set_rotation_angle()`只接受45/90/135/180/360°，与上层明确允许3~12°开局微调的设计冲突；不是传感器失联，也不是控制器再次正反馈。
- 7100933把底层旋转角合法范围改为连续`1~360°`。串口标定菜单仍只提供45/90/135/180°等标准选项，不改变既有操作；任务层现在可请求3~12°微调。若可选微调因其他瞬态状态仍启动失败，任务记录一次坏帧后使用当前IMU航向继续，避免一个精度增强功能锁死整场。
- 未修改：`YAW_HOLD_ROT_SIGN=+1`、最短角归一化、速度120、所有PID/PWM、编码器/电机映射、CPR/里程比例、制动提前量、视觉阈值、地图规划、PUSH和两台摄像头。
- 修改前备份：`project/_codex_backups/pre_7100933_START_HEADING_FINE_ROT_20260721_034651`；构建完成态：`project/_codex_backups/checkpoint_7100933_START_HEADING_FINE_ROT_FALLBACK_BUILT_20260721_035100`。MCU：`7100933 / START_HEADING_FINE_ROT_FALLBACK`，AXF SHA256=`4D667A5D8D4C05F1A3F181A1793888B3225F86340EBC762151E5E894E250EF45`，Keil构建`0 Error(s), 0 Warning(s)`，助手未下载。全局摄像头未修改，`oa_id=7100425`；第二摄像头未修改，`fp_id=7200102`且伪识别开启。
- 下一步仍只做E-only，完整流程：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认7100933/START_HEADING_FINE_ROT_FALLBACK -> 1 -> 等待FULL_MAP received/MAP_END并核对元素 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1与PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待START_HEADING_WAIT完成 -> S`。不要按`N/A`。
- 通过标准：若视觉中值偏差3~12°，允许一次低速、短促的`START_HEADING_ROTATE`，角误差应下降且转后进入`STEP_WAIT`；若微调无法启动，应直接降级进入`STEP_WAIT`而不是FAULT。确认后立即按`A`做完整FREE点到点和PUSH逐格验收。

### 2026-07-21 7100933实测结论与7100934软件航向参考

- 实测日志：`C:\Users\L\.codex\attachments\a1ed633b-fee9-426e-b331-759061374fdb\pasted-text.txt`。7100933地图、规划、N2 P1路径均正常，`PLAN_SUMMARY valid=1`、`PLAN_CHECK ok=1`，按`E`后最终进入`STEP_WAIT`，说明BRANCH-11的接口/回退故障已经排除。
- 已确认上一版没有执行物理微转：六帧中值报告`correction=2.4° spread=5.2° corrected=0`，偏差低于3°触发线。开局`START_POSE_ALIGN`从视觉`(1.4,8.1)`向逻辑`(1,8)`做约80mm/20mm短移，过程中IMU由0°漂到约-3.4°；随后静止航向环约17 counts、约460~500 PWM不足以克服小角度静摩擦，底盘保持偏斜。
- 全局视觉角度不适合物理微调：同一阶段IMU基本稳定时，视觉角可由约66.8°跳到101.3°；因此只用多帧稳健角估计地图/车身坐标偏角，不能把每个视觉角直接作为原地旋转目标。7100933长FREE `action=6 (2,6)->(10,6)`共8格/1.6m仍使用固定`used=90.0`，最终视觉约`(9.6,5.5)`，相对`(10,6)`误差80/100mm并`GRID_MISMATCH`；2.4°基准误差在1.6m产生约67mm横移，与现象同量级。
- 7100934取消起步物理微转。六帧稳定样本先求相对最近基准方向的中值误差，再反推出稳健视觉参考并写入`follower_map_heading_ref_deg`；IMU只锁定武装后的当前车身姿态。例：中值视觉约87.6°时，向地图右方移动的车身指令会补约2.4°，而不是要求底盘原地转2.4°。
- 起点外边界朝墙夹持从60mm扩大到100mm，专门覆盖上位机把左边缘真实位置显示为`x=1.4`这类情况，避免为追逐不可达格心先做80mm朝墙移动并扰动航向；另一轴仍按原规则对中。正常内场误差、远离墙方向、PUSH和运行中边界保护不变。
- 未修改：速度120、全部PID/PWM、三轮/编码器映射、CPR4096、里程比例0.915、制动提前量、视觉阈值、规划器、PUSH/N2/FP2、多格FREE上限。修改前备份：`project/_codex_backups/pre_7100934_SOFT_HEADING_REF_EDGE100_20260721_040021`；完成态：`project/_codex_backups/checkpoint_7100934_SOFT_HEADING_REF_EDGE100_P2P_BUILT_20260721_040425`。
- MCU：`7100934 / SOFT_HEADING_REF_EDGE100_P2P`；固定AXF：`project/mdk/Objects/rt1064_TEAM_B_7100934_SOFT_HEADING_REF_EDGE100_P2P_MODE0.axf`；SHA256=`91D19E010E6B7926F5B577FF8AE25A3BE0ED68B07241D35964EF93F104CD65E8`；Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`；助手未下载。全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`且伪识别开启。
- 下一步直接做一次完整A，提高调试效率：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100934 stage=SOFT_HEADING_REF_EDGE100_P2P -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> S -> A -> 任务停止或完成后S`。
- 7100934验收重点：`E`后不应出现`START_HEADING_ROTATE`；若起点仅有不超过100mm的朝外墙残差，不应为该分量移动；`RUN_HEADING correction`允许非零，`FOLLOW/RUN_TELEM used`应接近六帧稳健视觉参考（例如87.6）而不再固定90.0；同一8格长FREE的横向漂移应比7100933的约100mm明显下降，且不再因同类128mm终点误差暂停。PUSH仍必须`segment=1 strict=1`。若车明显失控立即按`X`，仅“车体肉眼略斜”不作为停止条件，判断以长段横向误差是否下降为准。

### 2026-07-21 7100935物体八邻域几何留距与快速段截断

- 任务归类：`BRANCH-13 / BRANCH-BLOCKING`。它直接解决“非计划擦碰箱子后，动态世界与旧计划失配而无法完赛”，服务于主线`MAIN-01`稳定前提下提速；完成本轮实车判定后立即回归完整A，不展开PID或卡尔曼支线。
- 实测日志：`C:\Users\L\.codex\attachments\796972de-427f-4997-9d63-44c2bc4b124e\pasted-text.txt`。固件为7100934，六帧软件地图航向参考已生效（示例`ref/used=92.4deg`），IMU航向误差多数只有数度，未出现航向环正反馈。视觉车角却会在约90/96.3deg间抖动，后段还跳到约71.6~78.7deg，不能用来做连续小角度物理伺服。
- 根因边界：7100934只保证“车中心运动方向”与地图轴一致，不能把有小角度偏斜的实体车壳摆正；精确追逐格心仍可能让车壳角部擦到相邻箱子/炸弹。原FAST_SAFE合并还只检查整段首尾上下文，可能跨过中间物体八邻域，不满足“物体附近必须逐格强处理”的既定规则。
- 7100935为每个非PUSH FREE步骤依据动态`mission_expected_map`中的箱子和炸弹位置计算安全目标偏置：正交相邻留距50mm，斜对角相邻每轴35mm，每轴最大50mm；同时检查目标格另一侧墙体，若避物方向会把车推向墙则取消该轴偏置。偏置同时考虑步骤起点和终点附近物体，所以进入、沿侧通过和离开物体邻域均有保护。
- 真正`PUSH_BOX/PUSH_BOMB`保持`clear=(0,0)mm`且目标仍是原格心。推前FREE若停在箱后50mm，下一PUSH按绝对目标自动把主体距离从200mm增加到约250mm，先有意接触再完整推一格；未修改算法中的箱子起点、终点或动态地图提交规则。
- FAST_SAFE合并改为逐小格检查执行上下文：仅当当前格和候选下一格均`merge_eligible`时继续合并。箱子/炸弹八邻域、目标点邻域、墙体关系转场及PUSH都会截断为`segment=1`；完全空旷的同向FREE仍可点到点，主线提速能力保留。
- 串口`EXEC_CONTEXT`、`MISSION_EXEC`和`RUN_VIS`新增`clear=(x,y)mm`。正常空旷段应为`clear=(0,0)`；物体邻域FREE通常出现35或50mm的非零分量；PUSH必须始终为0。若两侧物体对称或避让方向紧邻墙体，偏置会安全地抵消/钳为0。
- 未修改：速度120、PID/PWM、三轮/编码器映射、IMU参数、CPR4096、里程比例0.915、F50/R30/B47/L20制动、求解器Action语义、PUSH后地图复核、视觉阈值和两台摄像头代码。
- 修改前备份：`project/_codex_backups/pre_7100935_OBJECT_CLEARANCE_20260721_042220`。完成态：`project/_codex_backups/checkpoint_7100935_OBJECT_CLEARANCE50_BUILT_20260721_042856`。
- MCU：`7100935 / OBJECT_CLEARANCE50_NO_MERGE`，`CONTROL_DEBUG_ENABLE=0U`。Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`；AXF SHA256=`AF78DD46892A4C4667D933DEE2A61694DFB3F7877C6736A6DCEA078915FB130F`。固定副本：`project/mdk/Objects/rt1064_TEAM_B_7100935_OBJECT_CLEARANCE50_MODE0.axf`。助手未下载MCU。
- 自动验证：避碰几何6类场景（推前留距、沿箱侧通过、斜对角、墙体保护、真正PUSH零偏置、两侧物体抵消）`6/6`；经典C求解器17张地图及故障起点语义回放`18/18`；编译和识别数/模式0静态检查均通过。
- 摄像头：全局摄像头未修改，仍为`oa_id=7100425`；第二摄像头未修改，仍为`fp_id=7200102`且`FAKE_RECOGNITION_ENABLE=True`。
- 下一步直接完整A，避免重复慢测：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100935 stage=OBJECT_CLEARANCE50_NO_MERGE -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S -> A -> 任务停止或完成后S`。
- 重点观察：空旷直线仍可`segment>1 clear=(0,0)`；进入箱子/炸弹八邻域必须变为`segment=1 strict=1`，FREE应看到合理的`clear`分量，PUSH应为`clear=(0,0)`。若FREE偏置方向反而朝物体/墙、PUSH出现非零clear、仍发生非计划碰箱或箱位变化，立即按`X`并回传从该FREE前至少一格到停车后的完整日志。

### 2026-07-21 7100935实测与7100936推后40mm稳定目标格

- 实测日志：`C:\Users\L\.codex\attachments\dfde9c4a-7dd1-41ea-a46f-8bce74ee8057\pasted-text.txt`。固件确认是`7100935`。物体旁非PUSH FREE实际出现`gear=BOX_NEAR_PRECISE segment=1 strict=1 clear=(35,0)mm`并完成分轴矫正；随后真正PUSH为`gear=PUSH_PRECISE segment=1 strict=1 clear=(0,0)mm`，说明7100935的斜邻留距方向、物体邻域截断和PUSH零偏置均按设计工作。
- 停止点不是撞箱、跟随器故障或规划错误。`action=10/50`推箱`(2,10)->(2,9)`结束时，跟随器报告`DONE/NONE`，最终快照`car=(1.9,9.1)`、`delta=(20,-20)mm`、`dist=28mm`；推后状态随后进入`POST_PUSH_REANCHOR`，稳定视觉主要落在`car=(1.8,9.1)`，也就是`delta=(40,-20)mm`。
- 根因是两套成功阈值不一致：运动跟随器允许该动作正常结束，但`MISSION_POST_PUSH_TARGET_X10=1`只允许每轴20mm；稳定簇虽仍在逻辑目标格，却在2秒后被判为`MISMATCH/REPLAN_REQUIRED`。这会在物理推箱成功后阻断余下任务。
- 7100936仅把`MISSION_POST_PUSH_TARGET_X10`从1改为2，对应每轴40mm。稳定帧数仍为4帧，簇稳定时间仍为200ms，必须四舍五入到同一个逻辑目标格；60mm及以上或错误格仍不能通过。运动过程的严格20/30mm矫正、目标点推箱补走、2秒超时、地图语义、物体留距和PUSH零偏置均未放松。
- 修改前备份：`project/_codex_backups/pre_7100936_POST_PUSH40_STABLE_20260721_045517`；完成态：`project/_codex_backups/checkpoint_7100936_POST_PUSH40_STABLE_BUILT_20260721_050206`。MCU为`7100936 / POST_PUSH40_STABLE`，`CONTROL_DEBUG_ENABLE=0U`；固定固件`project/mdk/Objects/rt1064_TEAM_B_7100936_POST_PUSH40_STABLE_MODE0.axf`；AXF SHA256=`C660C0AAA8181EEFB8B75CBB6D1F8B9E4FADFAA7B940DC1E8D43284E4567ECF3`。Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`，助手未下载MCU。
- 自动验证：本次日志对应的20/-20mm和40/-20mm均通过；40/40mm边界通过；60mm和错误格拒绝，共`5/5`。经典C求解器17张地图加故障起点语义回放仍为`18/18`。
- 两台摄像头均未修改：全局摄像头仍为`oa_id=7100425`；第二摄像头仍为`fp_id=7200102`，`FAKE_RECOGNITION_ENABLE=True`。
- 下一步不重复单格慢测，直接完整A验证阻塞点并回归主线：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100936 stage=POST_PUSH40_STABLE -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S -> A -> 任务完成或停止后S`。
- 验收重点：与本次相同的PUSH后若稳定位置是`delta=(40,-20)mm`一类边界值，应该在4帧/200ms稳定后自动进入下一步，不再出现`PAUSED_REPLAN`；空旷FREE仍可`segment>1`，物体邻域FREE继续显示合理`clear`，PUSH继续`clear=(0,0)`。若再次停车，保留从最后一个正常PUSH前一格到停车后的完整日志。

### 2026-07-21 7100936实测与7100937推箱延迟视觉隔离

- 实测日志：`C:\Users\L\.codex\attachments\4ea8293e-959d-4e81-b5f4-548ad04c550b\pasted-text.txt`。固件确认为7100936。上一阻塞点已经通过：第一个PUSH结束后`post_push result=1 frames=4 at=(19,89)`，任务由action10继续到action11，说明7100936的40mm推后稳定目标格修复有效。
- 新停车点位于action12的PUSH：逻辑为车`(1,8)->(2,8)`、箱`(2,8)->(3,8)`。开始时稳定视觉车位约`(1.3,8.0)`，绝对目标为`(2.0,8.0)`，跟随器正确计算只剩140mm，并非盲走完整200mm。
- 真正根因是运动后的延迟视觉：主体运动结束后视觉仍报告旧位置约`x=1.4`，旧代码把120mm残差当成真实欠程并进入`ALIGN_ENC_IMU`。约250ms后视觉先追到`x=2.1`，随后继续追到`x=2.6~2.8`；该追加运动使箱子可能被多推，最终目标误差约`(-160,-40)mm`并`GRID_MISMATCH/PAUSED_REPLAN`。这不是7100936的40mm门槛再次过严，也不能通过继续放宽门槛解决。
- 7100937为真正`PUSH_PRECISE + interaction_locked`增加专用延迟视觉屏障：运动完成后绝不启动主动位置ALIGN，也不接受墙边snap；源格或其他旧帧只会使电机保持锁止并重新采样。3秒内看到目标逻辑格即结束动作，再由mission层原有4帧/200ms/每轴40mm推后确认复核；持续3秒仍在错误格才`GRID_MISMATCH`。
- 普通FREE、物体邻域FREE、推箱前强定位、目标留距、速度120、PID/PWM、IMU锁航向、三轮与编码器映射、地图算法、N2/FP2及两台摄像头均未修改。真正PUSH继续`segment=1 strict=1 clear=(0,0)`。
- 修改前备份：`project/_codex_backups/pre_7100937_PUSH_DELAY_GUARD_20260721_051824`；完成态：`project/_codex_backups/checkpoint_7100937_PUSH_DELAY_DRAIN_ONLY_BUILT_20260721_052544`。MCU为`7100937 / PUSH_DELAY_DRAIN_ONLY`，固定固件`project/mdk/Objects/rt1064_TEAM_B_7100937_PUSH_DELAY_DRAIN_ONLY_MODE0.axf`，AXF SHA256=`CB5A52335DC399FBBF8CCDAA15556CBC68FE7258053B9BD8B209AEA9B32E61B5`；Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`，助手未下载MCU。
- 自动验证：推箱延迟模型覆盖“旧源格后追到目标格”“持续错误格超时”“目标格内大残差交给推后门槛”与“全程零追加ALIGN”共`4/4`；经典C求解器17张地图加故障起点语义回放仍为`18/18`。
- 摄像头均未修改：全局摄像头`oa_id=7100425`；第二摄像头`fp_id=7200102`且`FAKE_RECOGNITION_ENABLE=True`。
- 下一步直接完整A复测同一路径：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100937 stage=PUSH_DELAY_DRAIN_ONLY -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S -> A -> 任务完成或停止后S`。
- 关键验收：PUSH主体运动结束后可进入`WAIT_VIS_STABLE`并短暂停车，但该PUSH期间不得再出现`phase=ALIGN_ENC_IMU`或`rebase=2`主动补走；视觉应由源格追到目标格，随后进入`POST_PUSH_REANCHOR`并继续。若3秒后仍错误格，允许安全暂停重规划；回传从该PUSH前一个FREE开始到停车后的日志。

### 2026-07-21 7100937完整实测与7100938推后80mm稳定目标格

- 实测日志：`C:\Users\L\.codex\attachments\5f0d7202-f288-41e9-84e8-98a54ecf76ca\pasted-text.txt`，固件确认是`7100937`。BRANCH-15的核心修复已通过：日志中的所有真正PUSH在主体运动结束后都没有再次进入`ALIGN_ENC_IMU`；炸弹PUSH已在延迟视觉追到目标格后完成`POST_PUSH_REANCHOR`并继续后续动作，说明7100937没有再多推一格。
- 最终停车发生在新规划`action=1/51`的箱子PUSH，逻辑车位`(3,9)->(3,8)`、箱子`(3,8)->(3,7)`。跟随器执行PUSH后锁车等待，延迟视觉最终稳定在`car=(2.7,8.1)`，相对目标误差约60/20mm；跟随器报告`DONE/NONE`且没有追加运动，但mission层两秒后进入`PAUSED_REPLAN`。
- 根因仍是两层容差不一致：跟随器严格物体模式允许相对100mm/绝对120mm的目标格稳定位置结束，mission层却仍使用每轴40mm。此次60mm残差仍四舍五入到目标格，且车辆已经锁止、位置帧持续稳定，因此属于有效落点而不是跨格错误。
- 7100938仅把`MISSION_POST_PUSH_TARGET_X10`从2改为4，对应每轴80mm。目标格判定、4个新视觉帧、200ms稳定、错误格拒绝、PUSH后零主动ALIGN和3秒延迟追帧屏障全部保留；100mm及以上仍不能通过。未修改速度、PID/PWM、IMU、编码器、电机映射、物体留距、规划器、N2/FP2或摄像头。
- 修改前备份：`project/_codex_backups/pre_7100938_POST_PUSH_TOLERANCE_20260721_053822`；完成态：`project/_codex_backups/checkpoint_7100938_PUSH_POST_REANCHOR80_BUILT_20260721_054300`。MCU：`7100938 / PUSH_POST_REANCHOR80`，`CONTROL_DEBUG_ENABLE=0U`；固定固件：`project/mdk/Objects/rt1064_TEAM_B_7100938_PUSH_POST_REANCHOR80_MODE0.axf`；AXF SHA256=`5C7BB3E2734F52FB5FE344BD35A27C30438AECE5ED9150B52633EFEA336632C2`；Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`，助手未下载MCU。
- 自动验证：本次实测60/20mm、旧实测40/20mm、80/80mm边界和格心均通过；100mm及错误格拒绝，共`6/6`。本轮不改求解器或Action数据流；编译链接完成且固件ID三处一致。
- 摄像头均未修改：全局摄像头仍为`oa_id=7100425`；第二摄像头仍为`fp_id=7200102`，`FAKE_RECOGNITION_ENABLE=True`。
- 下一步直接完整A复测，不重复单格：`Keil下载Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100938 stage=PUSH_POST_REANCHOR80 -> 1 -> 等待FULL_MAP received/MAP_END并核对元素数 -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待STEP_WAIT -> S -> A -> 任务完成或停止后S`。
- 验收重点：本次同类PUSH在视觉稳定于目标格且每轴误差不超过80mm时应自动继续；PUSH主体运动后仍不得出现`ALIGN_ENC_IMU`。若出现100mm以上、错误格或跨格，允许重规划；若再次停车，保留最后一个正常PUSH前一格到停车后的完整日志。

### 2026-07-21 7100938实车确认完赛基线

- 用户已明确确认：`7100938 / PUSH_POST_REANCHOR80` 是可以完整完赛的实车稳定版本。它不再是“待验证候选”，而是后续所有提速修改的强制回退基线。
- 冻结备份：`project/_codex_backups/checkpoint_7100938_CONFIRMED_FINISH_BASELINE_20260721_060028`。备份包含完整`project/code`、`project/user`、Keil工程文件、标准与固定AXF、两台摄像头脚本、构建日志、本记忆文件和`BASELINE_INFO.txt`；后续实验不得覆盖。
- 基线固件：`project/mdk/Objects/rt1064_TEAM_B_7100938_PUSH_POST_REANCHOR80_MODE0.axf`；SHA256=`5C7BB3E2734F52FB5FE344BD35A27C30438AECE5ED9150B52633EFEA336632C2`；`CONTROL_DEBUG_ENABLE=0U`。
- 主线继续为“在不破坏完赛稳定性的前提下提速”。当前用户指定的下一步是：所有同一直线点到点位移一次运行到终点，包括多格FREE、连续推箱和连续推炸弹；仍必须保留起点对齐、IMU锁航向、途中边界约束、终点稳定视觉复核、错误格拒绝和可恢复回退。
- 摄像头基线不变：全局摄像头`oa_id=7100425`；第二摄像头`fp_id=7200102`且`FAKE_RECOGNITION_ENABLE=True`。

### 2026-07-21 7100939所有直线点到点与原子连续PUSH候选

- 用户要求：以已确认完赛的`7100938 / PUSH_POST_REANCHOR80`为基础，在保留稳定机制的前提下，把同一直线上的点到点位移一次运行到终点；范围同时包含多格FREE、连续推箱和连续推炸弹。`7100938`仍是强制回退基线，本候选未经实车完整A验证前不得替代它。
- 修改前快照：`project/_codex_backups/pre_7100939_ALL_P2P_ATOMIC_20260721_060640`。完成态检查点：`project/_codex_backups/checkpoint_7100939_ALL_P2P_ATOMIC_PUSH_BUILT_20260721_062458`。实车完赛冻结基线继续位于`project/_codex_backups/checkpoint_7100938_CONFIRMED_FINISH_BASELINE_20260721_060028`，不得覆盖。
- MCU候选：`7100939 / ALL_P2P_ATOMIC_PUSH`，`CONTROL_DEBUG_ENABLE=0U`。固定固件：`project/mdk/Objects/rt1064_TEAM_B_7100939_ALL_P2P_ATOMIC_PUSH_MODE0.axf`；标准AXF与固定副本SHA256均为`F87F45421E82F2198F1E71220C0E7C13503BF1FF3E72483EACB35CA3F9A22170`。Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`；助手未下载MCU，必须由操作者在Keil中下载。
- FREE点到点：默认`FAST_SAFE`下，连续同方向FREE不再因箱子/炸弹八邻域、目标点、单边墙、走廊或转场上下文而物理分段。生成一条长段前会逐格累计全部安全上下文，并采用整段最保守档位；物体/目标邻域仍使用`BOX_NEAR_PRECISE`，墙边/走廊/转场仍保留对应限制，冲突的单边墙纠偏方向会被关闭。整段最终端点继续强制稳定视觉重锚。
- PUSH点到点：连续同对象、同方向、几何连续的`PUSH_BOX`或`PUSH_BOMB`在算法适配层压成一个Action；不同对象、不同方向、OBSERVE、WAIT、阶段切换等仍是硬边界。执行前在私有地图上逐格预演整段，逐格检查车位、物体位、墙、其他箱子/炸弹和目标；箱子不得在最终步前提前进入目标，炸弹只允许在整段最后一步入墙引爆，且最终物体位置必须与Action元数据一致。预演失败时电机不会启动。
- 稳定机制未删除：开局格心重锚、视觉确定地图航向象限、IMU锁航向、物体附近精细档、PUSH延迟视觉屏障、PUSH后禁止主动追加ALIGN、目标逻辑格检查、4个新位置帧/200ms/每轴80mm推后稳定复核、错误格拒绝、动态地图只在物理整段完成后提交，均继续有效。
- 操作边界：默认`A`在`FAST_SAFE`下合并全部同向FREE/PUSH；`N`也强制执行当前连续同向段；`G`保留为逐格诊断/回退方式；`Q`切到`STANDARD`后，`A`可恢复逐格执行，便于无刷写临时对照。转弯、识别、等待、对象变化、方向变化和算法阶段变化不会被跨越。
- 自动验证：带炸弹/N2适配器除既定不要求的map7外全部`16/16`通过，日志`project/mdk/logs/adapter_p2p_7100939.txt`；已实际生成多格PUSH，例如map0最大连续7格、map5(4)最大11格、map9最大8格。经典求解器地图与语义回放`18/18`通过，日志`project/mdk/logs/classic_solver_7100939.txt`。Keil构建日志为`project/mdk/build_7100939.log`。
- 摄像头均未修改：全局摄像头仍为`oa_id=7100425`；第二摄像头仍为`fp_id=7200102`，`FAKE_RECOGNITION_ENABLE=True`。两份摄像头脚本与7100938冻结基线SHA256一致。
- 首轮实车只做一次完整A，不重复逐格慢测：`Keil下载project/mdk/Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100939 stage=ALL_P2P_ATOMIC_PUSH -> 1 -> 等待FULL_MAP received/MAP_END并核对wall/box/goal/bomb -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S -> A -> 完赛或异常停车后S`。首次不要按`Q`，保持默认`FAST_SAFE`。
- 验收重点：规划允许时FREE和连续PUSH应出现`segment>1`并一次运行到终点；PUSH主体结束后可以锁车等待延迟视觉，但不得出现主动`ALIGN_ENC_IMU`，随后应进入`POST_PUSH_REANCHOR`并继续。任何错误方向、撞到非目标物、炸弹提前引爆、跨越转弯/识别边界、持续失控或明显偏离时立即按`X`，并回退下载`rt1064_TEAM_B_7100938_PUSH_POST_REANCHOR80_MODE0.axf`。

### 2026-07-21 7100940连续推物的接触中心与轴向终点补足

- 任务归类：继续服务主线“在不破坏稳定完赛前提下提速”。这是`7100939 / ALL_P2P_ATOMIC_PUSH`实车暴露出的阻塞支线：连续推箱子和炸弹已经能一次运行，但车在开始接触与结束停靠时不够准，导致被推物体没有落到算法假定的格心。已确认完赛的`7100938 / PUSH_POST_REANCHOR80`仍是冻结回退基线，不被本候选替代或覆盖。
- 实测日志：`C:\Users\L\.codex\attachments\15b80399-8856-4e5b-936a-fa49d6aec970\pasted-text.txt`。代表性误差并不服从统一比例：向上推2格时纵向基本正确但横向约40mm；向下推2格时沿推力方向短约40mm；向左推7格时沿推力方向短约120mm并触发`POSE_REPLAN/GRID_MISMATCH`。因此不能通过统一增大里程比例、统一多推固定距离或修改CPR来解决，否则会让已准确方向过推。
- 7100939的另一个风险是：只要车位离计划`car_from`不超过120mm即可直接开始PUSH。车可能带着横向偏差或纵向欠位接触物体，误差随后被带入整段连续推物。7100940在每个PUSH动作装载后增加“推前接触中心预对准”：若视觉稳定位置相对计划`car_from`格心的残差大于30mm且不超过120mm，先在不接触物体的状态下低速对准；最多2次。边界格中朝地图外墙方向的残差会被钳掉，绝不为了追格心把车送向场外。
- 多格PUSH主体仍保持一次性原子运行，不恢复逐格停顿。主体结束后电机锁止2秒，让全局视觉延迟数据追上；随后只允许一次低速“沿原推力方向补足”，触发条件同时要求：终点误差仍在原推力前方、沿轴欠程30~120mm、横向误差不超过60mm。横向误差不会带着箱子/炸弹侧移，过推也不会反向拉车，超过120mm或错误格仍交给原有失败/重规划机制。
- 单格PUSH保留7100938已验证稳定的延迟视觉与推后重锚行为，只额外受益于推前接触中心预对准；多格PUSH的终点补足完成后仍进入既有`POST_PUSH_REANCHOR`，继续要求新位置帧、稳定时间、目标格与每轴80mm门槛。求解器Action、动态物体提交、炸弹引爆语义、N2/FP2通信、速度/PID/PWM、IMU、编码器映射、CPR4096、里程比例0.915和制动提前量均未修改。
- 串口`RUN_VIS`新增`pushfix=PREa/b+ENDc/d(along,cross)`。`PRE1/...`表示主推前执行过接触中心预对准；长推结束若真实欠程满足安全门控，应看到`END1/1`并只做一次沿轴补足。仅横向误差时必须保持`END0/...`，不能出现带载横移。
- 修改前备份：`project/_codex_backups/pre_7100940_PUSH_ACCURACY_20260721`，其中7100939 AXF SHA256仍为`F87F45421E82F2198F1E71220C0E7C13503BF1FF3E72483EACB35CA3F9A22170`。7100940完成态检查点：`project/_codex_backups/checkpoint_7100940_PUSH_CENTER_AXIS_FINISH_BUILT_20260721`。
- MCU候选：`7100940 / PUSH_CENTER_AXIS_FINISH`，`CONTROL_DEBUG_ENABLE=0U`。标准固件：`project/mdk/Objects/rt1064.axf`；固定副本：`project/mdk/Objects/rt1064_TEAM_B_7100940_PUSH_CENTER_AXIS_FINISH_MODE0.axf`；两者SHA256均为`B6CF609973C258E3A4EF1CC0611CBB27FEE34D1532AED35C9EE4A186CB23EA3E`。Keil ARM Compiler 6.16构建`0 Error(s), 0 Warning(s)`；助手未下载MCU，必须由操作者在Keil中下载。
- 自动验证：带炸弹/N2适配器除既定不要求的map7外`16/16`通过；经典求解器与语义回放`18/18`通过；推物终点门控5类场景`5/5`通过，包括“向下短40mm允许前补”“向左短120mm允许前补”“过推40mm禁止反向”“仅横向40mm禁止带载侧移”“横向80mm禁止补足”。日志位于`project/mdk/logs/adapter_p2p_7100940.txt`、`classic_solver_7100940.txt`和`push_gate_7100940.txt`。
- 摄像头均未修改或重新同步：全局摄像头仍为`oa_id=7100425`；第二摄像头仍为`fp_id=7200102`，`FAKE_RECOGNITION_ENABLE=True`。
- 下一步只做一次完整A，直接覆盖短推与长推场景：`Keil下载project/mdk/Objects/rt1064.axf -> 完全断电重上电 -> 确认#MCU_BOOT id=7100940 stage=PUSH_CENTER_AXIS_FINISH -> 1 -> 等待FULL_MAP received/MAP_END并核对wall/box/goal/bomb -> 2 -> D -> 等待PLAN_SUMMARY valid=1 current=1及PLAN_CHECK ok=1 -> B -> 3 -> E -> 等待最终STEP_WAIT -> S -> A -> 完赛或异常停车后S`。保持默认`FAST_SAFE`，不要按`Q`。
- 验收重点：同一张图中重点观察2格和7格连续PUSH。主推前若源位偏差超过30mm，应先出现`pushfix=PRE1/...`；主推后若只是沿推力方向欠30~120mm，可出现一次`END1/1`低速补足；横向偏差不得触发带载横移，过推不得反向。完成后必须继续进入`POST_PUSH_REANCHOR`并执行余下任务。若碰撞、对象落错格或异常停车，按`X`并保留从该PUSH前一个FREE到停车后`S`的完整日志；可立即回退固定固件`rt1064_TEAM_B_7100938_PUSH_POST_REANCHOR80_MODE0.axf`。

## 队员B阶段关闭与队员A接回

2026-07-21队员A已明确返回。队员B期间最重要的已确认成果是：`7100938 / PUSH_POST_REANCHOR80`被实车确认可以完整完赛，并冻结为后续修改的强制回退基线；7100939和7100940属于连续多格/连续PUSH提速候选，不能替代稳定基线。队员A随后把主线提升为“基地出发、地图加载、单图执行、完赛复核、返航、连续三图和离线按键菜单”的完整比赛闭环。

当前MCU电脑端候选已进入`7100941 / MATCH_MANAGER_UART_FULL3`，详细设计、完整菜单、已知假设和20小时计划见`design/7100941_完整三图比赛管理器与20小时调试计划.md`。本文件从此只作队员B历史证据源，不再继续追加队员A的日常调试内容；任何需要回看队员B建议时只读取对应小节，不覆盖队员A主记忆。
