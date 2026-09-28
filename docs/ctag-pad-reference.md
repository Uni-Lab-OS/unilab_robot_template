# CTAG2F90 指垫参考几何

`unilab_end_effector_sim.ctag_reference` 保存冻结CTAG原型的两段型号专用几何函数：

- `pad_bounds(model)`：从调用方模型的 `ctag_Left_Pad_Link` / `ctag_Right_Pad_Link`
  读取相对路径的二进制STL，返回原网格轴对齐包围盒的八个角点。
- `projected_pad_gap(bounds, palm_relative_poses)`：把角点投影到掌心X轴，计算
  左指垫最小X减右指垫最大X；负值保留交叠含义，不截断为零。

模型必须显式提供 `xml` 与 `path`，位姿由调用方提供。模块没有默认旧工作区、资产搜索、
设备注册、时间推进或执行动作。可配合已有 `MimicModel` 使用；实际关节观测与名义mimic
正解仍须区分。两个函数保留旧AST，不改变通用夹爪的运行路径。

这是CTAG固定链接名/掌心轴约定下的参考算法。旧读取器按原始STL顶点处理，不合成
URDF visual origin/scale，不验证任意网格或单位；非二进制/截断输入沿用原错误边界。
调用方须核对资产摘要、几何语义与坐标。投影间隙不是实测毫米开度、完整碰撞、抓力、
闭环四连杆或接触资格；模块也不内置旧试验的0.20–0.90 rad命令限制。

限定验证：原7项CTAG消费者回归及仅替换两函数后的7项通过；包内相关17项通过，
其中包含原穿越用例、固定二进制网格角点、长度错误及旋转投影。冻结CR12+CTAG模型和
两份指垫网格按原bundle核对，64组名义开度的角点/间隙与原版完全一致，输入未变。
首轮误选缺PyYAML的CPU环境在收集时失败；使用现有PLC环境后通过，无安装或构建。

本次不迁入整机prepare_gripper、旧场景profile、执行器或Isaac后端；CR12型号、CTAG
安装/资产发行及物理执行仍须分别归位。未修改现有启动JSON、simulation、OS解析或
任何领域装配，所以机械臂领域装配检查器不适用于本次纯几何函数提取，也未据此授予装配通过。
来源及精确摘要见[迁移记录](../migration/ctag-pad-reference.json)。
