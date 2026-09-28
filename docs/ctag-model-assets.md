# CTAG2F90 冻结型号资产

`unilab-end-effector-sim` 保存 CTAG2F90 的 13 份冻结来源：可移植 URDF、厂家原 URDF 和 11 个 STL，共 1,450,937 字节。来源字节未改；`models/ctag2f90/source-lock.json` 记录原始路径、摘要与包内路径。原始 URDF 作为来源保存，其旧相对路径不作为消费入口。

```python
from unilab_end_effector_sim.ctag_assets import prepare_ctag_model

model = prepare_ctag_model(output_directory, max_velocity_rad_s=0.5)
```

输出目录必须不存在。先验证包内来源清单及全部文件，再生成 `model.urdf`、网格闭包、`mechanics.json`、`provenance.json` 和沿用已有格式的 `bundle.json`。输出可整体搬移；没有旧工作区路径或默认现场目录依赖。`max_velocity_rad_s` 是显式开发参数，0.5 仅为旧实验值，不代表厂家速度。未构建或安装 wheel；包数据声明和独立检出搬移测试不等于安装包验收。

派生规则来自冻结 `work/ctag-dev-20260913/packages/labos_p4r/gripper.py::prepare_gripper`：CTAG 命名、仿射 mimic 展开、从动限位和惯量检查。六条顶层派生语句的 AST 除速度改成显式参数外保持一致。机械臂与场景从该入口排除；它只返回独立夹爪资产，不创建第二份部署图或顶层 `simulation` 配置。通用资产转换仍由 Asset_pipeline 维护。

当前输出沿用历史 `ctag_` 名称，是参考资产，**不是 OS 设备 Provider、MoveIt 包或双实例装配入口**。未来领域装配应按现有启动图选择安装位置，不把旧 CR12 名义固定安装、实验初始姿态、速度控制器或相机布局塞入型号包。没有修改领域仓，因此本次领域装配检查器不适用；新的六轴机械臂型号仍须遵循机械臂包规范。

原 URDF 的主关节范围保留为 `[0, 1]` 弧度；旧实验只允许 `[0.2, 0.9]`，因为低角度处源模型指垫发生穿越。此工具不派发命令，也不自动提供执行范围限制。几何转移和仿射关系不证明闭环四连杆动力学、90 mm 实物行程、抓力、接触、负载、安装螺孔或标定 TCP；这些资格均未授予。厂家标称质量也不用于补造源模型缺失质量。

限定验证：30 项 CPU 测试通过；新生成夹爪与原函数重新生成的组合模型中的 21 个 CTAG 元素一致，11 份惯量及 8 条限位派生一致，64 组角度相对正解最大差异小于 `1e-12`。旧实验原文件保持不变。Isaac 加载、物理步进及原 OS 执行链另行验收。
