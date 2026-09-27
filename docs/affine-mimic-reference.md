# URDF仿射mimic参考计算

`unilab_end_effector_sim.mimic.mimic_map(tree)` 从已解析的URDF Element展开每个非fixed关节，返回 `关节名 -> (独立源关节, multiplier, offset)`。支持嵌套偏置、负倍率和多个独立驱动，拒绝循环、未知/固定源关节及原实现禁止的零或非有限单项系数。函数AST与冻结CTAG来源一致，不包含CTAG名称、资产路径、开合范围或设备参数。

这只是名义仿射关系。实际关节反馈仍必须读回所有受观测关节，不能把命令按此展开后称为实测；函数不创建世界/线程、不写执行器、也不替代Isaac原生约束。URDF结构、重复命名、单位匹配和派生关节限位仍由调用者校验，深链系数乘积溢出沿用原处理边界，不能当通用URDF有效性验证器。

C30b：5项正式包检查通过（包括原两项mimic方法及现有逻辑操作组合），外部隔离视图的7项旧CTAG消费者检查通过，并断言其实际引用本函数。固定原CTAG URDF的8个关系与旧展开完全一致。没有运行实际Isaac/ROS、构建或实物；旧CTAG工作树和现有部署图未改。

现有 `SimulatedGripper` 仍是立即修改holding状态的逻辑模型，本次没有将它升级为接触夹持，也未让生产驱动自动使用新函数。CTAG指垫/装配、CR12型号及其资产后续按领域与型号包分别整理。新六轴包须走本仓package-arm-moveit规范，不能把旧组合URDF冒充六轴型号包。来源见 `migration/affine-mimic.json`。
