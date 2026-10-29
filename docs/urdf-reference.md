# 有限URDF参考正解

`unilab_robot_contracts.urdf_reference.Model(path, allow_mimic=False)` 读取调用方指定的有限URDF树，支持fixed、受限revolute/prismatic，返回所有link的齐次变换矩阵。关节顺序来自URDF；输入为各活动关节的完整观测向量，默认检查限位。它不读网格、不发运动指令、不自动选择资产，也不是新的CR12型号包或MoveIt Provider。

`unilab_end_effector_sim.urdf_reference.MimicModel(path)` 使用共享mimic关系，分别提供 `fk(全部观测关节)` 和 `nominal_fk(独立关节字典)`。前者不会按命令改写从动关节读数；后者只是名义仿射结果。实测/名义两条路径不能混用来证明跟随、抓取或停稳。

原Model、MimicModel及数字/原点/关节旋转/惯量检查方法AST保留，旧labos_sim坐标依赖替换为已有RigidTransform，矩阵边界仅保留兼容转换和乘法。没有复制完整SE3包，也未让现有CR5/CR7后端自动改用本实现。型号的限位/网格、CTAG指垫与安装配置仍归相应设备包；输入文件摘要由调用者核对。

18项包检查、11项外部旧消费者检查通过。三个真实已有URDF（CR12、CR12+CTAG、CR12+合成平行夹具）各64组、共192组关节状态与原正解比较，最大矩阵元素误差8.9e-16以内（预设容差1e-12）；惯量记录相同。这是数值迁移证据，不是这些模型的物理或实物资格。

模型保留旧输入边界：不是通用URDF解析器；格式残缺可产生原解析异常。惯量正定/二阶矩筛查只是必要代数条件，不等于参数已标定。调用方必须另行检查资产闭包、单位和来源；没有声明新OS注册、场景启动、统一仿真时序、碰撞或动态接触。来源及限定范围见 `migration/urdf-reference.json`。
