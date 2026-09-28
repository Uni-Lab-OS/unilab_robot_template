"""珞石xMateCR12型号资产、正解与静态MoveIt模型入口。"""
from .factory import MODEL_DESCRIPTOR, MODULE_API_VERSION, MODULE_KIND, MODULE_VERSION
from .moveit_model import MoveItModelBundle, build_joint_state_name_map, build_moveit_model

__version__ = MODULE_VERSION
__all__ = ["MODEL_DESCRIPTOR", "MODULE_API_VERSION", "MODULE_KIND", "MODULE_VERSION",
           "MoveItModelBundle", "build_joint_state_name_map", "build_moveit_model"]
