"""
EL-A3 SDK 工具函数

数据映射转换、单位换算等。
"""

import math


def float_to_uint16(x: float, x_min: float, x_max: float) -> int:
    """浮点数线性映射到 uint16 (0~65535)"""
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * 65535.0 / (x_max - x_min))


def uint16_to_float(x_int: int, x_min: float, x_max: float) -> float:
    """uint16 (0~65535) 线性映射到浮点数"""
    return x_int * (x_max - x_min) / 65535.0 + x_min


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))
