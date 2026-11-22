"""stm32_bridge.kinematics —— 用 ``ctypes`` 加载 ``firmware_core`` 编出的 ``libomninav_core.so``。

★★★ 铁律（``11_接口契约.md`` §3 末 / §5.3 + T2 任务卡「软依赖」）

    麦轮正逆解与里程计圆弧积分**只能有一份实现**，就在 ``firmware_core`` 的 C 代码里
    （``mecanum.c`` / ``odom.c``）。本文件**不做任何数学**，只把 C 符号包成 Python 函数。
    找不到 ``.so`` 时**必须抛清晰异常**，**绝不许**用 Python 自己算一遍 ——
    那就是第二份实现，迟早与 C 版分叉，而且分叉的那一天没有任何测试会报警。

``.so`` 查找顺序（契约 §5.3，**冻结**，第一个存在的就用）：

    1. 环境变量 ``OMNINAV_CORE_SO``（便于在笔记本 / WSL 上开发时指向别处）
    2. ``<本包目录>/lib/libomninav_core.so``
    3. ``/opt/omninav/lib/libomninav_core.so``   <- ``pi_setup/install_core.sh`` 负责装到这里

都不存在 -> :class:`CoreLibraryNotFound`；``.so`` 存在但少符号 -> :class:`CoreSymbolMissing`。
两种都是**明确报错、不降级**。
"""

from __future__ import annotations

import ctypes
import os
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "WHEEL_COUNT",
    "SONAME",
    "CORE_SO_ENV",
    "SYSTEM_CORE_SO",
    "REQUIRED_SYMBOLS",
    "CoreLibraryError",
    "CoreLibraryNotFound",
    "CoreSymbolMissing",
    "MecanumGeom",
    "MecanumTwist",
    "MecanumWheels",
    "OdomPose",
    "OdomTotal",
    "candidate_paths",
    "find_library",
    "Core",
    "default_core",
    "reset_default_core",
]

#: 轮数（FL, FR, RL, RR）。
WHEEL_COUNT = 4

#: 契约 §5.2 冻结的产物名。
SONAME = "libomninav_core.so"

#: 契约 §5.3 第 1 条：开发时用环境变量覆盖。
CORE_SO_ENV = "OMNINAV_CORE_SO"

#: 契约 §5.3 第 3 条：``pi_setup/install_core.sh`` 装到的地方。
SYSTEM_CORE_SO = "/opt/omninav/lib/" + SONAME

#: 契约 §5.2 冻结的「必须导出的符号」。
REQUIRED_SYMBOLS: Tuple[str, ...] = (
    "mecanum_forward",
    "mecanum_inverse",
    "mecanum_l_mm",
    "mecanum_default_geom",
    "mecanum_geom_is_valid",
    "odom_update",
    "odom_reset",
)

#: ``<本包目录>/lib/<SONAME>``（契约 §5.3 第 2 条）。
PACKAGE_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")


class CoreLibraryError(RuntimeError):
    """``libomninav_core.so`` 相关的任何错误 —— 一律往上抛，绝不吞掉。"""


class CoreLibraryNotFound(CoreLibraryError):
    """三个候选路径都不存在。"""


class CoreSymbolMissing(CoreLibraryError):
    """``.so`` 加载成功了，但契约要求的符号没全导出。"""


# --------------------------------------------------------------------------
# ctypes 结构体（字段顺序 / 类型必须与 firmware_core 的头文件逐字对应）
# --------------------------------------------------------------------------


class MecanumGeom(ctypes.Structure):
    """对应 C 的 ``mecanum_geom_t``（``firmware_core/include/mecanum.h``）。"""

    _fields_ = [
        ("half_wheelbase_mm", ctypes.c_int32),
        ("half_track_mm", ctypes.c_int32),
        ("vy_polarity", ctypes.c_int8),
        ("omega_polarity", ctypes.c_int8),
    ]


class MecanumTwist(ctypes.Structure):
    """对应 C 的 ``mecanum_twist_t``。单位 mm/s、mrad/s。"""

    _fields_ = [
        ("vx_mm_s", ctypes.c_int32),
        ("vy_mm_s", ctypes.c_int32),
        ("omega_mrad_s", ctypes.c_int32),
    ]


class MecanumWheels(ctypes.Structure):
    """对应 C 的 ``mecanum_wheels_t``：四轮线速度，顺序 FL, FR, RL, RR。"""

    _fields_ = [("wheel_mm_s", ctypes.c_int32 * WHEEL_COUNT)]


class OdomPose(ctypes.Structure):
    """对应 C 的 ``odom_pose_t``：``{int64_t x_mm, int64_t y_mm, int32_t theta_mrad}``。

    ★ 注意尾部 padding：C 侧按自然对齐算出来是 **24 字节**（4 字节尾填充），
    不是 20。``ctypes.Structure`` 会按同样的 ABI 规则补齐，所以按字段原样声明就是对的；
    但在别处**不要**假设 ``sizeof(OdomPose) == 20``。
    """

    _fields_ = [
        ("x_mm", ctypes.c_int64),
        ("y_mm", ctypes.c_int64),
        ("theta_mrad", ctypes.c_int32),
    ]


class OdomTotal(ctypes.Structure):
    """对应 C 的 ``odom_total_t``：不做角度归一化的累计量。"""

    _fields_ = [
        ("x_mm", ctypes.c_int64),
        ("y_mm", ctypes.c_int64),
        ("theta_mrad", ctypes.c_int64),
    ]


# --------------------------------------------------------------------------
# 库查找
# --------------------------------------------------------------------------


def _normalize_path(raw: str) -> str:
    """展开 ``~`` 与 ``$VAR``，再取绝对路径。

    ★ 契约 §5.3 的错误提示会劝人写 ``OMNINAV_CORE_SO=~/omninav/lib/libomninav_core.so``，
      而 ``os.path.isfile()`` **不认** ``~``（它只是个普通字符）。不展开的话，
      照着提示做反而会拿到"找不到 .so"。
    """
    return os.path.abspath(os.path.expanduser(os.path.expandvars(raw)))


def candidate_paths() -> List[str]:
    """按契约 §5.3 的顺序返回候选路径（第 1 条做 ~ / $VAR 展开）。"""
    paths: List[str] = []
    override = os.environ.get(CORE_SO_ENV, "").strip()
    if override:
        paths.append(_normalize_path(override))
    paths.append(os.path.join(PACKAGE_LIB_DIR, SONAME))
    paths.append(SYSTEM_CORE_SO)
    return paths


def find_library() -> str:
    """返回第一个存在的候选路径；都不存在则抛 :class:`CoreLibraryNotFound`。"""
    tried = candidate_paths()
    for path in tried:
        if os.path.isfile(path):
            return path
    raise CoreLibraryNotFound(
        "找不到 %s。按契约 §5.3 的顺序找过这些路径：\n  %s\n"
        "修法：\n"
        "  * 树莓派：让 pi_setup/install_core.sh 把 .so 装到 %s\n"
        "  * 笔记本 / WSL 开发：%s=/绝对路径/%s\n"
        "  * 或者先在 firmware_core/ 里跑 `make lib` 把 .so 编出来"
        % (SONAME, "\n  ".join(tried), SYSTEM_CORE_SO, CORE_SO_ENV, SONAME)
    )


class Core:
    """``libomninav_core.so`` 的 Python 门面。**不含任何数学**。"""

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = path or find_library()
        try:
            self._lib = ctypes.CDLL(self.path)
        except OSError as exc:
            raise CoreLibraryError(
                "ctypes 加载 %s 失败：%s\n"
                "常见原因：架构不符（x86_64 的 .so 不能给树莓派 aarch64 用；"
                "树莓派上必须在树莓派本地 make lib）" % (self.path, exc)
            ) from exc
        self._bind()

    # -- 符号绑定 ---------------------------------------------------------

    def _bind(self) -> None:
        missing = [name for name in REQUIRED_SYMBOLS if not hasattr(self._lib, name)]
        if missing:
            raise CoreSymbolMissing(
                "%s 缺少契约 §5.2 要求导出的符号：%s\n"
                "（``make lib`` 必须用 -fPIC -shared 编，且不要把符号 static 掉）"
                % (self.path, ", ".join(missing))
            )

        lib = self._lib

        lib.mecanum_default_geom.restype = MecanumGeom
        lib.mecanum_default_geom.argtypes = []

        lib.mecanum_l_mm.restype = ctypes.c_int32
        lib.mecanum_l_mm.argtypes = [ctypes.POINTER(MecanumGeom)]

        lib.mecanum_geom_is_valid.restype = ctypes.c_int
        lib.mecanum_geom_is_valid.argtypes = [ctypes.POINTER(MecanumGeom)]

        lib.mecanum_forward.restype = None
        lib.mecanum_forward.argtypes = [
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(MecanumWheels),
            ctypes.POINTER(MecanumTwist),
        ]

        lib.mecanum_inverse.restype = None
        lib.mecanum_inverse.argtypes = [
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(MecanumTwist),
            ctypes.POINTER(MecanumWheels),
        ]

        lib.odom_reset.restype = None
        lib.odom_reset.argtypes = [ctypes.POINTER(OdomPose), ctypes.POINTER(OdomTotal)]

        lib.odom_update.restype = ctypes.c_int
        lib.odom_update.argtypes = [
            ctypes.POINTER(OdomPose),
            ctypes.POINTER(OdomTotal),
            ctypes.POINTER(MecanumGeom),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(MecanumTwist),
        ]

    # -- 高层封装 ---------------------------------------------------------

    def default_geom(self) -> MecanumGeom:
        """C 侧默认几何（**几何与标定参数的唯一来源**，契约 §6）。"""
        return self._lib.mecanum_default_geom()

    def make_geom(
        self,
        half_wheelbase_mm: int,
        half_track_mm: int,
        vy_polarity: int,
        omega_polarity: int,
    ) -> MecanumGeom:
        geom = MecanumGeom(
            half_wheelbase_mm=int(half_wheelbase_mm),
            half_track_mm=int(half_track_mm),
            vy_polarity=int(vy_polarity),
            omega_polarity=int(omega_polarity),
        )
        if not self.geom_is_valid(geom):
            raise ValueError(
                "几何参数不合法（mecanum_geom_is_valid 返回 0）："
                "half_wheelbase_mm=%d half_track_mm=%d vy_polarity=%d omega_polarity=%d"
                % (half_wheelbase_mm, half_track_mm, vy_polarity, omega_polarity)
            )
        return geom

    def geom_is_valid(self, geom: MecanumGeom) -> bool:
        return bool(self._lib.mecanum_geom_is_valid(ctypes.byref(geom)))

    def l_mm(self, geom: MecanumGeom) -> int:
        """``L = half_wheelbase_mm + half_track_mm``（由 C 算，不在 Python 里再算一遍）。"""
        return int(self._lib.mecanum_l_mm(ctypes.byref(geom)))

    def forward(self, geom: MecanumGeom, wheels_mm_s: Sequence[int]) -> MecanumTwist:
        """正解：四轮线速度（FL,FR,RL,RR）-> 机体速度。"""
        wheels = MecanumWheels((ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(wheels_mm_s)))
        twist = MecanumTwist()
        self._lib.mecanum_forward(ctypes.byref(geom), ctypes.byref(wheels), ctypes.byref(twist))
        return twist

    def inverse(self, geom: MecanumGeom, twist: MecanumTwist) -> MecanumWheels:
        """逆解：机体速度 -> 四轮线速度（FL,FR,RL,RR）。"""
        wheels = MecanumWheels()
        self._lib.mecanum_inverse(ctypes.byref(geom), ctypes.byref(twist), ctypes.byref(wheels))
        return wheels

    def odom_reset(self) -> Tuple[OdomPose, OdomTotal]:
        """清零位姿与累计量，返回一对新的结构体给调用方持有。"""
        pose = OdomPose()
        total = OdomTotal()
        self._lib.odom_reset(ctypes.byref(pose), ctypes.byref(total))
        return pose, total

    def odom_update(
        self,
        pose: OdomPose,
        total: OdomTotal,
        geom: MecanumGeom,
        d_now: Sequence[int],
        d_prev: Sequence[int],
        twist_out: Optional[MecanumTwist] = None,
    ) -> Optional[MecanumTwist]:
        """用四轮**累计里程**增量更新位姿（圆弧积分，在 C 里算）。

        :returns: 更新成功时返回本次的机体速度估计（mm/s、mrad/s）；
                  C 返回 0（几何无效 / 指针为空）时返回 ``None``，**位姿保持不变**。
                  ⇒ 调用方**不要**在这种情况下推进 ``d_prev``，否则会静默丢里程。
        """
        now = (ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(d_now))
        prev = (ctypes.c_int32 * WHEEL_COUNT)(*_check_wheels(d_prev))
        twist = twist_out if twist_out is not None else MecanumTwist()
        ok = self._lib.odom_update(
            ctypes.byref(pose),
            ctypes.byref(total),
            ctypes.byref(geom),
            now,
            prev,
            ctypes.byref(twist),
        )
        return twist if ok else None

    # -- 诊断 -------------------------------------------------------------

    def describe(self) -> str:
        return "%s (OdomPose=%d B, OdomTotal=%d B)" % (
            self.path,
            ctypes.sizeof(OdomPose),
            ctypes.sizeof(OdomTotal),
        )


def _check_wheels(values: Sequence[int]) -> Tuple[int, int, int, int]:
    if len(values) != WHEEL_COUNT:
        raise ValueError("需要 %d 个轮值，拿到 %d 个" % (WHEEL_COUNT, len(values)))
    out = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("wheel[%d] 必须是 int，拿到 %r" % (index, type(value).__name__))
        if not -(2 ** 31) <= value <= 2 ** 31 - 1:
            raise ValueError("wheel[%d]=%d 超出 int32" % (index, value))
        out.append(value)
    return (out[0], out[1], out[2], out[3])


# --------------------------------------------------------------------------
# 模块级默认实例（节点与工具脚本共用一份，避免重复 dlopen）
# --------------------------------------------------------------------------

_DEFAULT_CORE: Optional[Core] = None


def default_core() -> Core:
    """懒加载的默认 :class:`Core`。找不到 ``.so`` 时抛 :class:`CoreLibraryNotFound`。"""
    global _DEFAULT_CORE
    if _DEFAULT_CORE is None:
        _DEFAULT_CORE = Core()
    return _DEFAULT_CORE


def reset_default_core() -> None:
    """丢掉缓存的默认实例（单测 / 换 ``OMNINAV_CORE_SO`` 后用）。"""
    global _DEFAULT_CORE
    _DEFAULT_CORE = None
