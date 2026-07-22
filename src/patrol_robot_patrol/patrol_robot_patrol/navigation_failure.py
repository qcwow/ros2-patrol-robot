"""Classify Nav2 action failures without importing the ROS runtime."""


_ERROR_LABELS = {
    0: '无错误码',
    100: '控制器未知错误',
    101: '控制器插件无效',
    102: '控制器 TF 错误',
    103: '控制器收到无效路径',
    104: '控制器容错时间耗尽',
    105: '车辆未取得进展',
    106: '控制器找不到有效控制指令',
    107: '控制器循环超时',
    200: '规划器未知错误',
    201: '规划器插件无效',
    202: '规划器 TF 错误',
    203: '起点位于地图外',
    204: '终点位于地图外',
    205: '起点被占用',
    206: '终点被占用',
    207: '全局规划超时',
    208: '不存在有效全局路径',
}

# These failures say that the current geometric path or its endpoint cannot be
# executed. They should exclude that candidate, not retry it more slowly.
_ROUTE_FAILURE_CODES = {103, 104, 105, 106, 203, 204, 205, 206, 208}


def navigation_error_label(error_code: int | None) -> str:
    if error_code is None:
        return '未提供错误码'
    return _ERROR_LABELS.get(error_code, f'未知错误码 {error_code}')


def is_route_failure(error_code: int | None) -> bool:
    """Return true only for deterministic route/control feasibility errors."""

    return error_code in _ROUTE_FAILURE_CODES
