"""프로젝트 공통 예외 (명세서 §31 오류 코드와 연동)."""
from .enums import ErrorCode


class RcCarError(Exception):
    """모든 내부 예외의 뿌리. code는 로그·디버깅용 — report:error에는 싣지 않는다."""

    def __init__(self, code: ErrorCode, message: str = ""):
        self.code = code
        super().__init__(f"[{code.value}] {message}")


class ConfigError(RcCarError):
    def __init__(self, message: str):
        super().__init__(ErrorCode.CONFIG_INVALID, message)


class DriverError(RcCarError):
    pass


class MapError(RcCarError):
    """차선 그래프 맵 로드·검증 실패 (단계 5). 기동 시 발견 = 즉시 거부."""

    def __init__(self, message: str):
        super().__init__(ErrorCode.CONFIG_INVALID, message)
