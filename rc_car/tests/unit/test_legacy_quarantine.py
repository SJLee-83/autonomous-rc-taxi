"""🔴 말뚝 테스트 — 가상 라인 트레이싱(원호 추종)의 재유입 차단 (2026-08-06 사용자 지시).

배경: `app.yaml driver_mode: mock` 이 PC 정본 재배포로 되살아나 실차에서 모터 무반응을
만든 사고가 **3번** 있었다(§0-43 "config 원복 패턴"). 사용자 지시는 명확하다 —
목·금 실주행에서 폐기한 로직이 "예전 거 나온 듯" 하고 튀어나오면 안 된다.

그래서 이 파일이 강제하는 것:
    ① 기본값은 항상 새 로직(LaneFollower)이다
    ② 폐기 로직은 **환경변수 RC_CAR_LEGACY_ARC=1 로만** 켜진다
    ③ **config/*.yaml 어디에도 켜는 스위치가 없다** — 재배포로 config가 덮여도 안 켜진다
    ④ 옛 경로(control/waypoint_follower.py)에 파일이 되살아나면 여기서 걸린다
    ⑤ 평상시 import 그래프에 폐기 모듈이 끌려 들어오지 않는다

이 파일을 지우지 않는 한 재유입은 CI에서 걸린다.
"""
import os
import subprocess
import sys
import unittest
from pathlib import Path

from control.legacy import LEGACY_ENV, legacy_arc_enabled

RC_CAR = Path(__file__).resolve().parents[2]
CONFIG_DIR = RC_CAR / "config"


class TestLegacyGate(unittest.TestCase):
    """② 환경변수 단독 판정 — config 를 읽지 않는다."""

    def setUp(self):
        self._saved = os.environ.get(LEGACY_ENV)
        os.environ.pop(LEGACY_ENV, None)

    def tearDown(self):
        os.environ.pop(LEGACY_ENV, None)
        if self._saved is not None:
            os.environ[LEGACY_ENV] = self._saved

    def test_default_is_disabled(self):
        self.assertFalse(legacy_arc_enabled(), "환경변수 없으면 폐기 로직은 꺼져 있어야 한다")

    def test_only_exact_one_enables(self):
        for value, expected in [("1", True), (" 1 ", True), ("0", False), ("", False),
                                ("true", False), ("yes", False), ("legacy", False)]:
            with self.subTest(value=value):
                os.environ[LEGACY_ENV] = value
                self.assertEqual(legacy_arc_enabled(), expected)

    def test_env_name_is_not_generic(self):
        """이름이 흔하면 다른 도구가 우연히 켤 수 있다."""
        self.assertEqual(LEGACY_ENV, "RC_CAR_LEGACY_ARC")


class TestNoConfigSwitch(unittest.TestCase):
    """③ config 로는 켤 수 없다 — 재배포가 config 를 덮어써도 안전하다."""

    def test_no_yaml_mentions_the_switch(self):
        hits = []
        for path in sorted(CONFIG_DIR.glob("*.yaml")):
            text = path.read_text(encoding="utf-8")
            for needle in (LEGACY_ENV, "legacy", "waypoint_follower"):
                if needle.lower() in text.lower():
                    hits.append(f"{path.name}: '{needle}'")
        self.assertEqual(hits, [], "config/*.yaml 에 폐기 로직 스위치가 생겼다 — "
                                   "환경변수 단독 규칙 위반 (재배포로 켜질 수 있다)")

    def test_runtime_gates_on_env_only(self):
        """runtime 은 legacy_arc_enabled() 로만 분기해야 한다."""
        src = (RC_CAR / "app" / "runtime.py").read_text(encoding="utf-8")
        self.assertIn("legacy_arc_enabled()", src)
        self.assertIn("from control.lane_follower import LaneFollower", src)
        # 폐기 모듈은 게이트 안에서 지연 import 되어야 한다 (최상단 import 금지)
        for line in src.splitlines():
            stripped = line.strip()
            if stripped.startswith(("import ", "from ")) and "legacy.waypoint_follower" in stripped:
                self.assertTrue(line.startswith("    "),
                                "폐기 모듈이 최상단에서 import 되고 있다 — 게이트 안으로 옮길 것")


class TestOldPathStaysEmpty(unittest.TestCase):
    """④ 옛 경로 부활 감지 — 스테일 배포가 파일을 되돌려 놓으면 걸린다."""

    def test_old_module_path_absent(self):
        old = RC_CAR / "control" / "waypoint_follower.py"
        self.assertFalse(old.exists(),
                         f"{old} 가 되살아났다. 폐기 로직의 정본은 "
                         "control/legacy/waypoint_follower.py 하나뿐이어야 한다 "
                         "(스테일 배포·머지 사고 의심)")

    def test_new_default_module_exists(self):
        self.assertTrue((RC_CAR / "control" / "lane_follower.py").exists())
        self.assertTrue((RC_CAR / "control" / "legacy" / "waypoint_follower.py").exists(),
                        "폐기 로직은 지우지 않고 격리 보관한다 — 시연 당일 최후 폴백")

    def test_legacy_module_is_marked(self):
        """구석에 박아둔 표식이 지워지면 다음 사람이 기본 로직으로 오인한다."""
        for name in ("__init__.py", "waypoint_follower.py"):
            text = (RC_CAR / "control" / "legacy" / name).read_text(encoding="utf-8")
            self.assertIn("🔴", text)
            self.assertIn(LEGACY_ENV, text)


class TestNotImportedByDefault(unittest.TestCase):
    """⑤ 평상시 import 그래프에 폐기 모듈이 없다 — '조용히 켜짐'의 마지막 구멍 차단."""

    def test_importing_runtime_does_not_pull_legacy_follower(self):
        code = (
            "import sys; import app.runtime\n"
            "assert 'control.lane_follower' in sys.modules, 'new follower missing'\n"
            "assert 'control.legacy.waypoint_follower' not in sys.modules, "
            "'legacy arc follower was imported without the env var'\n"
            "print('ok')\n")
        env = dict(os.environ)
        env.pop(LEGACY_ENV, None)
        proc = subprocess.run([sys.executable, "-c", code], cwd=str(RC_CAR),
                              capture_output=True, text=True, env=env)
        self.assertEqual(proc.returncode, 0, proc.stderr)


if __name__ == "__main__":
    unittest.main()
