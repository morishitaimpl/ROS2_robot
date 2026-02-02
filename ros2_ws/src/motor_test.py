#!/usr/bin/env python3
"""
Raspbot V2: 4輪を同時に駆動し、1秒で必ず停止するテスト。

使い方:
  python3 four_wheels_1s.py
  python3 four_wheels_1s.py --speed 120 --seconds 1.0 --direction 0

備考:
- `Raspbot_Lib` が環境に存在する場合はそれを優先して使用します。
- ない場合は `@src/raspbot_driver.py` の `RaspbotV2Driver` を使用します。
- 例外/CTRL+C/kill(SIGTERM)でも停止指令を送るようにフェイルセーフを入れています。
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from typing import Callable, Optional


DEFAULT_MOTOR_IDS = [1, 2, 3, 4]


def _parse_ids_csv(spec: str) -> list[int]:
    return [int(x.strip()) for x in spec.split(",") if x.strip()]


def _load_driver():
    # 1) ユーザー環境にある可能性が高いライブラリ
    try:
        from Raspbot_Lib import Raspbot  # type: ignore

        rb = Raspbot()
        return rb, "Raspbot_Lib.Raspbot"
    except Exception:
        pass

    # 2) このワークスペースの簡易ドライバ
    from raspbot_driver import RaspbotV2Driver

    rb = RaspbotV2Driver()
    return rb, "src/raspbot_driver.py:RaspbotV2Driver"


def _make_stop_fn(rb) -> Callable[[], None]:
    """
    どのドライバでも止められる stop 関数を作る。
    """

    def stop_all():
        # motor_dir は停止時は0固定で送る（仕様不明でも安全側）
        for mid in DEFAULT_MOTOR_IDS:
            try:
                rb.Ctrl_Car(mid, 0, 0)
            except Exception:
                # 停止処理中に1個失敗しても他を止める
                pass

    return stop_all


def _maybe_close(rb) -> None:
    close = getattr(rb, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            pass


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=int, default=120, help="モーター速度 (例: 0-255)")
    parser.add_argument("--direction", type=int, default=0, help="モーター方向 (環境依存。例: 0=前進)")
    parser.add_argument("--seconds", type=float, default=1.0, help="駆動時間(秒)。1.0で1秒停止")
    parser.add_argument(
        "--motor-ids",
        default="1,2,3,4",
        help="同時に回すmotor_idのCSV。例: 1,2,3,4 / 1,2,3,5",
    )
    parser.add_argument(
        "--keepalive-hz",
        type=float,
        default=0.0,
        help="駆動中に指令を再送する周波数(Hz)。0で再送なし",
    )
    args = parser.parse_args(argv)

    rb, drv_name = _load_driver()
    motor_ids = _parse_ids_csv(args.motor_ids)

    def stop_all():
        for mid in motor_ids:
            try:
                rb.Ctrl_Car(mid, 0, 0)
            except Exception:
                pass

    def cleanup():
        stop_all()
        _maybe_close(rb)

    # 例外終了でも止める
    atexit.register(cleanup)

    # SIGINT/SIGTERMでも止める（docker/ros2起動中の停止も安全側）
    def _signal_handler(signum, frame):
        cleanup()
        raise SystemExit(0)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _signal_handler)
        except Exception:
            pass

    print(f"driver={drv_name}")
    print(f"start: motors={motor_ids}, direction={args.direction}, speed={args.speed}, seconds={args.seconds}")

    # 4輪同時に開始
    for mid in motor_ids:
        rb.Ctrl_Car(mid, args.direction, args.speed)

    # 指定秒だけ回す（monotonicで堅牢に）
    t0 = time.monotonic()
    period = (1.0 / args.keepalive_hz) if args.keepalive_hz and args.keepalive_hz > 0 else None
    next_send = time.monotonic() + (period or 10**9)
    while True:
        if time.monotonic() - t0 >= args.seconds:
            break
        if period is not None and time.monotonic() >= next_send:
            # keepalive: 一部の基板で「最後の指令が保持されない」ケースの切り分け用
            for mid in motor_ids:
                try:
                    rb.Ctrl_Car(mid, args.direction, args.speed)
                except Exception:
                    pass
            next_send = time.monotonic() + period
        time.sleep(0.01)

    # 1秒で必ず停止
    stop_all()
    print("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

