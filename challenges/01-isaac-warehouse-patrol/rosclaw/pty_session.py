"""PTY utility adapted from ROSClaw upstream test_product_journey (MIT). No model stub is used."""

import contextlib
import base64
import json
import fcntl
import os
import subprocess
import termios
import threading
import time
from pathlib import Path

_ANSI_RE = None


def _strip_ansi(data: bytes) -> bytes:
    """去 ANSI 转义/控制序列（PTY 文本断言用——换行重绘会把控制码
    插进文本中间）。"""
    global _ANSI_RE
    if _ANSI_RE is None:
        import re as _re

        # 只去 CSI/字符集序列——OSC（窗口标题 \x1b]0;...\x07）保留：
        # 旅程的品牌标题断言匹配的就是 OSC 内容。
        _ANSI_RE = _re.compile(rb"\x1b\[[0-9;?]*[a-zA-Z]|\x1b[()][0-9A-B]")
    return _ANSI_RE.sub(b"", data)


class PtySession:
    """最小 PTY 驱动：expect/send。"""

    def __init__(
        self,
        argv: list[str],
        env: dict[str, str],
        log_path: Path | None = None,
        cwd: Path | None = None,
    ) -> None:
        import pty as _pty

        if env.get("FAKE_JOURNEY_KEY"):
            # Loopback fake-model servers must be contacted directly.
            env = dict(env)
            for proxy in (
                "HTTP_PROXY",
                "HTTPS_PROXY",
                "ALL_PROXY",
                "http_proxy",
                "https_proxy",
                "all_proxy",
            ):
                env.pop(proxy, None)
            env["NO_PROXY"] = "127.0.0.1,localhost,::1"

        self.master, slave = _pty.openpty()
        # CI 失败诊断（五审 Gate Evidence）：PTY 全量输出落盘——超时
        # 断言只有尾部 3000 字节，完整输出是定位 CI-only 失败的唯一证据。
        self._log = log_path.open("wb") if log_path else None
        self._events = (
            log_path.with_suffix(".events.jsonl").open("w") if log_path else None
        )

        def _make_controlling_tty() -> None:
            # 新 session + 控制终端——否则 TIOCSWINSZ 的 SIGWINCH 没有
            # 前台进程组可投递，TUI 永远收不到 resize 事件。
            os.setsid()
            fcntl.ioctl(slave, termios.TIOCSCTTY, 0)

        self.proc = subprocess.Popen(
            argv,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            env=env,
            close_fds=True,
            preexec_fn=_make_controlling_tty,
            cwd=str(cwd) if cwd else None,
        )
        os.close(slave)
        self.output = b""
        # 六审 §7：长行在窄 PTY 换行后 ANSI 控制码会把文本切碎——
        # expect 一律匹配去 ANSI 的缓冲（raw output 保留给日志/诊断）。
        self.clean = b""
        self.last_at = time.monotonic()
        # 后台持续 drain——PTY 缓冲满会阻塞子进程写（没有它测试会假死）。
        self._lock = threading.Lock()
        self._draining = True

        def _drain() -> None:
            import select as _select

            while self._draining:
                try:
                    ready, _, _ = _select.select([self.master], [], [], 0.2)
                    if ready:
                        chunk = os.read(self.master, 4096)
                        if not chunk:
                            break
                        with self._lock:
                            self.output += chunk
                            self.clean += _strip_ansi(chunk)
                            self.last_at = time.monotonic()
                        if self._log is not None:
                            with contextlib.suppress(OSError):
                                self._log.write(chunk)
                                self._log.flush()
                                self._events.write(
                                    json.dumps(
                                        {
                                            "wall_time": time.time(),
                                            "data": base64.b64encode(chunk).decode(),
                                        }
                                    )
                                    + "\n"
                                )
                                self._events.flush()
                except OSError:
                    break

        self._drain_thread = threading.Thread(target=_drain, daemon=True)
        self._drain_thread.start()

    def expect(self, marker: bytes, timeout: float = 60.0, *, after: int = 0) -> bytes:
        # 七审 PR-SEVEN-7：after 偏移——同一标记（如授权卡）在一次
        # 会话里出现多次时，只匹配 after 之后的新内容。
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if marker in self.clean[after:]:
                    return self.clean
            if self.proc.poll() is not None:
                with self._lock:
                    if marker in self.clean[after:]:
                        return self.clean
                break
            time.sleep(0.1)
        with self._lock:
            tail = self.output[-3000:]
        raise AssertionError(f"PTY 超时未等到 {marker!r}；已收输出尾部: {tail!r}")

    def expect_with_resend(
        self,
        marker: bytes,
        payload: str,
        timeout: float = 60.0,
        *,
        after: int = 0,
        interval: float = 2.0,
    ) -> bytes:
        """发送 payload 直到 marker 出现（七审 PR-SEVEN-7：overlay 聚焦/
        输入时机竞态的确定性重试——授权卡 decided 后 handleInput 忽略
        后续按键，/quit 重复发送无害）。"""
        deadline = time.monotonic() + timeout
        last_sent = 0.0
        while time.monotonic() < deadline:
            with self._lock:
                if marker in self.clean[after:]:
                    return self.clean
            if self.proc.poll() is not None:
                with self._lock:
                    if marker in self.clean[after:]:
                        return self.clean
                break
            now = time.monotonic()
            if now - last_sent >= interval:
                self.send(payload)
                last_sent = now
            time.sleep(0.1)
        with self._lock:
            tail = self.output[-3000:]
        raise AssertionError(f"PTY 超时未等到 {marker!r}；已收输出尾部: {tail!r}")

    def send(self, text: str) -> None:
        os.write(self.master, text.encode())

    def stop(self) -> None:
        self._draining = False
        with contextlib.suppress(OSError):
            os.close(self.master)
        if self._log is not None:
            with contextlib.suppress(OSError):
                self._log.close()
            self._log = None
        if self._events is not None:
            self._events.close()
            self._events = None
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
