"""무선 통신 블록 `link`.

지상국이 드론에 보내는 명령은 localhost의 실제 UDP 소켓을 거친다. 지연과 손실은
설정으로 바꿀 수 있다. 명령 종류는 Crazyflie(cflib, Crazyswarm2)와 같다.

Radio link block. Ground-station commands go to the drones over a real UDP
socket on localhost, with configurable delay and loss. The command set is the
same as Crazyflie's (cflib, Crazyswarm2).

    takeoff(h, dur), go_to(p, yaw, dur), cmd_position(p, yaw), land(dur), emergency_stop()

    link = UdpLink(names, clock=lambda: world.t, loss=0.02, seed=0)
    cf = link["cf1"]; cf.takeoff(1.0, 2.0); cf.cmd_position([0, 0, 1], 0.0)
    radio = DroneRadio("cf1", link.port_of("cf1"), clock=..., delay=0.01)   # 드론 칩 쪽
    for pkt in radio.poll(): ...

패킷 형식 (little-endian, 31바이트):
    <B type> <H seq> <d stamp> <5f payload>
"""
from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

import numpy as np

TAKEOFF, GOTO, POSITION, LAND, STOP = 1, 2, 3, 4, 5
TYPE_NAME = {TAKEOFF: "takeoff", GOTO: "go_to", POSITION: "cmd_position", LAND: "land", STOP: "emergency_stop"}
_FMT = "<BHd5f"
PKT_SIZE = struct.calcsize(_FMT)
BASE_PORT = 47100


@dataclass
class Packet:
    type: int
    seq: int
    stamp: float
    data: tuple

    @property
    def name(self):
        return TYPE_NAME.get(self.type, "?")

    def encode(self) -> bytes:
        d = list(self.data) + [0.0] * (5 - len(self.data))
        return struct.pack(_FMT, self.type, self.seq & 0xFFFF, self.stamp, *d)

    @staticmethod
    def decode(b: bytes) -> "Packet":
        t, seq, stamp, *d = struct.unpack(_FMT, b[:PKT_SIZE])
        return Packet(t, seq, stamp, tuple(d))


class CrazyflieHandle:
    """드론 한 대에 명령을 보내는 손잡이. 함수 이름은 Crazyflie와 같다.

    Sends commands to one drone. Method names match Crazyflie's.
    """

    def __init__(self, link: "UdpLink", name: str):
        self.link, self.name = link, name

    def takeoff(self, height: float, duration: float):
        self.link._send(self.name, TAKEOFF, (height, duration), reliable=True)

    def go_to(self, p, yaw: float, duration: float):
        self.link._send(self.name, GOTO, (p[0], p[1], p[2], yaw, duration), reliable=True)

    def cmd_position(self, p, yaw: float = 0.0):
        """스트리밍 setpoint. 50 Hz로 계속 보내야 한다.

        Streaming setpoint; keep sending it at 50 Hz.
        """
        self.link._send(self.name, POSITION, (p[0], p[1], p[2], yaw), reliable=False)

    def land(self, duration: float):
        self.link._send(self.name, LAND, (duration,), reliable=True)

    def emergency_stop(self):
        self.link._send(self.name, STOP, (), reliable=True, copies=5)


class UdpLink:
    """지상국 쪽 무선. clock은 지금 시각을 돌려주는 함수(시뮬 시각 또는 벽시계)이고,
    패킷 손실은 보내는 쪽에서 loss 확률로 일으킨다. 드론 i는 port_base + i 포트에서 받는다.

    Ground-station side of the radio.

    Args:
        names: drone names
        clock: function returning the current time (sim time or wall clock)
        loss: packet loss probability, applied at the sender
        port_base: drone i listens on port_base + i
    """

    def __init__(self, names, clock=time.monotonic, loss=0.0, seed=0, port_base=BASE_PORT, host="127.0.0.1",
                 ports=None):
        self.names = list(names)
        self.ports = dict(ports) if ports else {n: port_base + i for i, n in enumerate(self.names)}
        self.clock = clock
        self.loss = float(loss)
        self.rng = np.random.default_rng(seed + 1000)
        self.port_base = port_base
        self.host = host
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0
        self.sent = 0
        self.dropped = 0
        self._handles = {n: CrazyflieHandle(self, n) for n in self.names}

    def __getitem__(self, name) -> CrazyflieHandle:
        return self._handles[name]

    def port_of(self, name) -> int:
        return self.ports[name]

    def _send(self, name, typ, data, reliable=False, copies=3):
        self._seq += 1
        pkt = Packet(typ, self._seq, float(self.clock()), tuple(float(x) for x in data)).encode()
        # 중요한 명령은 여러 번 보낸다. 받는 쪽에서 seq로 중복을 걸러 낸다.
        for _ in range(copies if reliable else 1):
            self.sent += 1
            if self.rng.random() < self.loss:
                self.dropped += 1
                continue
            self.sock.sendto(pkt, (self.host, self.port_of(name)))

    def close(self):
        self.sock.close()


class DroneRadio:
    """드론 칩 쪽 무선 수신기 (시뮬).

    지연은 받는 쪽에서 넣는다. 패킷을 `stamp + delay` 시각까지 붙잡아 두므로
    시뮬 시각 기준으로 결과가 재현된다.

    Drone-side receiver (sim). Latency is applied here: each packet is held
    until stamp + delay, so runs stay reproducible in sim time.
    """

    def __init__(self, name, port=0, clock=time.monotonic, delay=0.0, host="127.0.0.1"):
        self.name, self.clock, self.delay = name, clock, float(delay)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((host, port))           # port=0이면 OS가 빈 포트를 골라 준다
        self.port = self.sock.getsockname()[1]
        self.sock.setblocking(False)
        self._pending = []
        self._seen = set()
        self.received = 0

    def poll(self):
        """지금까지 도착한 패킷 목록.

        Packets that have arrived by now.
        """
        while True:
            try:
                b = self.sock.recv(64)
            except (BlockingIOError, InterruptedError):
                break
            pkt = Packet.decode(b)
            if pkt.type != POSITION:          # 여러 번 보낸 명령은 한 번만 처리
                if pkt.seq in self._seen:
                    continue
                self._seen.add(pkt.seq)
                if len(self._seen) > 4096:
                    self._seen = set(list(self._seen)[-1024:])
            self._pending.append(pkt)
        now = self.clock()
        out = [p for p in self._pending if p.stamp + self.delay <= now + 1e-9]
        if out:
            self._pending = [p for p in self._pending if p.stamp + self.delay > now + 1e-9]
            self.received += len(out)
        return out

    def close(self):
        self.sock.close()


class EspUdpLink(UdpLink):
    """실기체(ESP32-S3 드론)용 자리. 펌웨어가 정해지면 채운다.

    - ESP-Drone 기본 펌웨어는 CRTP 패킷 끝에 checksum을 붙여서 표준 cflib과 형식이 다르다.
    - LiteWing 계열은 표준 cflib으로 동작한다는 사례가 있다. 직접 확인해 봐야 한다.
    - `_send`만 CRTP(high-level commander, position setpoint) 인코딩으로 바꾸면
      지상국 코드는 고칠 것이 없다.

    Placeholder for the real ESP32-S3 drones, to be filled in once the
    firmware is chosen. Only `_send` needs a CRTP encoder; the ground-station
    code stays the same.
    """

    def __init__(self, names, addresses: dict, **kw):
        super().__init__(names, **kw)
        self.addresses = addresses   # 이름 -> (ip, port)

    def _send(self, name, typ, data, reliable=False, copies=3):
        raise NotImplementedError("EspUdpLink: CRTP 인코딩은 펌웨어가 정해진 뒤에 구현한다")
