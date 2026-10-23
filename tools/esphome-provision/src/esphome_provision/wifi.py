"""Serial port handling and Wi-Fi setup through Improv."""

import os
import socket
import time

from esphome_provision.common import ProvisionError
from esphome_provision.improv import ImprovClient


def wait_for_port(port: str, mac: str, timeout: float) -> str:
    import serial
    from serial.tools import list_ports

    time.sleep(2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(port):
            try:
                serial.Serial(port).close()
                return port
            except (OSError, serial.SerialException):
                pass
        else:
            same = [
                p.device
                for p in list_ports.comports()
                if p.vid == 0x303A
                and (p.serial_number or "").lower().replace("-", ":") == mac.lower()
            ]
            if len(same) == 1:
                print(f"The serial port is now {same[0]}")
                return same[0]
        time.sleep(0.5)
    raise ProvisionError(
        f"The serial port {port} did not come back within {timeout:.0f} s"
    )


def improv_host(port: str, ssid: str, password: str, fallback: str | None) -> str:
    import serial

    link = serial.Serial()
    link.port = port
    link.baudrate = 115200
    link.timeout = 0.2
    link.dtr = False
    link.rts = False
    link.open()
    try:
        client = ImprovClient(link)
        print("Waiting for the Improv serial port")
        client.wait_authorized(90)
        print("Sending the Wi-Fi settings")
        result = client.send_wifi(ssid, password, 60)
        host = result.host()
        if host is None:
            name = client.device_name()
            if name:
                try:
                    host = socket.gethostbyname(f"{name}.local")
                except OSError:
                    host = None
    finally:
        link.close()
    host = host or fallback
    if host is None:
        raise ProvisionError(
            "The device joined Wi-Fi but its address is unknown. Pass --host with the IP address."
        )
    return host
