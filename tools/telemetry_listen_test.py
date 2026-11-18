"""Test listener for SR2_TELEMETRY: records the game's UDP datagrams to two
files next to the output path given:
    <out>.ndjson  one JSON line per frame (format "json" / "both")
    <out>.forza   raw 324-byte packets concatenated, nothing else ("forza"/"both")
so parsers can read them without guessing at framing.

Usage: python tools/telemetry_listen_test.py <port> <max_packets> <out path>
Stops at max_packets (counting both formats) or 30 s after the last datagram.
Prints one line of summary at the end.
"""
import socket
import sys

port, max_n, base = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", port))
s.settimeout(30)
json_n = forza_n = 0
fj = open(base + ".ndjson", "wb")
ff = open(base + ".forza", "wb")
try:
    while json_n + forza_n < max_n:
        data, addr = s.recvfrom(2048)
        if len(data) == 324:
            forza_n += 1
            ff.write(data)
        elif data.startswith(b"{"):
            json_n += 1
            fj.write(data)
except socket.timeout:
    print(f"timeout after {json_n} json, {forza_n} forza")
fj.close()
ff.close()
print(f"{json_n} json -> {base}.ndjson, {forza_n} forza -> {base}.forza")
