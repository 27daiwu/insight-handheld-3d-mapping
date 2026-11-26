import socket, struct, select, time, sys
IFADDR = "169.254.10.2"
ports = {7400 + 250*d: d for d in range(0, 233)}
socks = {}
for p, d in ports.items():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except Exception: pass
    try:
        s.bind(("", p))
        for grp in ("239.255.0.1",):
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(grp) + socket.inet_aton(IFADDR))
        socks[s] = (p, d)
    except OSError as e:
        pass
print("listening on", len(socks), "domain ports for", sys.argv[1], "s")
seen = {}
end = time.time() + float(sys.argv[1])
while time.time() < end:
    r, _, _ = select.select(list(socks), [], [], 0.5)
    for s in r:
        data, addr = s.recvfrom(65535)
        p, d = socks[s]
        key = (d, addr[0], data[:4] == b"RTPS", data[4:6] if data[:4]==b"RTPS" else b"", data[8:20].hex() if data[:4]==b"RTPS" else "")
        seen[key] = seen.get(key, 0) + 1
for k, n in sorted(seen.items()):
    print(f"domain={k[0]} src={k[1]} rtps={k[2]} ver={k[3].hex()} guidprefix={k[4]} packets={n}")
if not seen: print("no RTPS multicast traffic seen")
