import sys, time
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
dp = DomainParticipant(9)
rd = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
seen = {}; end = time.time() + float(sys.argv[1])
while time.time() < end:
    for s in rd.take(N=100):
        seen[s.topic_name] = s
    time.sleep(0.3)
for n, s in sorted(seen.items()):
    q = s.qos; print(n, "| QOS:", q)
if not seen: print("no publications discovered")
