import time, sys
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant, BuiltinTopicDcpsTopic
dp = DomainParticipant(9)
rp = BuiltinDataReader(dp, BuiltinTopicDcpsParticipant)
rt = BuiltinDataReader(dp, BuiltinTopicDcpsTopic)
end=time.time()+float(sys.argv[1]); parts={}; topics={}
while time.time()<end:
    for s in rp.take(N=50): parts[str(s.key)]=s.qos
    for s in rt.take(N=100): topics[s.topic_name]=s.type_name
    time.sleep(0.3)
print("participants:",len(parts)); 
for t,v in sorted(topics.items()): print(t,v)
