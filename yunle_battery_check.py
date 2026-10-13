import sys, os

# Apollo cyber paths
_apollo_neo = "/opt/apollo/neo"
_apollo_cyber = "/opt/apollo/cyber"

for p in [
    os.path.join(_apollo_neo, "lib/python3.8/site-packages"),
    os.path.join(_apollo_neo, "lib/python3.8"),
    os.path.join(_apollo_cyber, "python"),
    _apollo_cyber,
    os.path.join(_apollo_cyber, "lib"),
]:
    if p not in sys.path and os.path.isdir(p):
        sys.path.insert(0, p)

os.environ.setdefault("CYBER_PATH", _apollo_cyber)
os.environ.setdefault("LD_LIBRARY_PATH", "")
if "/opt/apollo/neo/lib" not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = "/opt/apollo/neo/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
if "/opt/apollo/cyber/lib" not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] += ":/opt/apollo/cyber/lib"

try:
    from cyber.python.cyber_py3 import cyber
    print("cyber imported")
except Exception as e:
    print("cyber import failed:", e)
    # try alternate
    try:
        import _cyber_wrapper
        print("_cyber_wrapper imported")
    except:
        pass
    sys.exit(1)

from modules.canbus_vehicle.yunle.proto import yunle_chassis_receiver_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
import time, threading

detail_lock = threading.Lock()
chassis_lock = threading.Lock()
detail_latest = None
chassis_latest = None

def detail_cb(msg):
    global detail_latest
    with detail_lock:
        detail_latest = yunle_chassis_receiver_pb2.YunleChassisDetail()
        detail_latest.CopyFrom(msg)

def chassis_cb(msg):
    global chassis_latest
    with chassis_lock:
        chassis_latest = chassis_pb2.Chassis()
        chassis_latest.CopyFrom(msg)

cyber.init("yunle_battery_check")
node = cyber.Node("yunle_battery_check")
node.create_reader("/apollo/canbus/yunle_chassis_detail",
                   yunle_chassis_receiver_pb2.YunleChassisDetail, detail_cb)
node.create_reader("/apollo/canbus/chassis",
                   chassis_pb2.Chassis, chassis_cb)
time.sleep(3.0)

detail = None
chassis = None
with detail_lock:
    if detail_latest is not None:
        detail = yunle_chassis_receiver_pb2.YunleChassisDetail()
        detail.CopyFrom(detail_latest)
with chassis_lock:
    if chassis_latest is not None:
        chassis = chassis_pb2.Chassis()
        chassis.CopyFrom(chassis_latest)

if detail is not None:
    print("communication_ok={}".format(detail.communication_ok))
    print("battery={:.1f}V soc={}%".format(
        detail.battery_voltage_v if detail.HasField("battery_voltage_v") else -1,
        detail.battery_soc_percentage if detail.HasField("battery_soc_percentage") else -1))
    print("vehicle_speed_kph={:.3f}".format(detail.vehicle_speed_kph))
else:
    print("No detail message received")

if chassis is not None:
    print("chassis_speed_kph={:.3f}".format(chassis.speed_mps * 3.6))
else:
    print("No chassis message received")

print("done")
