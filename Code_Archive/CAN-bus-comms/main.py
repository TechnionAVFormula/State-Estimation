# import the library
import can

bus1 = can.interface.Bus("test", bustype="virtual")
bus2 = can.interface.Bus("test", bustype="virtual")

msg1 = can.Message(arbitration_id=0xABCDE, data=[1, 2, 3])
bus1.send(msg1)
print(msg1)
msg2 = bus2.recv()
print(msg2)
assert msg1.data == msg2.data
