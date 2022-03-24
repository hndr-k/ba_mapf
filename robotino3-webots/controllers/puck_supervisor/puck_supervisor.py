"""puck_supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Field, Robot, DistanceSensor, Node

# create the Robot instance.
supervisor = Supervisor()
puck1 = supervisor.getFromDef("Puck")
puck1_ring_node = supervisor.getFromDef("Ring1_color")
puck1_ring2_node = supervisor.getFromDef("Ring2_color")
puck1_ring3_node = supervisor.getFromDef("Ring3_color")

laser = supervisor.getDeviceByIndex(0)
laser.enable(1)
color = puck1_ring_node.getField("baseColor")

transparency = puck1_ring_node.getField("transparency")
transparency2 = puck1_ring2_node.getField("transparency")
transparency3 = puck1_ring3_node.getField("transparency")

print(color.getSFColor())
print(transparency.getSFFloat())

color.setSFColor([1.0, 0.161, 0.161])
transparency.setSFFloat(1)
transparency2.setSFFloat(1)
transparency3.setSFFloat(1)
timestep = int(supervisor.getBasicTimeStep())
while supervisor.step(timestep) != -1:
    if laser.getValue() <= 0.29:
        transparency.setSFFloat(0)
    pass

# Enter here exit cleanup code.
