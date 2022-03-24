"""mps_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import asyncio
import time
import copy
import logging
from asyncua import ua, uamethod, Server
from datetime import datetime
from math import sin
import sys
from opcua.server.user_manager import UserManager
from asyncua.ua.uatypes import StringNodeId
from controller import Robot, DistanceSensor, Keyboard, Supervisor, Field
import _thread
import rclpy
import unittest
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Bool

@uamethod
def func(parent, value):
    return value * 2

def place_ring(robot):
    puck1 = robot.getFromDef("Puck")
    puck1_ring_node = robot.getFromDef("Ring1_color")
    puck1_ring2_node = robot.getFromDef("Ring2_color")
    puck1_ring3_node = robot.getFromDef("Ring3_color")

    transparency = puck1_ring_node.getField("transparency")
    transparency2 = puck1_ring2_node.getField("transparency")
    transparency3 = puck1_ring3_node.getField("transparency")
    print(transparency.getSFFloat())
    if transparency.getSFFloat() == 1:
        transparency.setSFFloat(0)
    elif transparency2.getSFFloat() == 1:
        transparency2.setSFFloat(0)
    elif transparency3.getSFFloat() == 1:
        transparency3.setSFFloat(0)

def start_belt(robot, con_belt):
    laser = robot.getDeviceByIndex(1)
    laser_mid = robot.getDeviceByIndex(0)
    laser.enable(1)
    laser_mid.enable(1)
    puck_detected = False 
    con_belt.setVelocity(-0.025)

    while robot.step(int(robot.getBasicTimeStep())) != -1:
        if laser_mid.getValue() >= 0.4 and not puck_detected:
            pass
        else:
            print("Place Ring")
            place_ring(robot)
            break

    while robot.step(int(robot.getBasicTimeStep())) != -1:
        if laser.getValue() >= 0.4 and not puck_detected:
            pass
        else:
            print("BREAK ")
            puck_handle(robot, laser, con_belt)
            break

def puck_handle(robot, laser, con_belt):
    max_range = 0.4
    print("Puck Handle")
    while robot.step(int(robot.getBasicTimeStep())) != -1 and laser.getValue() < max_range:
        pass
    con_belt.setVelocity(0.025)
    while robot.step(int(robot.getBasicTimeStep())) != -1 and laser.getValue() == max_range:
        pass
        print(robot.step(int(robot.getBasicTimeStep())))

    print("Puck at End")
    con_belt.setVelocity(0.0)
    belt_status = False

belt_status = False

def run_sim(robot):
    print("run sim")
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        robot.step(32)

async def main():
    server = Server()
    server.set_endpoint('opc.tcp://127.0.0.1:4840/')
    server.set_server_name("MPS Server")
    await server.init()
    server.private_key = None
    server.certificate = None
    server.user_manager = UserManager(parent=server)
    server._security_policy = [ua.SecurityPolicyType.NoSecurity]
    server._policyIDs = ["Anonymous"]
    # setup our own namespace, not really necessary but should as spec
    uri = 'http://examples.freeopcua.github.io'

    idx_uri = await server.register_namespace(uri)
    idx_uri_sec = await server.register_namespace('dummy')
    idx = await server.register_namespace('dummyy')
    print("######## IDX"+str(idx))

    myserver = await server.nodes.objects.add_object(f"ns={idx}; s=Server", "Server")
    await myserver.set_modelling_rule(True)

    dev = await myserver.add_object_type(f"ns={idx}; s=|var|CPX-E-CEC-C1-PN", "CPX-E-CEC-C1-PN")
    await dev.set_modelling_rule(True)

    res = await dev.add_object(idx, "Resource")
    await res.set_modelling_rule(True)

    app = await res.add_object(idx, "Application")
    await app.set_modelling_rule(True)

    global_vars = await app.add_object(idx, "GlobalVars")
    await global_vars.set_modelling_rule(True)

    folder_G = await global_vars.add_folder(idx, "G")
    await folder_G.set_modelling_rule(True)

    myvar = await myserver.add_variable(idx, 'MyVariable', 6.7)

    in_var = await folder_G.add_variable(idx, 'In', 6.7)
    basic_var = await folder_G.add_variable(idx, 'Basic', 6.7)
    #####BASIC
    basic_p_var = await basic_var.add_variable(idx, 'p', 6.7)
    basic_action_id = await basic_p_var.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.Basic.p.ActionId', 'ActionId', 0.0)
    ######In
    p_var = await in_var.add_variable(idx, 'p', 6.7)
    action_id = await p_var.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.ActionId', 'ActionId', 0.0)
    data_array = await p_var.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Data', 'Data', 0)
    data_0 = await data_array.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Data[0]', 'Data[0]', 0)
    data_1 = await data_array.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Data[1]', 'Data[1]', 0)

    ####STATUS
    status = await p_var.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Status', 'Status', 0.0)
    busy = await status.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Status.Busy', 'Busy', False)
    error = await status.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Status.Error', 'Error', False)
    enable = await status.add_variable(f'ns={idx}; s=|var|CPX-E-CEC-C1-PN.Application.G.In.p.Status.Enable', 'Enable', False)

    # Set MyVariable to be writable by clients
    #obj = await server.nodes.objects.add_object(idx, "Server", myserver)
    await myvar.set_writable()
    await basic_var.set_writable()
    await basic_p_var.set_writable()
    await basic_action_id.set_writable()
    await p_var.set_writable()
    await action_id.set_writable()
    await status.set_writable()
    await busy.set_writable()
    await error.set_writable()
    await enable.set_writable()
    await data_array.set_writable()
    await data_0.set_writable()
    await data_1.set_writable()
    await server.nodes.objects.add_method(ua.NodeId('ServerMethod', 2), ua.QualifiedName('ServerMethod', 2), func,
                                          [ua.VariantType.Int64], [ua.VariantType.Int64])

    # get the time step of the current world.
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    index = 0
    while index < robot.getNumberOfDevices():
        print(robot.getDeviceByIndex(index))
        index += 1
    con_belt = robot.getDeviceByIndex(2)
    con_belt.setPosition(float('inf'))
    con_belt.setVelocity(0.0)

    print('Starting server!')
   # starting!
    async with server:
        print("Available loggers are: ", logging.Logger.manager.loggerDict.keys())
        while True:
            await asyncio.sleep(0.005)
            robot.step(500)
            if await enable.get_value():
                print("Start Belt")
                await enable.write_value(False)
                start_belt(robot, con_belt)

if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    asyncio.run(main())
# Enter here exit cleanup code.
