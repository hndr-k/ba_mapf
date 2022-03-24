import logging
import asyncio
import sys

sys.path.insert(0, "..")

from asyncua import ua, Server
from asyncua.common.methods import uamethod
from opcua.server.user_manager import UserManager
from asyncua.ua.uatypes import StringNodeId
from events import Events

@uamethod
def func(parent, value):
    return value * 2


events = Events()


async def start_belt(enable):
    print('Enable Event triggered.. START BELT' + str(enable))
    print('Conveyor Belt started')
    await enable.write_value(False)

async def main():
    _logger = logging.getLogger('asyncua')
    # setup our server
    server = Server()

    server.set_endpoint('opc.tcp://127.0.0.1:4840/')
    await server.init()
    server.private_key = None
    server.certificate = None
    server.user_manager = UserManager(parent=server)
    server._security_policy = [ua.SecurityPolicyType.NoSecurity]
    server._policyIDs = ["Anonymous"]
    # setup our own namespace, not really necessary but should as spec
    uri = 'http://examples.freeopcua.github.io'
    address_space = 4
    s = "|var|CPX-E-CEC-C1-PN"
    idx_uri = await server.register_namespace(uri)
    idx_uri_sec = await server.register_namespace('test')
    idx = await server.register_namespace('test-fh-aachen')
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

    events.on_change += start_belt
    _logger.info('Starting server!')
    async with server:
        while True:
            await asyncio.sleep(1)
            new_val = await myvar.get_value() + 0.1
            #_logger.info('Set value of %s to %.1f', myvar, new_val)
            await myvar.write_value(new_val)
            _logger.info('Enable %r',await enable.get_value())
            if await enable.get_value():
                print('Fire Event')
                await start_belt(enable)
                #await enable.write_value(False)
                #events.on_change(enable)
                #events.on_change -= start_belt


if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    asyncio.run(main(), debug=False)
