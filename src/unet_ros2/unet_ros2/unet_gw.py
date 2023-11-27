import unet_msg.srv
import unet_msg.msg
import unetpy
import rclpy
from rclpy.node import Node

QUEUE_SIZE = 16
MONITOR_PERIOD = 0.1

class UnetGatewayService(Node):

    def __init__(self):
        super().__init__('unet')
        self.create_service(unet_msg.srv.DatagramReq, 'unet/tx/datagram', self.datagram_req)
        self.create_service(unet_msg.srv.AddressResolutionReq, 'unet/get/address', self.address_resolution_req)
        self.pub_datagram_ntf = self.create_publisher(unet_msg.msg.DatagramNtf, 'unet/rx/datagram', QUEUE_SIZE)
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 1100)
        self.declare_parameter('timeout', 1000)
        self.add_on_set_parameters_callback(self.params_changed)
        self.create_timer(MONITOR_PERIOD, self.monitor)
        self.sock = None

    def __del__(self):
        if self.sock:
            self.sock.close()
        self.sock = None

    def params_changed(self, params):
        for param in params:
            if param.name == 'host' or param.name == 'port':
                if self.sock:
                    self.sock.close()
                    self.sock = None

    def connect(self):
        if self.sock == None:
            host = self.get_parameter('host').value
            port = self.get_parameter('port').value
            try:
                self.sock = unetpy.UnetSocket(host, port)
                self.sock.setTimeout(0)
                self.gw = self.sock.getGateway()
                self.dsp = self.gw.agentForService(unetpy.Services.REMOTE)
            except:
                self.sock = None

    def address_resolution_req(self, request, response):
        self.connect()
        if request.name == '':
            response.address = self.sock.getLocalAddress()
        else:
            response.address = self.sock.host(request.name)
        return response

    def monitor(self):
        self.connect()
        try:
            rsp = self.sock.receive()
        except:
            return
        if rsp != None:
            msg = unet_msg.msg.DatagramNtf(
                data=rsp.data,
                protocol=rsp.protocol,
                to=rsp.to,
                from_addr=rsp.from_
            )
            self.pub_datagram_ntf.publish(msg)

    def datagram_req(self, request, response):
        self.connect()
        req = unetpy.DatagramReq(
            recipient=self.dsp,
            to=request.to,
            data=request.data.tolist(),
            protocol=request.protocol,
            robustness=request.robustness,
            priority=request.priority,
            shortcircuit=request.shortcircuit
        )
        if request.ttl >= 0:
            req.ttl = request.ttl
        if request.reliability == unet_msg.srv.DatagramReq.Request.RELIABLE:
            req.reliability = True
        elif request.reliability == unet_msg.srv.DatagramReq.Request.UNRELIABLE:
            req.reliability = False
        if request.route:
            req.route = request.route
        timeout = self.get_parameter('timeout').value
        rsp = self.gw.request(req, timeout)
        if rsp == None:
            response.agree = False
            response.id = ''
            response.reason = 'Timeout'
        elif rsp.performative == unetpy.Performative.AGREE:
            response.agree = True
            response.id = rsp.inReplyTo
            response.reason = ''
        else:
            response.agree = False
            response.id = ''
            response.reason = rsp.reason
        return response

def main(args=None):
    rclpy.init(args=args)
    ugw = UnetGatewayService()
    rclpy.spin(ugw)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
