import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class gotoClosest(Node):
    def __init__(self):
        super().__init__('gotoClosest')
        self.srv = self.create_service(SetBool, 'gotoClosest', self.callback)
    
    def callback(self, request, response):
        self.go = request.data

        if self.go:
            response.message = "ON"
        else:
            response.messgae = "OFF"
        response.success = True
        return response

        
def main(args=None):
    rclpy.init(args=args)
    goto_closest = gotoClosest()
    rclpy.spin(goto_closest)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
