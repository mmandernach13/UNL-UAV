from rclpy import Node

# Placeholder for the PayloadControlNode class
# need to be able to pick which payload to control
# bottle or beacon. make the server side of the service

class PayloadControlNode(Node):
    def __init__(self):
        super().__init__('payload_control_node')
        # Initialization code here

    def control_payload(self):
        pass

def main():
    pass

if __name__ == "__main__":
    main()