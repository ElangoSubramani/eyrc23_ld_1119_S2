import rclpy
from swift_msgs.srv import CommandBool

async def call_service():
    node = rclpy.create_node('service_caller')
    
    # Create a client for the CommandBool service
    client = node.create_client(CommandBool, '/swift/cmd/arming')

    # Wait for the service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    # Prepare the request
    request = CommandBool.Request()
    request.value = False

    # Call the service asynchronously
    future = client.call_async(request)

    # Wait for the service call to complete
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node)

    # Check the result
    if future.result():
        response = future.result()
        node.get_logger().info(f'Response: {response.success}, Result: {response.result}')
    else:
        node.get_logger().error('Service call failed')

    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    # rclpy.spin_until_future_complete(rclpy.create_context(), call_service())
