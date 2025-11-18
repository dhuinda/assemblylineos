#!/usr/bin/env python3

"""
Sensor Controller for Assembly Line OS

This node subscribes to sensor topics (digital and analog) and publishes
sensor status updates. It supports multiple sensor types including limit switches,
encoders, and analog sensors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Float32
import json
import time


class SensorController(Node):
    """Manages sensor subscriptions and publishes sensor status"""
    
    def __init__(self):
        super().__init__('sensor_controller')
        
        # Track sensor states
        self.sensor_states = {}
        
        # Sensor subscriptions (dynamically created)
        self.sensor_subscriptions = {}
        
        # Publish sensor status updates
        self.sensor_status_pub = self.create_publisher(
            String, 'sensor/status', 10)
        
        # Subscribe to sensor registration requests
        self.register_sub = self.create_subscription(
            String, 'sensor/register', 
            self.register_sensor_callback, 10)
        
        # Update sensor status periodically
        self.status_timer = self.create_timer(0.1, self.publish_all_status)  # 10 Hz
        
        self.get_logger().info('Sensor controller node started')
    
    def register_sensor_callback(self, msg):
        """Register a new sensor to monitor"""
        try:
            data = json.loads(msg.data)
            sensor_id = data.get('sensor_id')
            topic = data.get('topic')
            sensor_type = data.get('type', 'digital')  # 'digital' or 'analog'
            
            if not sensor_id or not topic:
                self.get_logger().warn('Invalid sensor registration: missing sensor_id or topic')
                return
            
            # Initialize sensor state
            self.sensor_states[sensor_id] = {
                'sensor_id': sensor_id,
                'topic': topic,
                'type': sensor_type,
                'value': False if sensor_type == 'digital' else 0.0,
                'last_update': time.time(),
                'connected': False
            }
            
            # Subscribe to the sensor topic
            self.subscribe_to_sensor(sensor_id, topic, sensor_type)
            
            self.get_logger().info(f'Registered sensor {sensor_id} on topic {topic} (type: {sensor_type})')
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse sensor registration: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error registering sensor: {e}')
    
    def subscribe_to_sensor(self, sensor_id, topic, sensor_type):
        """Subscribe to a sensor topic"""
        if sensor_id in self.sensor_subscriptions:
            # Already subscribed
            return
        
        try:
            if sensor_type == 'digital':
                # Digital sensors use Bool messages
                sub = self.create_subscription(
                    Bool, topic,
                    lambda msg: self.sensor_callback(sensor_id, msg.data, sensor_type),
                    10)
            elif sensor_type == 'analog':
                # Analog sensors use Float32 messages
                sub = self.create_subscription(
                    Float32, topic,
                    lambda msg: self.sensor_callback(sensor_id, msg.data, sensor_type),
                    10)
            else:
                # Default to String messages
                sub = self.create_subscription(
                    String, topic,
                    lambda msg: self.sensor_callback(sensor_id, msg.data, sensor_type),
                    10)
            
            self.sensor_subscriptions[sensor_id] = sub
            self.sensor_states[sensor_id]['connected'] = True
            
            self.get_logger().info(f'Subscribed to sensor {sensor_id} on topic {topic}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to sensor {sensor_id}: {e}')
    
    def sensor_callback(self, sensor_id, value, sensor_type):
        """Handle sensor value updates"""
        if sensor_id not in self.sensor_states:
            return
        
        self.sensor_states[sensor_id]['value'] = value
        self.sensor_states[sensor_id]['last_update'] = time.time()
        self.sensor_states[sensor_id]['connected'] = True
        
        # Publish status update
        self.publish_sensor_status(sensor_id)
    
    def publish_sensor_status(self, sensor_id):
        """Publish status for a specific sensor"""
        if sensor_id not in self.sensor_states:
            return
        
        sensor = self.sensor_states[sensor_id]
        
        status = {
            'sensor_id': sensor_id,
            'type': sensor['type'],
            'value': sensor['value'],
            'connected': sensor['connected'],
            'last_update': sensor['last_update']
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.sensor_status_pub.publish(msg)
    
    def publish_all_status(self):
        """Publish status for all sensors"""
        for sensor_id in self.sensor_states:
            self.publish_sensor_status(sensor_id)


def main(args=None):
    rclpy.init(args=args)
    sensor_controller = SensorController()
    
    try:
        rclpy.spin(sensor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

