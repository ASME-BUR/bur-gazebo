#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import pygame

class KeyboardJoy(Node):

    def __init__(self):
        super().__init__('bur_keyboard_joy')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        timer_period = 1.0 / 30 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pygame.event.get()
        keys = pygame.key.get_pressed()
        
        msg = Joy()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 10
        
        # Axes assignments based on PS4 yaml under bur_rov/config
        msg.axes[1] = -1.0 * (keys[pygame.K_w] - keys[pygame.K_s]) # linear X
        # msg.axes[3] = 1.0 * (keys[pygame.K_a] - keys[pygame.K_d]) # linear Y
        msg.axes[4] = 1.0 * (keys[pygame.K_SPACE] - keys[pygame.K_LSHIFT]) # linear Z
        # msg.axes[6] = 1.0 * (keys[pygame.K_q] - keys[pygame.K_e]) # angular X
        # msg.axes[7] = 1.0 * (keys[pygame.K_UP] - keys[pygame.K_DOWN]) # angular Y
        msg.axes[3] = 1.0 * (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) # angular Z
        
        msg.buttons[9] = 1

        self.publisher_.publish(msg)
        

def main(args=None):
    pygame.init()
    window = pygame.display.set_mode((640,480))
    rclpy.init(args=args)
    node = KeyboardJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
