#!/usr/bin/env python3
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image

from typing import Final
import numpy as np
from cv_bridge import CvBridge

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"

bridge = CvBridge()

def generate_image(width=320, height=240):
    img = np.random.randint(0, 256, (height, width), dtype=np.uint8)
    image_message = bridge.cv2_to_imgmsg(img, encoding="mono8")
    return image_message


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  # Потому что rospy.Publisher() автоматически использует имя ROS_IMAGE_TOPIC,
  # и если оно меняется при запуске, ROS сам его корректно интерпретирует
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)

  while not rospy.is_shutdown():
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    image_message = generate_image()
    publisher.publish(image_message)
    
    rate.sleep()


if __name__ == '__main__':
    main()
