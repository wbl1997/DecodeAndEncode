import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os

class ImageSaver:
    def __init__(self, topic, save_dir, frame_count):
        self.topic = topic
        self.save_dir = save_dir
        self.frame_count = frame_count
        self.current_frame = 0

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        log_dir = os.path.join(save_dir, 'logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        rospy.init_node('image_saver', anonymous=True)
        rospy.Subscriber(self.topic, Image, self.callback)
        rospy.spin()

    def callback(self, data):
        if self.current_frame < self.frame_count:
            self.save_image(data)
            self.current_frame += 1
        else:
            rospy.signal_shutdown('Finished saving images')

    def save_image(self, data):
        try:
            height = data.height
            width = data.width
            encoding = data.encoding
            is_bigendian = data.is_bigendian
            step = data.step
            image_data = np.frombuffer(data.data, dtype=np.uint8).reshape((height, width, -1))

            resized_image = cv2.resize(image_data, (640, 480))

            image_filename = os.path.join(self.save_dir, "frame_{}.jpg".format(self.current_frame))

            cv2.imwrite(image_filename, resized_image)
            rospy.loginfo("Saved {}".format(image_filename))
        except Exception as e:
            rospy.logerr("Failed to save image: {}".format(e))

if __name__ == '__main__':
    topic = '/iray/thermal_img'
    save_dir = './PCD/JPG'
    frame_count = 100

    ImageSaver(topic, save_dir, frame_count)
