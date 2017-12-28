#!/usr/bin/env python

import rospy

from PIL import Image
from sensor_msgs.msg import Image as SensorImage

from scene_description import processRequest
from tts import tts

done = False


def image_callback(data):
    global done
    if not done:
        done = True
        print('Processing image...')

        imgSize = (data.width, data.height)
        rawData = data.data
        img = Image.frombytes('RGB', imgSize, rawData)
        image_file = 'robot_image.png'
        img.save(image_file)

        with open(image_file, 'rb') as f:
            data = f.read()

        result = processRequest(data)

        if result is not None:
            description = result['description']['captions'][0]['text']
            print(description)
            tts(description)


def caption_image():
    rospy.init_node('imgcap')
    rospy.Subscriber("/kinect2/qhd/image_color", SensorImage, image_callback)
    print "Demo is ready"
    rospy.spin()


if __name__ == "__main__":
    caption_image()
