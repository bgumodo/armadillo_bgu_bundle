#!/usr/bin/env python

import rospy

from PIL import Image, ImageTk, ImageDraw
from sensor_msgs.msg import Image as SensorImage

from objects_detection import processRequest, parse_query
from tts import tts

from voice_recognition import SpeechDetector

from Tkinter import Tk, Label, Button

done = False
image_file = 'two_cups.jpg'


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

def caption_image():
    rospy.init_node('demo')
    rospy.Subscriber("kinect2/qhd/image_color", SensorImage, image_callback)
    # rospy.Subscriber("/front_camera/image_raw", SensorImage, image_callback)

    print "Demo is ready"
    rospy.spin()


class MyFirstGUI:
    def __init__(self, master, image_file):
        self.master = master
        master.title("Demo")

        self.start_demo_button = Button(master, text="Start", command=self.start_demo)
        self.start_demo_button.pack()

        image = Image.open(image_file)
        photo = ImageTk.PhotoImage(image)
        self.label = Label(image=photo)
        self.label.image = photo  # keep a reference!
        self.label.pack()

    def draw_box(self, y1, x1, y2, x2):
        global image_file

        image = Image.open(image_file)

        draw = ImageDraw.Draw(image)

        draw.rectangle((x1, y1, x2, y2), outline=(0, 0, 200))

        del draw

        photo = ImageTk.PhotoImage(image)
        self.label.configure(image=photo)
        self.label.image = photo  # keep a reference!

    def query_callback(self, query, ignore_params=None):
        global image_file
        query = query.replace('cap', 'cup')
        print(query)

        if query == '<unrecognized speech>':
            return

        parsed_query = parse_query(query)

        subject = parsed_query['subject']
        label = parsed_query['label']
        if label is None:
            return

        with open(image_file, 'rb') as f:
            data = f.read()

        response = processRequest(data)
        print(response)

        candidate_indices = []

        for i, class_name in enumerate(response['result']['class_names']):
            if class_name.startswith(label + '|'):
                candidate_indices.append(i)

        if len(candidate_indices) == 1:
            y1, x1, y2, x2 = response['result']['yx_boxes'][0]
            self.draw_box(y1, x1, y2, x2)
            tts("Here is the " + subject)
        elif len(candidate_indices) == 2:
            candidate_boxes = []

            for i in candidate_indices:
                y1, x1, y2, x2 = response['result']['yx_boxes'][i]
                x_center = (x1 + x2) / 2

                candidate_boxes.append((y1, x1, y2, x2, x_center))

            candidate_boxes.sort(key=lambda box: box[4])

            tts("Do you mean the " + subject + " on the left or the " + subject + " on the right?")
            sd = SpeechDetector()
            sd.run(self.answer_callback, (candidate_boxes, subject))

    def answer_callback(self, answer, candidate_boxes_and_subject):
        print(answer)
        if answer=='<unrecognized speech>':
            tts('I didn\'t understand. Please try again.')
            sd = SpeechDetector()
            sd.run(self.answer_callback, candidate_boxes_and_subject)
            return

        if 'left' in answer:
            box = candidate_boxes_and_subject[0][0]
        else:
            box = candidate_boxes_and_subject[0][1]

        self.draw_box(box[0], box[1], box[2], box[3])
        tts("Here is the " + candidate_boxes_and_subject[1])



    def start_demo(self):
        sd = SpeechDetector()
        sd.run(self.query_callback)

def main():
    global image_file

    root = Tk()
    my_gui = MyFirstGUI(root, image_file)

    root.mainloop()


if __name__ == "__main__":
    main()
    # caption_image()
