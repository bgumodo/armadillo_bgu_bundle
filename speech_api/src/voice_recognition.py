#!/usr/bin/env python

# http://askubuntu.com/questions/691109/how-do-i-install-ffmpeg-and-codecs
import ffmpy

import os
import pyaudio
import wave
import audioop
from collections import deque
import time
import math
from threading import Thread

from bing import BingSpeechAPI
from std_msgs.msg import String
from std_srvs.srv import Trigger
#from text_to_speech.srv

import rospy

__author__ = 'dan'


class SpeechDetector:
    def __init__(self):
        self.pub = rospy.Publisher('speech_text', String, queue_size=10)

        # Microphone stream config.
        self.CHUNK = 8192#1024  # CHUNKS of bytes to read each time from mic
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RECORD_RATE = 44100 # The default rate of the microphone
        self.RECOGNITION_RATE = 16000 # Must be 16000 and not 44100 in order for bing to work!
        # FFMPEG is used to convert from 44100 to 16000:
        self.FFMPEG_PATH = 'ffmpeg'

        self.SILENCE_LIMIT = 1  # Silence limit in seconds. The max ammount of seconds where
                           # only silence is recorded. When this time passes the
                           # recording finishes and the file is decoded

        self.PREV_AUDIO = 0.5  # Previous audio (in seconds) to prepend. When noise
                          # is detected, how much of previously recorded audio is
                          # prepended. This helps to prevent chopping the beginning
                          # of the phrase.

        self.THRESHOLD = 4500
        self.num_phrases = -1

    def setup_mic(self, num_samples=10):
        """ Gets average audio intensity of your mic sound. You can use it to get
            average intensities while you're talking and/or silent. The average
            is the avg of the .2 of the largest intensities recorded.
        """
        rospy.loginfo("Getting intensity values from mic.")
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RECORD_RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        values = [math.sqrt(abs(audioop.avg(stream.read(self.CHUNK), 4)))
                  for x in range(num_samples)]
        values = sorted(values, reverse=True)
        r = sum(values[:int(num_samples * 0.2)]) / int(num_samples * 0.2)
        rospy.loginfo(" Finished ")
        rospy.loginfo(" Average audio intensity is " + str(r))
        stream.close()
        p.terminate()

        # if r < 3000:
        #     self.THRESHOLD = 3500
        # else:
        #     self.THRESHOLD = r + 100

        self.THRESHOLD = r + 100

        rospy.loginfo('Threshold:' + str(self.THRESHOLD))

    def save_speech(self, data, p):
        """
        Saves mic data to temporary WAV file. Returns filename of saved
        file
        """
        filename = 'output_'+str(int(time.time()))
        temp_filename = filename + '_temp.wav'
        # writes data to WAV file
        data = ''.join(data)
        wf = wave.open(temp_filename, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(self.RECORD_RATE)
        wf.writeframes(data)
        wf.close()

        ff = ffmpy.FFmpeg(executable=self.FFMPEG_PATH,
                          inputs={temp_filename: None},
                          outputs={filename + '.wav': '-ar ' + str(self.RECOGNITION_RATE) + ' -ac 1'})
        ff.run()

        os.remove(temp_filename)

        return filename + '.wav'

    def run(self):
        """
        Listens to Microphone, extracts phrases from it and calls bing
        to decode the sound
        """
        self.setup_mic()

        #Open stream
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RECORD_RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)
        rospy.loginfo("* Mic set up and listening. ")

        audio2send = []
        cur_data = ''  # current chunk of audio data
        rel = self.RECORD_RATE/self.CHUNK
        slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
        #Prepend audio from 0.5 seconds before noise was detected
        prev_audio = deque(maxlen=self.PREV_AUDIO * rel)
        started = False

        bing = BingSpeechAPI()

        text = None
        while text is None:
            cur_data = stream.read(self.CHUNK)
            slid_win.append(math.sqrt(abs(audioop.avg(cur_data, 4))))

            if sum([x > self.THRESHOLD for x in slid_win]) > 0:
                if started == False:
                    rospy.loginfo("Starting recording of phrase")
                    started = True
                audio2send.append(cur_data)

            elif started:
                rospy.loginfo("Finished recording, decoding phrase")
                filename = self.save_speech(list(prev_audio) + audio2send, p)

                wave_file = wave.open(filename, 'rb')
                speech = wave_file.readframes(wave_file.getnframes())
                wave_file.close()

                try:
                    text = bing.recognize(speech, language='en-US')
                    text = text.encode('utf-8')
                    rospy.loginfo('STT:{}'.format(text))
                    self.pub.publish(String(text))

                except ValueError:
                    rospy.loginfo('STT: ValueError')
                finally:
                    # Removes temp audio file
                    os.remove(filename)

                # Reset all
                started = False
                slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
                prev_audio = deque(maxlen=0.5 * rel)
                audio2send = []
                #rospy.loginfo("Listening ...")

            else:
                prev_audio.append(cur_data)

        rospy.loginfo("* Done listening")
        stream.close()
        p.terminate()


def speech_to_text_thread():
    sd = SpeechDetector()
    sd.run()

def run_speech_to_text(ignore_me):
    t = Thread(target=speech_to_text_thread)
    t.start()
    return True, ""

def run_text_to_speech(ignore_me):
    bing = BingSpeechAPI()
    bing.text_to_speech(text='Here is your coffee')

def main():
    NODE_NAME = "SpeechAPI"
    PACKAGE_NAME = "speech_api"
    debugLevel = rospy.DEBUG

    fname = NODE_NAME
    rospy.init_node(NODE_NAME, anonymous=False, log_level=debugLevel)

    rospy.loginfo("{}: Initializing speech api node".format(fname))

    service = rospy.Service('speech_to_text', Trigger, run_speech_to_text)
    service = rospy.Service('text_to_speech', Trigger, run_text_to_speech)

    rospy.spin()

if __name__ == "__main__":
    main()
