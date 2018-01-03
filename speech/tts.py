# sudo pip install git+https://github.com/westparkcom/Python-Bing-TTS.git
# sudo apt-get install python-pyaudio python3-pyaudio

import os
from bingtts import Translator
import pyaudio
import wave


def main():
    tts()


def tts(text="Hi there"):
    translator = Translator(os.getenv('BING_KEY', ''))
    output = translator.speak(text, "en-US", "Female", "riff-16khz-16bit-mono-pcm")

    with open('output.wav', 'wb') as f:
        f.write(output)

    CHUNK = 1024

    wf = wave.open('output.wav', 'rb')

    # instantiate PyAudio (1)
    p = pyaudio.PyAudio()

    # open stream (2)
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    # read data
    data = wf.readframes(CHUNK)

    # play stream (3)
    while len(data) > 0:
        stream.write(data)
        data = wf.readframes(CHUNK)

    # stop stream (4)
    stream.stop_stream()
    stream.close()

    # close PyAudio (5)
    p.terminate()


if __name__ == '__main__':
    main()
