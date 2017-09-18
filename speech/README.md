# Speech overview

The speech bundle supports both voice recognition and text to speech.

# Voice Recognition

Voice recognition uses Microsoft Bing API to recognize spoken words.  
It then detects the command type based on the following supported commands:

## Command:Regular Expressions

'command-bring-coffee': ['johnny.*coffee'],  
'coffee-ready': ['coffee.*ready', 'here.*coffee'],  
'floor-0': ['ground'],  
'floor-1': ['first', 'floor.*one'],  
'floor-2': ['second', 'floor.*two'],  
'floor-3': ['third', 'floor.*three'],  
'floor-4': ['fourth', 'floor.*four'],  
'floor-5': ['fifth', 'floor.*five'],  
'yes': ['yes', 'true', 'correct', 'exactly'],  
'no': ['no', 'false', 'incorrect', 'untrue']

To support a new command, edit voice_recognition.py and inside "def run(self):" of SpeechDetector search for the regular expressions dictionary above and add the new command. E.g.  

'no': ['no', 'false', 'incorrect', 'untrue'],  
'maybe': ['maybe', 'not sure']}

## Usage

Run "roscore" from the command line.  
Then run "python voice_recognition.py".  
Then run "rqt".  
Then select Plugins -> Services -> Service Caller from the main menu of rqt.  
Select "speech_to_text" from the dropdown menu and press "Call".  
Now you have to be silent until you see "* Mic set up and listening." in the command line.  
Then say a command such as: "Johnny bring me coffee" or "Coffee is ready".  
You will see the recognized command in the command line.
