# Voice Recognizer ROS1

ROS1 package to recognize human voice and converts to text. Firstly, voice is recorded and converted to audio file using VoiceRecognizer. Then, audio file is sent to openai-whisper for speech to text conversion. Finally, recognized text is published to ROS topic.

## Installation

You need to install the following packages:

```
pip install pynput
pip install openai-whisper
pip install sounddevice numpy scipy 
```

## Usage

You can launch the node using the following command:

```
roslaunch voice_recognizer start_recognizing.launch
```
