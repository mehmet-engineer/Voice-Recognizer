import os
from pynput.keyboard import Listener, Key

import rospy, rospkg
from std_msgs.msg import String

from VoiceToText import VoiceToText
from VoiceRecognizer import VoiceRecognizer

class VttPublisher:
    def __init__(self, topic: str, pkg_name: str, audio_path: str, button_key: str, sample_rate: int, model: str, auto_detect: bool, lang: str):
        
        self.topic = topic
        self.pkg_name = pkg_name
        self.audio_path = audio_path
        self.button_key = button_key
        self.sample_rate = sample_rate
        self.model = model
        self.auto_detect = auto_detect
        self.lang = lang
        
        self.queue_size = 10
        self.publisher = rospy.Publisher(self.topic, String, queue_size=self.queue_size)
        self.pkg_path = rospkg.RosPack().get_path(self.pkg_name)
        self.audio_gen_path = self.pkg_path + self.audio_path
        
        self.recognizer = VoiceRecognizer(sample_rate)
        self.transcriber = VoiceToText(model, auto_detect, lang)
        
        kbd_listener = Listener(on_press=self.on_press, on_release=self.on_release)
        kbd_listener.start()
        
        self.keys = {
            "enter": Key.enter,
            "space": Key.space
        }
        
        self.key_list = [i for i in self.keys.keys()]
        
        if self.button_key not in self.key_list:
            raise ValueError(f"Invalid key: {self.button_key}. Available keys are: {self.key_list}")
        
        self._valid_key = self.keys[self.button_key]

    def publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        
    def on_press(self, key):
        if not self.recognizer.is_recording():
            if key == self._valid_key:
                self.recognizer.start_recording()
                rospy.loginfo("Recording...")
            
    def on_release(self, key):
        if self.recognizer.is_recording():
            if key == self._valid_key:
                self.recognizer.stop_recording()
                rospy.loginfo("Recording stopped.")
                
                print(self.audio_gen_path)
                self.recognizer.save(self.audio_gen_path)
                rospy.loginfo("Voice audio saved.")
                
                rospy.loginfo("Converting voice to text...")
                text = self.transcriber.convert_to_text(self.audio_gen_path)
                os.remove(self.audio_gen_path)
                
                self.publish_text(text)
                rospy.loginfo("Converted text: " + text)
                rospy.loginfo("Text published to ROS topic: " + self.topic)
                rospy.loginfo("")
                rospy.loginfo("\n\n Press " + self.button_key + " to record.")
    
    def run_publisher(self):
        rospy.spin()