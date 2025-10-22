#!/usr/bin/python3.9

import rospy
from VttPublisher import VttPublisher

def main():    
        
    rospy.init_node("voice_to_text_node")
    rospy.loginfo("Voice to Text ROS Node Initializing ...")
    
    topic = "/voice_to_text"
    pkg_name = "voice_recognizer"
    button_key = "enter"
    audio_path = "/voices/voice_recorded.wav"
    
    model = "medium"
    auto_detect = False
    lang = "en"
    sample_rate = 32000
    
    publisher = VttPublisher(topic, pkg_name, audio_path, button_key, sample_rate, model, auto_detect, lang)
    rospy.sleep(1)
    rospy.loginfo("Voice to Text Publisher Initialized.")
    
    rospy.loginfo("Voice to Text Publisher Running...")
    rospy.loginfo(" ")
    rospy.loginfo("\n\n Press " + button_key + " to record.")
    publisher.run_publisher()

if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Node interrupted.")
