#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_msgs.msg import String

class VoiceRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.pub = rospy.Publisher('/voice_commands', String, queue_size=10)
    def listen_loop(self):
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Voice recognizer ready for interplaneter recruitement:..")

            while not rospy.is_shutdown():
                try:
                    audio = self.recognizer.listen(source, timeout=3)
                    text = self.recognizer.recognize_google(audio).lower()
                    self.pub.publish(text)
                    rospy.loginfo(f"Heard: {text}")
                except sr.UnknownValueError:
                    rospy.logwarn("Could not understand audio")
                except sr.RequestError as e:
                    rospy.logerr(f"Recognition error: {e}")
                except sr.WaitTimeoutError:
                    continue

if __name__ == '__main__':
    rospy.init_node('speaker_node')
    vr = VoiceRecognizer()
    vr.listen_loop()
