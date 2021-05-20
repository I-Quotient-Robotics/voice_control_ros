#! /usr/bin/env python

import sys
import string

import rospy
import actionlib

import voice_control.msg
import std_msgs.msg
import speech_recognition_msgs.msg


class VoiceCommandClient():
    def __init__(self):
        rospy.Subscriber('speech_to_text', speech_recognition_msgs.msg.SpeechRecognitionCandidates, self.__speech_recog_callback)

        self.__action_client = actionlib.SimpleActionClient('task_server', voice_control.msg.TaskServerAction)
        rospy.loginfo('wait for sever')
        self.__action_client.wait_for_server()
        rospy.loginfo('server connected')
        self.__hotword = rospy.get_param('voice_command/hotword', 'Alexa')
        self.__command_list = rospy.get_param('voice_command/command')
        for x in self.__command_list:
            print x['keywords']
        rospy.loginfo('Voice Command Client initialized.')

    def __speech_recog_callback(self, data):
        raw_data = data.transcript[0]

        if raw_data.startswith(self.__hotword):
            data_split = raw_data.translate(None, string.punctuation).lower().split(' ')
            rospy.loginfo('data split: %s', data_split)

            for command in self.__command_list:
                if all(x in data_split for x in command['keywords']):
                    rospy.loginfo('get command: %s', command['task'])
                    goal = voice_control.msg.TaskServerGoal()
                    goal.task = command['task']
                    goal.value = command['value']

                    self.__action_client.send_goal(goal)
                    self.__action_client.wait_for_result(rospy.Duration(0))
                    # if self.__action_client.wait_for_result(rospy.Duration(20.0)):
                        # rospy.loginfo('voice command action result: %s', self.__action_client.get_result())
                    # else:
                        # rospy.logwarn('voice command action result: action timeout, cancel all')
                        # self.__action_client.cancel_all_goals()

                    break


def main():
    rospy.init_node('voice_command_client')
    client = VoiceCommandClient()
    rospy.spin()

if __name__ == '__main__':
    main()
