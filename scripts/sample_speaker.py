#!/usr/bin/env python
#encoding: utf8
import rospy
from speak_tools.srv import *
import sys


def speak_client(text):
    rospy.wait_for_service("speak_text")
    try:
        speak_text = rospy.ServiceProxy("speak_text", SpeakedText)
        resp = speak_text(text)
        return resp.message
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    return "%s <text>" % sys.argv[0]


if __name__ == "__main__":
    rospy.init_node('sample_speaker')
    print "%s" % (speak_client("こんばんわに"))

"""
speak_server.pyのテスト用
"""
