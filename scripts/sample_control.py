#!/usr/bin/env python
#encoding: utf8
import rospy
from subprocess import *
from speak_tools.srv import *
from std_msgs.msg import String
import rosnode
import MeCab
import time


def speak_client(text):
    # 発話させる
    rospy.wait_for_service("speak_text")
    try:
        speak_text = rospy.ServiceProxy("speak_text", SpeakedText)
        resp = speak_text(text)
        return resp.message
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


class SampleControl():
    def __init__(self):
        sub = rospy.Subscriber("/speech", String, self.get_words)
        self.words = []  # 1文を単語単位に分割して格納
        self.isLaunched = False  # 今、呼び出されているアプリがあるか
        self.crr_node = ''  # 　現在起動中のアプリのノード名

    def parse(self, text):
        """/speechを単語単位に分割"""
        m = MeCab.Tagger("-Ochasen")
        node = m.parseToNode(text)
        words_list = []
        while node:
            word = node.surface
            wclass = node.feature.split(',')
            if wclass[0] != 'BOS/EOS':
                words_list.append(word)
            node = node.next
        return words_list

    def get_words(self, msg):
        """Subscriber用"""
        self.words = self.parse(msg.data)

    def call_app(self, pkg, type):
        """
        指定したアプリケーションを呼び出す関数
        プログラムが正常終了すると処理を抜ける。
        @param pkg 呼び出したいノードのパッケージ名
        @param type ファイル名
        @exsample call_app('speak_tools', 'speak_client.py')
        """
        call(['rosrun', pkg, type])

    def call_app_mult(self, pkg, type):
        """
        指定したアプリケーションを呼び出す関数
        起動したノードをマルチプロセスで起動したまま次の処理へ移行。
        @param pkg 呼び出したいノードのパッケージ名
        @param type ファイル名
        """
        Popen(['rosrun', pkg, type])

    def kill_node(self, name):
        """
        指定されたノード名を持つノードをkillする。
        @param name ノード名
        """
        call(['rosnode', 'kill', '/' + name])

    def node_is_launched(self, name):
        """
        指定したノードが起動しているのかを返す
        """
        return rosnode.rosnode_ping('/' + name, max_count=1, verbose=False)


def start_app(sc, pkg, type, name):
    """アプリをスタートさせる"""
    sc.isLaunched = True
    sc.crr_node = name
    sc.call_app_mult(pkg, type)
    while sc.node_is_launched(sc.crr_node) is False:
        # 読んだノードが立ち上がるまで待つ
        pass
    sc.words = []


def start_app_not_loop(sc, pkg, type, name):
    """自動的に終わるようなアプリをスタートさせる"""
    sc.isLaunched = True
    sc.crr_node = name
    sc.call_app(pkg, type)
    sc.words = []


def end_app(sc):
    """アプリを終了させる"""
    if sc.crr_node == "weather_forecast":
        info_msg = "天気予報モードを終了します。"
        speak_client(info_msg)
        print info_msg + "\n"
    sc.kill_node(sc.crr_node)
    rospy.loginfo("END: " + sc.crr_node)
    sc.isLaunched = False
    sc.crr_node = ""
    sc.words = []


def exit_control(sc):
    """このノード自身を終了させる"""
    sc.kill_node(sc.crr_node)
    rospy.loginfo("END: " + sc.crr_node)
    rospy.signal_shutdown('Quit')


if __name__ == '__main__':
    rospy.init_node('sample_control')
    sc = SampleControl()
    speak_client("ろぼすけを起動しました。")
    time.sleep(1.5)
    flag = False  # モード選択で発話するか否か
    while not rospy.is_shutdown():
        if sc.isLaunched is False:
            """起動しているアプリがないので、呼ばれるまで待機"""
            if flag is False:
                info_msg = "モードを選択してください"
                speak_client(info_msg)
                print info_msg + "\n"
                flag = True
            preword = ""
            for word in sc.words:
                if word == "人見知り":
                    info_msg = "人見知りモードを開始します。"
                    speak_client(info_msg)
                    print info_msg + "\n"
                    time.sleep(1.5)
                    start_app_not_loop(sc, 'shy_rbsk',
                                       'shy_robot.py', 'shy_robot_node')
                    flag = False
                    break
                elif word == "予報" and preword == "天気":
                    info_msg = "天気予報モードを開始します。"
                    speak_client(info_msg)
                    print info_msg + "\n"
                    time.sleep(1.5)
                    start_app(sc, 'sweather_forecast',
                              'weather_forecast.py', "sweather_forecast")
                    info_msg = "お天気をお調べします。地名を教えてください"
                    speak_client(info_msg)
                    print info_msg + "\n"
                    flag = False
                    break
                elif word == "バイバイ":
                    speak_client("ろぼすけを終了します")
                    rospy.signal_shutdown('Quit')
                preword = word
        elif sc.isLaunched is True:
            """アプリが起動しているので、終了させる"""
            if sc.node_is_launched(sc.crr_node) is False:
                """こちらからkillせず、アプリ側で終了した場合"""
                rospy.loginfo("END: " + sc.crr_node)
                sc.isLaunched = False
                sc.crr_node = ""
                continue

            for word in sc.words:
                if word == "終わり":
                    end_app(sc)
                    break
                elif word == "バイバイ":
                    # このROSノードをシャットダウン
                    exit_control(sc)
