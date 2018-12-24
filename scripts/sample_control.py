#!/usr/bin/env python
#encoding: utf8
import rospy
from subprocess import *
from std_msgs.msg import String
import rosnode
import MeCab


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


if __name__ == '__main__':
    rospy.init_node('sample_control')
    sc = SampleControl()

    while not rospy.is_shutdown():
        if sc.isLaunched is False:
            """起動しているアプリがないので、呼ばれるまで待機"""
            for word in sc.words:
                if word == "テスト":
                    sc.isLaunched = True
                    sc.crr_node = 'sample_sleep'
                    sc.call_app_mult('sample_controller', 'sample_sleep.py')
		    sc.words = []
                    break
                elif word == "話す":
                    sc.isLaunched = True
                    sc.crr_node = "sample_speaker"
                    sc.call_app('sample_controller', 'sample_speaker.py')
		    sc.words = []
                    break
        elif sc.isLaunched is True:
            """アプリが起動しているので、終了させる"""
            if sc.node_is_launched(sc.crr_node) is False:
                """こちらからkillせず、アプリ側で終了した場合"""
                sc.isLaunched = False
                sc.crr_node = ""
                continue

            for word in sc.words:
                if word == "終わり":
                    sc.kill_node(sc.crr_node)
                    rospy.loginfo("END: " + sc.crr_node)
                    sc.isLaunched = False
                    sc.crr_node = ""
		    sc.words = []
                    break
