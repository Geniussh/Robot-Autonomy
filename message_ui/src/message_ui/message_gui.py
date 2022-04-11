# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin
from message_ui.msg import sent_recipe
from message_ui.msg import reply_msg


class MessageGUI(Plugin):

    def reply_msg_callback(self, msg_in):
        self._widget.reply.setText(msg_in.message)


    def __init__(self, context):
        super(MessageGUI, self).__init__(context)
        self.setObjectName('MessageGUI')

        self.message_pub = rospy.Publisher("sent_recipe", sent_recipe, queue_size=1000)

        rospy.Subscriber("reply_msg", reply_msg, self.reply_msg_callback)

        self.msg_to_send = sent_recipe()
        self.counter_req_id = -1

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('message_ui'), 'resource', 'MessageGUI.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MessageGUIui')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._widget.red_shots.textChanged.connect(self._red_shots_changed) 
        self._widget.green_shots.textChanged.connect(self._green_shots_changed) 
        self._widget.blue_shots.textChanged.connect(self._blue_shots_changed) 
        
        self._widget.send_message.pressed.connect(self._on_send_message_pressed)
        

    def _red_shots_changed(self, msg):
        msg = int(msg) if msg else 0
        self.msg_to_send.Red = msg

    def _green_shots_changed(self, msg):
        msg = int(msg) if msg else 0
        self.msg_to_send.Green = msg

    def _blue_shots_changed(self, msg):
        msg = int(msg) if msg else 0
        self.msg_to_send.Blue = msg

    def _on_send_message_pressed(self):
        self.msg_to_send.header.stamp = rospy.Time.now()
        self.message_pub.publish(self.msg_to_send)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        
        
        
        
        
        
        
        