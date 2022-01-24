#!/usr/bin/env python

import socket
import sys

import rosgraph
import rospy
import rosnode
import rostopic
from asciimatics.event import KeyboardEvent
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Divider
from asciimatics.widgets import Frame
# from asciimatics.widgets import Label
from asciimatics.widgets import Layout
from asciimatics.widgets import ListBox
# from asciimatics.widgets import MultiColumnListBox
from asciimatics.widgets import PopUpDialog
from asciimatics.widgets import Text
from asciimatics.widgets import Widget
from asciimatics.exceptions import ResizeScreenError
from asciimatics.exceptions import StopApplication


def get_topic_data(master=None):
    pubs, subs = rostopic.get_topic_list(master=master)
    topic_data = {}
    # print(f"subs {len(subs)}")
    for topic in pubs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
            topic_data[name]['subscribers'] = []
        topic_data[name]['publishers'] = topic[2]
    # print(f"subs {len(pubs)}")
    for topic in subs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
            topic_data[name]['publishers'] = []
        else:
            pass
            # TODO(lucasw) check for collisions that have mismatched types
        topic_data[name]['subscribers'] = topic[2]
    return topic_data


def make_options(items, header):
    item_list = [(header, 0)]
    item_list.extend([(name, i + 1) for i, name in enumerate(items)])
    # rospy.loginfo(item_list)
    return item_list


class TopicFrame(Frame):
    def __init__(self, screen):
        super(TopicFrame, self).__init__(
            screen, screen.height, screen.width, has_border=False, name="My Form")

        self.master = rosgraph.Master('/rosnode')

        # Create the (very simple) form layout...
        layout = Layout([1, 1], fill_frame=True)
        self.add_layout(layout)

        # Now populate it with the widgets we want to use.

        row0_height = int(screen.height * 0.5) - 2
        row1_height = int(screen.height * 0.25)

        #######################################################################
        column = 0
        self._topic_details = Text()
        # self._topic_details.disabled = True
        self._topic_details.custom_colour = "field"
        layout.add_widget(self._topic_details, column)
        layout.add_widget(Divider(), column)

        # all topics
        self._topic_list = ListBox(row0_height, [], name="topics", add_scroll_bar=True,
                                   # on_select=self.select_topic_popup,
                                   on_change=self.on_topic_pick)
        layout.add_widget(self._topic_list, column)
        layout.add_widget(Divider(), column)

        # per node topics
        self._node_subs = ListBox(row1_height, [], name="node_subs",
                                  add_scroll_bar=True,
                                  on_change=self.on_node_sub_pick)
        layout.add_widget(self._node_subs, column)
        layout.add_widget(Divider(), column)

        self._node_pubs = ListBox(Widget.FILL_FRAME, [], name="node_pubs", add_scroll_bar=True,
                                  on_change=self.on_node_pub_pick)
        layout.add_widget(self._node_pubs, column)

        #######################################################################
        column = 1
        self._node_details = Text()
        layout.add_widget(self._node_details, column)
        layout.add_widget(Divider(), column)

        # all nodes
        self._node_list = ListBox(row0_height, [], name="nodes", add_scroll_bar=True,
                                  on_change=self.on_node_pick)
        layout.add_widget(self._node_list, column)
        layout.add_widget(Divider(), column)

        # per topic nodes
        self._topic_pub_list = ListBox(row1_height, [], name="publishers", add_scroll_bar=True,
                                       on_change=self.on_topic_pub_pick)
        layout.add_widget(self._topic_pub_list, column)
        layout.add_widget(Divider(), column)

        self._topic_sub_list = ListBox(Widget.FILL_FRAME, [], name="subscribers", add_scroll_bar=True,
                                       on_change=self.on_topic_sub_pick)
        layout.add_widget(self._topic_sub_list, column)

        # self._node_info.custom_colour = "field"

        # layout.add_widget(Label("rostopic list"))
        # layout.add_widget(Divider())
        # layout.add_widget(Label("Press Enter to select or `q` to quit."), column)

        self.update_lists()

        # Prepare the Frame for use.
        self.fix()

    def select_topic_popup(self):
        # Just confirm whenever the user actually selects something.
        name = self._topic_names[self._topic_list.value]
        self._scene.add_effect(PopUpDialog(self._screen, f"You selected: {name}", ["OK"]))

    def update_topic_details(self, ind, topic_list):
        if ind is None or ind == 0:
            return

        topic_name, _ = topic_list[ind]
        topic_data = self._topic_data[topic_name]
        self._topic_details.value = topic_data['type']

        subs = sorted(topic_data['subscribers'])
        self._topic_sub_list.options = make_options(subs, f"nodes subscribing to {topic_name}")

        pubs = sorted(topic_data['publishers'])
        self._topic_pub_list.options = make_options(pubs, f"nodes publishing on {topic_name}")

    def on_topic_pick(self):
        self.update_topic_details(self._topic_list.value, self._topic_list.options)

    def on_node_sub_pick(self):
        self.update_topic_details(self._node_subs.value, self._node_subs.options)

    def on_node_pub_pick(self):
        self.update_topic_details(self._node_pubs.value, self._node_pubs.options)

    def update_node_details(self, ind, node_list):
        if ind is None or ind == 0:
            return

        node_name, _ = node_list[ind]

        pubs = sorted([t for t, l in self.state[0] if node_name in l])
        self._node_pubs.options = make_options(pubs, f"topics published from {node_name}")

        subs = sorted([t for t, l in self.state[1] if node_name in l])
        self._node_subs.options = make_options(subs, f"topics subscribed to by {node_name}")

        # srvs = sorted([t for t, l in state[2] if node_name in l])

    def on_node_pick(self):
        self.update_node_details(self._node_list.value, self._node_list.options)

    def on_topic_sub_pick(self):
        self.update_node_details(self._topic_sub_list.value, self._topic_sub_list.options)

    def on_topic_pub_pick(self):
        self.update_node_details(self._topic_pub_list.value, self._topic_pub_list.options)

    def update_lists(self):
        try:
            self.state = self.master.getSystemState()
        except socket.error:
            raise IOError("Unable to communicate with roscore!")

        # TODO(lucasw) having run getSystemState() probably already have this information
        self._topic_data = get_topic_data()
        self._topic_names = sorted(self._topic_data.keys())
        self._topic_list.options = make_options(self._topic_names, "all topics")

        # TODO(lucasw) set self._topic_list.value to 0?
        self.on_topic_pick()

        self._node_names = sorted(rosnode.get_node_names())
        self._node_list.options = make_options(self._node_names, "all nodes")

    def process_event(self, event):
        # Do the key handling for this Frame.
        if isinstance(event, KeyboardEvent):
            if event.key_code in [ord('q'), ord('Q'), Screen.ctrl("c")]:
                rospy.signal_shutdown('user key exit')
                raise StopApplication("User quit")
            if event.key_code in [ord('r'), ord('R')]:
                self.update_lists()

        # Now pass on to lower levels for normal handling of the event.
        return super(TopicFrame, self).process_event(event)


def play(screen, old_scene):
    screen.play([Scene([TopicFrame(screen)], -1)], stop_on_resize=True, start_scene=old_scene)


if __name__ == '__main__':
    rospy.init_node('ros_tui')
    last_scene = None

    # while not rospy.is_shutdown():
    while True:
        try:
            Screen.wrapper(play, catch_interrupt=False, arguments=[last_scene])
            sys.exit(0)
        except ResizeScreenError as rse:
            # print("resize")
            last_scene = rse.scene
