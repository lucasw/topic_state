#!/usr/bin/env python

import sys

# import rosgraph
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


class TopicFrame(Frame):
    def __init__(self, screen):
        super(TopicFrame, self).__init__(
            screen, screen.height, screen.width, has_border=False, name="My Form")

        # Create the (very simple) form layout...
        layout = Layout([1, 1], fill_frame=True)
        self.add_layout(layout)

        # Now populate it with the widgets we want to use.
        self._topic_details = Text()
        # self._topic_details.disabled = True
        self._topic_details.custom_colour = "field"

        self._topic_list = ListBox(int(screen.height * 0.5 + 1),
                                   [],
                                   name="topics",
                                   add_scroll_bar=True,
                                   # on_select=self.select_topic_popup,
                                   on_change=self.on_topic_pick)

        self._topic_pub_list = ListBox(int(screen.height * 0.25 - 1),
                                       [],
                                       name="publishers",
                                       add_scroll_bar=True,
                                       # on_select=self.select_popup,
                                       # on_change=self.on_pick)
                                       )

        self._topic_sub_list = ListBox(int(screen.height * 0.25),
                                       [],
                                       name="subscribers",
                                       add_scroll_bar=True,
                                       # on_select=self.select_popup,
                                       # on_change=self.on_pick)
                                       )

        self._node_list = ListBox(int(screen.height * 0.5) - 2,
                                  [],
                                  name="nodes",
                                  add_scroll_bar=True,
                                  # on_select=self.select_popup,
                                  on_change=self.on_node_pick
                                  )

        self._node_info = ListBox(Widget.FILL_FRAME,
                                  [],
                                  name="node_info",
                                  add_scroll_bar=True,
                                  on_change=self.on_node_info_pick)

        # self._node_info.custom_colour = "field"

        # layout.add_widget(Label("rostopic list"))
        # layout.add_widget(Divider())
        column = 0
        layout.add_widget(self._topic_list, column)
        layout.add_widget(Divider(), column)
        layout.add_widget(self._node_list, column)
        # layout.add_widget(Label("Press Enter to select or `q` to quit."), column)

        column = 1
        layout.add_widget(self._topic_details, column)
        layout.add_widget(Divider(), column)
        layout.add_widget(self._topic_pub_list, column)
        layout.add_widget(Divider(), column)
        layout.add_widget(self._topic_sub_list, column)
        layout.add_widget(Divider(), column)
        layout.add_widget(self._node_info, column)

        self.update_lists()

        # Prepare the Frame for use.
        self.fix()

    def select_topic_popup(self):
        # Just confirm whenever the user actually selects something.
        name = self._topic_names[self._topic_list.value]
        self._scene.add_effect(PopUpDialog(self._screen, f"You selected: {name}", ["OK"]))

    def on_topic_pick(self):
        # TODO(lucasw) give information on this topic in a right hand column
        new_value = '--'
        if self._topic_list.value is not None:
            topic_name = self._topic_names[self._topic_list.value]
            topic_type = self._topic_data[topic_name]['type']
            new_value = topic_type

            topic_data = self._topic_data[topic_name]
            subs = sorted(topic_data['subscribers'])
            sub_names = [(name, i) for i, name in enumerate(subs)]
            self._topic_sub_list.options = sub_names

            pubs = sorted(topic_data['publishers'])
            pub_names = [(name, i) for i, name in enumerate(pubs)]
            self._topic_pub_list.options = pub_names

        self._topic_details.value = new_value

    def on_node_pick(self):
        if self._node_list.value is not None:
            node_name = self._node_names[self._node_list.value]
            text = rosnode.get_node_info_description(node_name)
            # text = '\n'.join(text.split('\n'))
            text = text.replace('\n\n', '\n')
            lines = text.split('\n')
            # master = rosgraph.Master('/rosnode')
            self._node_info.options = [(name, i) for i, name in enumerate(lines)]

    def on_node_info_pick(self):
        pass
        # self._topic_details.value = "test"

    def update_lists(self):
        self._topic_data = get_topic_data()
        self._topic_names = sorted(self._topic_data.keys())
        topic_names = [(name, i) for i, name in enumerate(self._topic_names)]
        self._topic_list.options = topic_names

        self.on_topic_pick()

        self._node_names = sorted(rosnode.get_node_names())
        self._node_list.options = [(name, i) for i, name in enumerate(self._node_names)]

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

    # master = rosgraph.Master('/rostopic')
    topic_data = get_topic_data()

    for topic_name in sorted(topic_data.keys()):
        print(topic_name)
        topic = topic_data[topic_name]
        if False:
            print(f"  {topic['type']}")
            pubs = topic['publishers']
            subs = topic['subscribers']
            print(f"  {len(pubs)} {len(subs)}")

    # sys.exit(0)

    # while not rospy.is_shutdown():
    while True:
        try:
            Screen.wrapper(play, catch_interrupt=False, arguments=[last_scene])
            sys.exit(0)
        except ResizeScreenError as rse:
            # print("resize")
            last_scene = rse.scene
