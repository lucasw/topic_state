#!/usr/bin/env python

import sys

# import rosgraph
import rospy
import rostopic
from asciimatics.event import KeyboardEvent
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Divider
from asciimatics.widgets import Frame
from asciimatics.widgets import Label
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
    print(f"subs {len(subs)}")
    for topic in pubs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
            topic_data[name]['subscribers'] = []
        topic_data[name]['publishers'] = topic[2]
    print(f"subs {len(pubs)}")
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


class DemoFrame(Frame):
    def __init__(self, screen):
        super(DemoFrame, self).__init__(
            screen, screen.height, screen.width, has_border=False, name="My Form")

        # Create the (very simple) form layout...
        layout = Layout([1], fill_frame=True)
        self.add_layout(layout)

        # Now populate it with the widgets we want to use.
        self._details = Text()
        self._details.disabled = True
        self._details.custom_colour = "field"

        self._topic_data = get_topic_data()
        self._sorted_names = sorted(self._topic_data.keys())
        topic_names = [(name, i) for i, name in enumerate(self._sorted_names)]

        self._list = ListBox(Widget.FILL_FRAME,
                             topic_names,
                             name="topics",
                             add_scroll_bar=True,
                             on_select=self.select_popup,
                             on_change=self.on_pick)
        layout.add_widget(Label("rostopic list"))
        layout.add_widget(Divider())
        layout.add_widget(self._list)
        layout.add_widget(Divider())
        layout.add_widget(self._details)
        layout.add_widget(Label("Press Enter to select or `q` to quit."))

        # Prepare the Frame for use.
        self.fix()

    def select_popup(self):
        # Just confirm whenever the user actually selects something.
        name = self._sorted_names[self._list.value]
        self._scene.add_effect(PopUpDialog(self._screen, f"You selected: {name}", ["OK"]))

    def on_pick(self):
        # TODO(lucasw) give information on this topic in a right hand column
        new_value = '--'
        if self._list.value is not None:
            name = self._sorted_names[self._list.value]
            new_value = name

        self._details.value = new_value

    def process_event(self, event):
        # Do the key handling for this Frame.
        if isinstance(event, KeyboardEvent):
            if event.key_code in [ord('q'), ord('Q'), Screen.ctrl("c")]:
                rospy.signal_shutdown('user key exit')
                raise StopApplication("User quit")

        # Now pass on to lower levels for normal handling of the event.
        return super(DemoFrame, self).process_event(event)


class RosTui(object):
    def __init__(self):
        # self.list = MultiColumnListBox(Widget.FILL_FRAME)
        # TODO(lucasw) has to be in main thread?
        # self.timer = rospy.Timer(rospy.Duration(0.02), self.update)
        pass


def play(screen, old_scene):
    screen.play([Scene([DemoFrame(screen)], -1)], stop_on_resize=True, start_scene=old_scene)


if __name__ == '__main__':
    rospy.init_node('ros_tui')
    last_scene = None

    node = RosTui()

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
            print("resize")
            last_scene = rse.scene
