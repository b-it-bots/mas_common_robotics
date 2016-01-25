import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from node_widgets_container import NodeWidgetsContainer

import yaml


class NodeEventsPlugin(Plugin):

    def __init__(self, context):
        super(NodeEventsPlugin, self).__init__(context)
        self.setObjectName('NodeEventsPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-c", "--config", action="store",
                            dest="config_file",
                            help="path to config yaml file",
                            required=True)

        args, unknowns = parser.parse_known_args(context.argv())

        # config file needs to be specified as an argument
        # ex. rqt --standalone rqt_node_events --args -c /path/to/config/file
        yaml_file = args.config_file

        stream = open(yaml_file, "r")
        self.nodes = yaml.load(stream)

        self._widget = NodeWidgetsContainer(context, self.nodes)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resources', 'NodeEventsPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('NodeEventsPluginUi')
