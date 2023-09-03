#! /usr/bin/env python3

import rospy
import time
import csv
import os
import _thread
import pandas as pd
from math import *
from flight_teleop.msg import FlightParameters, FlightTarget, FlightState
from mavros_msgs.msg import BatteryStatus
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.core.window import Window
from kivymd.uix.datatables import MDDataTable
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.tab import MDTabsBase
from kivymd.theming import ThemeManager
from kivy_garden.graph import Graph, LinePlot
from kivy.metrics import dp
from kivy.config import Config
from kivy.clock import Clock

print('code started')                                                       # remove line when done
Config.set('graphics','resizable', False)
Window.size = (720, 480)


kivy_string = '''
#:kivy 2.1.0
#:import NoTransition kivy.uix.screenmanager.NoTransition

MDScreen:
    MDToolbar:
        id: toolbar
        title: 'Application for Monitoring and Controlling the Quadcopter'
        pos_hint: {'top': 1}
        elevation: 15
        left_action_items: [['menu', lambda x: nav_drawer.set_state("open")]]
    MDNavigationLayout:
        id: nav_layout
        ScreenManager:
            id: sm
            transition: NoTransition()
            MDScreen:
                name: 'HomeScreen'
                MDLabel:
                    text: 'Enter the target coordinates'
                    pos_hint: {"center_y": 0.81}
                    halign: 'center'
                MDTextField:
                    id: x_coordinate
                    hint_text: 'x coordinate'
                    pos_hint: {"center_x": 0.325, "center_y": 0.71}
                    size_hint: 0.12, None
                    helper_text_mode: 'persistent'
                    helper_text: ''
                    multiline: False
                    on_text_validate: y_coordinate.focus = True
                    helper_text_color_normal: 1, 0, 0, 1
                    helper_text_color_focus: 1, 0, 0, 1
                    on_text:
                        self.helper_text = ''
                MDTextField:
                    id: y_coordinate
                    hint_text: 'y coordinate'
                    pos_hint: {"center_x": 0.500, "center_y": 0.71}
                    size_hint: 0.12, None
                    helper_text_mode: 'persistent'
                    helper_text: ''
                    multiline: False
                    on_text_validate: z_coordinate.focus = True
                    helper_text_color_normal: 1, 0, 0, 1
                    helper_text_color_focus: 1, 0, 0, 1
                    on_text:
                        self.helper_text = ''
                MDTextField:
                    id: z_coordinate
                    hint_text: 'z coordinate'
                    pos_hint: {"center_x": 0.675, "center_y": 0.71}
                    size_hint: 0.12, None
                    helper_text_mode: 'persistent'
                    helper_text: ''
                    helper_text_color_normal: 1, 0, 0, 1
                    helper_text_color_focus: 1, 0, 0, 1
                    on_text:
                        self.helper_text = ''
                MDRaisedButton:
                    id: go
                    text: 'Send Target'
                    font_size: '17.5'
                    pos_hint: {"center_x": 0.5, "center_y": 0.58}
                    elevation: 10
                    on_press:
                        app.textcheck()
                MDSeparator:
                    pos_hint: {"center_y": 0.5}
                MDLabel:
                    text: 'Current quadcopter state is: -'
                    size_hint_x: 0.9
                    pos_hint: {"center_x": 0.5, "center_y": 0.47}
                    halign: 'left'
                MDLabel:
                    text: 'x :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.40}
                    halign: 'right'
                MDLabel:
                    text: 'y :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.35}
                    halign: 'right'
                MDLabel:
                    text: 'z :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.30}
                    halign: 'right'
                MDLabel:
                    text: 'velocity_x :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.25}
                    halign: 'right'
                MDLabel:
                    text: 'velocity_y :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.20}
                    halign: 'right'
                MDLabel:
                    text: 'velocity_z :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.15}
                    halign: 'right'
                MDLabel:
                    text: 'uptime :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.225, "center_y": 0.10}
                    halign: 'right'
                MDLabel:
                    id : x
                    size_hint_x: 0.15
                    text: 'str(app.x)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.40}
                    halign: 'left'
                MDLabel:
                    id : y
                    size_hint_x: 0.15
                    text: 'str(app.y)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.35}
                    halign: 'left'
                MDLabel:
                    id : z
                    size_hint_x: 0.15
                    text: 'str(app.z)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.30}
                    halign: 'left'
                MDLabel:
                    id : dx
                    size_hint_x: 0.15
                    text: 'str(app.dx)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.25}
                    halign: 'left'
                MDLabel:
                    id : dy
                    size_hint_x: 0.15
                    text: 'str(app.dy)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.20}
                    halign: 'left'
                MDLabel:
                    id : dz
                    size_hint_x: 0.15
                    text: 'str(app.dz)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.15}
                    halign: 'left'
                MDLabel:
                    id : uptime
                    size_hint_x: 0.15
                    text: 'str(uptime)'
                    pos_hint: {"center_x": 0.425, "center_y": 0.10}
                    halign: 'left'
                MDLabel:
                    text: 'yaw :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.40}
                    halign: 'right'
                MDLabel:
                    text: 'pitch :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.35}
                    halign: 'right'
                MDLabel:
                    text: 'roll :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.30}
                    halign: 'right'
                MDLabel:
                    text: 'yaw_rate :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.25}
                    halign: 'right'
                MDLabel:
                    text: 'pitch_rate :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.20}
                    halign: 'right'
                MDLabel:
                    text: 'roll_rate :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.15}
                    halign: 'right'                
                MDLabel:
                    text: 'battery :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.575, "center_y": 0.10}
                    halign: 'right'
                MDLabel:
                    id : psi
                    size_hint_x: 0.15
                    text: 'str(app.psi)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.40}
                    halign: 'left'
                MDLabel:
                    id : theta
                    size_hint_x: 0.15
                    text: 'str(app.theta)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.35}
                    halign: 'left'
                MDLabel:
                    id : phi
                    size_hint_x: 0.15
                    text: 'str(app.phi)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.30}
                    halign: 'left'
                MDLabel:
                    id : dpsi
                    size_hint_x: 0.15
                    text: 'str(app.dpsi)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.25}
                    halign: 'left'
                MDLabel:
                    id : dtheta
                    size_hint_x: 0.15
                    text: 'str(app.dtheta)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.20}
                    halign: 'left'
                MDLabel:
                    id : dphi
                    size_hint_x: 0.15
                    text: 'str(app.dphi)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.15}
                    halign: 'left'
                MDLabel:
                    id : battery
                    size_hint_x: 0.15
                    text: 'str(battery)'
                    pos_hint: {"center_x": 0.775, "center_y": 0.10}
                    halign: 'left'
            MDScreen:
                name: 'QuadPose'
                MDTabs:
                    pos_hint: {"center_y": 0.3650}
                    Tab:
                        title: 'LIVE DATA'
                        MDLabel:
                            text: 'Autoflight Mode :'
                            size_hint_x: 0.40
                            pos_hint: {"center_x": 0.20, "center_y": 0.95}
                            halign: 'right'
                        MDLabel:
                            id: flight_mode
                            text: 'Landed'
                            color: 0,0.5,0,1
                            size_hint_x: 0.30
                            pos_hint: {"center_x": 0.60, "center_y": 0.95}
                            halign: 'left'
                        MDLabel:
                            text: 'Status :'
                            size_hint_x: 0.40
                            pos_hint: {"center_x": 0.20, "center_y": 0.90}
                            halign: 'right'
                        MDLabel:
                            id: status
                            text: 'Chasing Target'
                            color: 0,0.5,0,1
                            size_hint_x: 0.30
                            pos_hint: {"center_x": 0.60, "center_y": 0.90}
                            halign: 'left'
                        MDLabel:
                            text: 'Target :'
                            size_hint_x: 0.40
                            pos_hint: {"center_x": 0.20, "center_y": 0.85}
                            halign: 'right'
                        MDLabel:
                            id: tar_coordinates
                            text: 'app.tar_coordinates'
                            size_hint_x: 0.30
                            pos_hint: {"center_x": 0.60, "center_y": 0.85}
                            halign: 'left'
                        MDLabel:
                            text: 'Target Distance :'
                            size_hint_x: 0.40
                            pos_hint: {"center_x": 0.20, "center_y": 0.80}
                            halign: 'right'
                        MDLabel:
                            id: tar_distance
                            text: 'app.tar_distance'
                            size_hint_x: 0.30
                            pos_hint: {"center_x": 0.60, "center_y": 0.80}
                            halign: 'left'
                        MDSeparator:
                            pos_hint: {"center_y": 0.76}
                        MDLabel:
                            text: 'Current quadcopter state is: -'
                            size_hint_x: 0.9
                            pos_hint: {"center_x": 0.5, "center_y": 0.72}
                            halign: 'left'
                        MDLabel:
                            text: 'x :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.65}
                            halign: 'right'
                        MDLabel:
                            text: 'y :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.60}
                            halign: 'right'
                        MDLabel:
                            text: 'z :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.55}
                            halign: 'right'
                        MDLabel:
                            text: 'velocity_x :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.50}
                            halign: 'right'
                        MDLabel:
                            text: 'velocity_y :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.45}
                            halign: 'right'
                        MDLabel:
                            text: 'velocity_z :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.40}
                            halign: 'right'
                        MDLabel:
                            text: 'uptime :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.225, "center_y": 0.35}
                            halign: 'right'
                        MDLabel:
                            id : x_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.65}
                            halign: 'left'
                        MDLabel:
                            id : y_2
                            size_hint_x: 0.15
                            text: '-0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.60}
                            halign: 'left'
                        MDLabel:
                            id : z_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.55}
                            halign: 'left'
                        MDLabel:
                            id : dx_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.50}
                            halign: 'left'
                        MDLabel:
                            id : dy_2
                            size_hint_x: 0.15
                            text: '-0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.45}
                            halign: 'left'
                        MDLabel:
                            id : dz_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.40}
                            halign: 'left'
                        MDLabel:
                            id : uptime_2
                            size_hint_x: 0.15
                            text: '0000.000'
                            pos_hint: {"center_x": 0.425, "center_y": 0.35}
                            halign: 'left'
                        MDLabel:
                            text: 'yaw :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.65}
                            halign: 'right'
                        MDLabel:
                            text: 'pitch :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.60}
                            halign: 'right'
                        MDLabel:
                            text: 'roll :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.55}
                            halign: 'right'
                        MDLabel:
                            text: 'yaw_rate :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.50}
                            halign: 'right'
                        MDLabel:
                            text: 'pitch_rate :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.45}
                            halign: 'right'
                        MDLabel:
                            text: 'roll_rate :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.40}
                            halign: 'right'                
                        MDLabel:
                            text: 'battery :'
                            size_hint_x: 0.15
                            pos_hint: {"center_x": 0.575, "center_y": 0.35}
                            halign: 'right'
                        MDLabel:
                            id : psi_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.65}
                            halign: 'left'
                        MDLabel:
                            id : theta_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.60}
                            halign: 'left'
                        MDLabel:
                            id : phi_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.55}
                            halign: 'left'
                        MDLabel:
                            id : dpsi_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.50}
                            halign: 'left'
                        MDLabel:
                            id : dtheta_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.45}
                            halign: 'left'
                        MDLabel:
                            id : dphi_2
                            size_hint_x: 0.15
                            text: '0.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.40}
                            halign: 'left'
                        MDLabel:
                            id : battery_2
                            size_hint_x: 0.15
                            text: '00.000'
                            pos_hint: {"center_x": 0.775, "center_y": 0.35}
                            halign: 'left'
                        MDRaisedButton:
                            text: 'Enter another target'
                            pos_hint: {"center_x": 0.35, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'HomeScreen'
                        MDRaisedButton:
                            text: 'Target History'
                            pos_hint: {"center_x": 0.65, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'TargetHistory'
                    Tab:
                        title: 'LIVE PLOTS'
                        MDScreen:
                            id: plots_screen
                            size_hint: 0.95, 0.675
                            pos_hint: {"center_x": 0.5, "center_y": 0.64}
                        MDRaisedButton:
                            text: 'Enter another target'
                            pos_hint: {"center_x": 0.35, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'HomeScreen'
                        MDRaisedButton:
                            text: 'Target History'
                            pos_hint: {"center_x": 0.65, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'TargetHistory'
                    Tab:
                        title: 'ORIENTATION'
                        MDScreen:
                            id: orientation_screen
                            size_hint: 0.95, 0.675
                            pos_hint: {"center_x": 0.5, "center_y": 0.64}
                        MDRaisedButton:
                            text: 'Enter another target'
                            pos_hint: {"center_x": 0.35, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'HomeScreen'
                        MDRaisedButton:
                            text: 'Target History'
                            pos_hint: {"center_x": 0.65, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'TargetHistory'
                    Tab:
                        title: 'FULL PLOTS'
                        MDScreen:
                            id: fullplots_screen
                            size_hint: 0.95, 0.675
                            pos_hint: {"center_x": 0.5, "center_y": 0.64}
                        MDRaisedButton:
                            text: 'Enter another target'
                            pos_hint: {"center_x": 0.35, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'HomeScreen'
                        MDRaisedButton:
                            text: 'Target History'
                            pos_hint: {"center_x": 0.65, "center_y": 0.235}
                            elevation: 10
                            size_hint: None, None
                            width: root.width*0.2
                            on_press: sm.current = 'TargetHistory'
            MDScreen:
                name: 'TargetHistory'
                id: 'history_screen'
                MDLabel:
                    text: 'Target history-coordinates are as follows: -'
                    size_hint_x: 0.5
                    pos_hint: {"center_x": 0.3, "center_y": 0.80}
                    halign: 'left'
                MDRaisedButton:
                    text: 'Enter another target'
                    pos_hint: {"center_x": 0.35, "center_y": 0.075}
                    elevation: 10
                    size_hint: None, None
                    width: root.width*0.2
                    on_press: sm.current = 'HomeScreen'
                MDRaisedButton:
                    text: 'Exit'
                    pos_hint: {"center_x": 0.65, "center_y": 0.075}
                    elevation: 10
                    size_hint: None, None
                    width: root.width*0.2
                    on_press: app.stop()
            MDScreen:
                name: 'Settings'
                MDLabel:
                    text: 'Flight Settings'
                    font_style: 'H4'
                    size_hint_x: 0.95
                    pos_hint: {"center_x": 0.5, "center_y": 0.82}
                    halign: 'left'
                MDLabel:
                    text: '(Warning: setting wrong flight parameters can result into an instant flight unstability issue.)'
                    theme_text_color: 'Error'
                    size_hint_x: 0.95
                    pos_hint: {"center_x": 0.5, "center_y": 0.75}
                    halign: 'left'
                MDSeparator:
                    pos_hint: {"center_y": 0.71}
                MDSeparator:
                    orientation: 'vertical'
                    size_hint_y: 0.56
                    pos_hint: {"center_x": 0.65, "center_y": 0.43}
                MDSeparator:
                    pos_hint: {"center_y": 0.15}
                MDSeparator:
                    orientation: 'vertical'
                    size_hint_y: 0.23
                    pos_hint: {"center_x": 0.325, "center_y": 0.365}
                MDSeparator:
                    orientation: 'horizontal'
                    size_hint_x: 0.55
                    pos_hint: {"center_x": 0.325, "center_y": 0.480}
                MDLabel:
                    text: 'Gain Settings'
                    font_style: 'H5'
                    theme_text_color: 'Secondary'
                    size_hint_x: 0.25
                    pos_hint: {"center_x": 0.175, "center_y": 0.68}
                    halign: 'left'
                MDLabel:
                    text: 'Mass Settings'
                    font_style: 'H5'
                    theme_text_color: 'Secondary'
                    size_hint_x: 0.25
                    pos_hint: {"center_x": 0.825, "center_y": 0.68}
                    halign: 'left'
                MDLabel:
                    text: 'Kpt :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.075, "center_y": 0.600}
                    halign: 'right'
                MDLabel:
                    text: 'Kit :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.075, "center_y": 0.525}
                    halign: 'right'
                MDLabel:
                    text: 'Kpx :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.075, "center_y": 0.435}
                    halign: 'right'
                MDLabel:
                    text: 'Kix :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.075, "center_y": 0.360}
                    halign: 'right'
                MDLabel:
                    text: 'Kdx :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.075, "center_y": 0.285}
                    halign: 'right'
                MDLabel:
                    text: 'Kdt :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.35, "center_y": 0.600}
                    halign: 'right'
                MDLabel:
                    text: 'Kwt :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.35, "center_y": 0.525}
                    halign: 'right'                    
                MDLabel:
                    text: 'Kpy :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.35, "center_y": 0.435}
                    halign: 'right'
                MDLabel:
                    text: 'Kiy :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.35, "center_y": 0.360}
                    halign: 'right'
                MDLabel:
                    text: 'Kdy :'
                    size_hint_x: 0.1
                    pos_hint: {"center_x": 0.35, "center_y": 0.285}
                    halign: 'right'
                MDLabel:
                    text: 'Takeoff Clearance :'
                    size_hint_x: 0.25
                    pos_hint: {"center_x": 0.15, "center_y": 0.20}
                    halign: 'right'
                MDTextFieldRect:
                    id: kpt
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kpt}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kpt.text = kpt.text; kit.focus = True
                    pos_hint: {"center_x": 0.205, "center_y": 0.600}
                MDTextFieldRect:
                    id: kit
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kit}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kit.text = kit.text; kdt.focus = True
                    pos_hint: {"center_x": 0.205, "center_y": 0.525}
                MDTextFieldRect:
                    id: kpx
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kpx}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kpx.text = kpx.text; kix.focus = True
                    pos_hint: {"center_x": 0.205, "center_y": 0.435}
                MDTextFieldRect:
                    id: kix
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kix}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kix.text = kix.text; kdx.focus = True
                    pos_hint: {"center_x": 0.205, "center_y": 0.360}
                MDTextFieldRect:
                    id: kdx
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kdx}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kdx.text = kdx.text; kpy.focus = True
                    pos_hint: {"center_x": 0.205, "center_y": 0.285}
                MDTextFieldRect:
                    id: kdt
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kdt}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kdt.text = kdt.text; kwt.focus = True
                    pos_hint: {"center_x": 0.48, "center_y": 0.600}
                MDTextFieldRect:
                    id: kwt
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kwt}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kwt.text = kwt.text; kpx.focus = True
                    pos_hint: {"center_x": 0.48, "center_y": 0.525}
                MDTextFieldRect:
                    id: kpy
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kpy}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kpy.text = kpy.text; kiy.focus = True
                    pos_hint: {"center_x": 0.48, "center_y": 0.435}
                MDTextFieldRect:
                    id: kiy
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kiy}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kiy.text = kiy.text; kdy.focus = True
                    pos_hint: {"center_x": 0.48, "center_y": 0.360}
                MDTextFieldRect:
                    id: kdy
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_Kdy}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: kdy.text = kdy.text; clearance.focus = True
                    pos_hint: {"center_x": 0.48, "center_y": 0.285}
                MDTextFieldRect:
                    id: clearance
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_takeoff_clearance}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: clearance.text = clearance.text
                    pos_hint: {"center_x": 0.35, "center_y": 0.200}
                MDLabel:
                    text: 'Gross Mass :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.725, "center_y": 0.55}
                    halign: 'right'
                MDLabel:
                    text: 'Gravity :'
                    size_hint_x: 0.15
                    pos_hint: {"center_x": 0.725, "center_y": 0.45}
                    halign: 'right'
                MDTextFieldRect:
                    id: mass
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_m}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: mass.text = mass.text; gravity.focus = True
                    pos_hint: {"center_x": 0.88, "center_y": 0.55}
                MDTextFieldRect:
                    id: gravity
                    size_hint_x: 0.12
                    size_hint_y: 0.065
                    hint_text: f'{app.set_g}'
                    input_filter: 'float'
                    multiline: False
                    on_text_validate: gravity.text = gravity.text
                    pos_hint: {"center_x": 0.88, "center_y": 0.45}
                MDLabel:
                    text: 'All parameters are in SI Units'
                    theme_text_color: 'Hint'
                    size_hint_x: 0.30
                    pos_hint: {"center_x": 0.825, "center_y": 0.20}
                    halign: 'center'                
                MDRaisedButton:
                    text: 'Home Page'
                    pos_hint: {"center_x": 0.35, "center_y": 0.075}
                    elevation: 10
                    size_hint: None, None
                    width: root.width*0.2
                    on_press: sm.current = 'HomeScreen'
                MDRaisedButton:
                    text: 'Apply Changes'
                    pos_hint: {"center_x": 0.65, "center_y": 0.075}
                    elevation: 10
                    size_hint: None, None
                    width: root.width*0.2
                    on_press:
                        app.set_parameters()
                        sm.current = 'QuadPose'
            MDScreen:
                name: 'AboutApp'
                MDLabel:
                    text: 'Application created by Abhinav Kumar'
                    size_hint_x: 0.75
                    pos_hint: {"center_x": 0.5, "center_y": 0.75}
                    halign: 'center'
        MDNavigationDrawer:
            id: nav_drawer
            orientation: 'vertical'
            padding: 10
            MDLabel:
                text: 'Command ROS-enabled Quadcopter'
                theme_text_color: 'Secondary'
                size_hint_y: 0.1
            MDSeparator:
            MDSeparator:
            Widget:
                size_hint: None, 0.005
            MDSeparator:
            MDSeparator:
            ScrollView:
                padding: 10
                MDList:
                    padding: 10
                    OneLineListItem:
                        text: 'Plots'
                        on_press: sm.current = 'QuadPose'; nav_drawer.set_state("close")
                    OneLineListItem:
                        text: 'Target History'
                        on_press: sm.current = 'TargetHistory'; nav_drawer.set_state("close")
                    OneLineListItem:
                        text: 'Set Flight Parameters'
                        on_press: sm.current = 'Settings'; nav_drawer.set_state("close")
                    OneLineListItem:
                        text: 'About Application'
                        on_press: sm.current = 'AboutApp'; nav_drawer.set_state("close")
                    OneLineListItem:
                        text: 'Exit'
                        on_press: app.stop()
'''


def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False

s_no = 0
start_time = time.perf_counter()
data_file = f'/home/abhinav/Documents/Data/Flightdata/flightdata_{time.strftime("%d-%m-%Y_%H:%M:%S", time.localtime())}.csv'

headers = ['S.No.', 'uptime', 'battery', 'x', 'y', 'z', 'psi', 'theta', 'phi', 'dx', 'dy', 'dz', 'dpsi', 'dtheta', 'dphi', 'thrust', 'target_x', 'target_y', 'target_z']

with open(data_file, 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames = headers)
    csv_writer.writeheader()


class Tab(MDFloatLayout, MDTabsBase):
    '''Class implementing content for a tab.'''


class Teleop_GUI(MDApp):
    print('App class started')
    theme_cls = ThemeManager()

    param_msg    = FlightParameters()
    target_msg   = FlightTarget()
    qc_state_msg = FlightState()
    battery_msg  = BatteryStatus()

    target_msg.header.seq = 0
    target_input_open     = False
    
    set_m   = 1.275
    set_g   = 9.81

    set_Kpt = 0.1   # 0.1
    set_Kdt = 0.10   # 0.1
    set_Kit = 0.2  # 0.2
    set_Kwt = 0.043  # 0.04327

    set_Kpx = 0.05   # 0.1
    set_Kix = 0.001
    set_Kdx = 0.05   # 0.2

    set_Kpy = set_Kpx
    set_Kiy = set_Kix
    set_Kdy = set_Kdx

    set_takeoff_clearance = 0.5


    def __init__(self, **kwargs):
        self.title = "node: '/teleop_GUI'"
        super().__init__(**kwargs)


    def qc_state_cb(self, state):
        self.qc_state_msg = state

    def battery_cb(self, battery_voltage):
        self.battery_msg = battery_voltage

    def quat2eul(self, q0, q1, q2, q3):
        phi    = atan2(2*(q0*q1 + q2*q3), (1 - 2*(q1**2 + q2**2)))
        theta  = asin(2*(q0*q2 - q3*q1))
        psi    = atan2(2*(q0*q3 + q1*q2), (1 - 2*(q2**2 + q3**2)))
        return psi, theta, phi


    def set_parameters(self):
        print('set_parameters started')
        if self.root.ids.mass.text == '':
            self.set_m = self.set_m
        else:
            self.set_m   = float(self.root.ids.mass.text)
        if self.root.ids.gravity.text == '':
            self.set_g = self.set_g
        else:
            self.set_g   = float(self.root.ids.gravity.text)

        if self.root.ids.kpt.text == '':
            self.set_Kpt = self.set_Kpt
        else:
            self.set_Kpt = float(self.root.ids.kpt.text)
        if self.root.ids.kit.text == '':
            self.set_Kit = self.set_Kit
        else:
            self.set_Kit = float(self.root.ids.kit.text)
        if self.root.ids.kdt.text == '':
            self.set_Kdt = self.set_Kdt
        else:
            self.set_Kdt = float(self.root.ids.kdt.text)
        if self.root.ids.kwt.text == '':
            self.set_Kwt = self.set_Kwt
        else:
            self.set_Kwt = float(self.root.ids.kwt.text)

        if self.root.ids.kpx.text == '':
            self.set_Kpx = self.set_Kpx
        else:
            self.set_Kpx = float(self.root.ids.kpx.text)
        if self.root.ids.kix.text == '':
            self.set_Kix = self.set_Kix
        else:
            self.set_Kix = float(self.root.ids.kix.text)
        if self.root.ids.kdx.text == '':
            self.set_Kdx = self.set_Kdx
        else:
            self.set_Kdx = float(self.root.ids.kdx.text)

        if self.root.ids.kpy.text == '':
            self.set_Kpy = self.set_Kpy
        else:
            self.set_Kpy = float(self.root.ids.kpy.text)
        if self.root.ids.kiy.text == '':
            self.set_Kiy = self.set_Kiy
        else:
            self.set_Kiy = float(self.root.ids.kiy.text)
        if self.root.ids.kdy.text == '':
            self.set_Kdy = self.set_Kdy
        else:
            self.set_Kdy = float(self.root.ids.kdy.text)

        if self.root.ids.clearance.text == '':
            self.set_takeoff_clearance = self.set_takeoff_clearance
        else:
            self.set_takeoff_clearance = float(self.root.ids.clearance.text)


    def publish_parameters(self):
        print('publish_parameters started')
        while not rospy.is_shutdown():
            self.param_msg.m   = self.set_m
            self.param_msg.g   = self.set_g

            self.param_msg.Kpt = self.set_Kpt
            self.param_msg.Kit = self.set_Kit
            self.param_msg.Kdt = self.set_Kdt
            self.param_msg.Kwt = self.set_Kwt

            self.param_msg.Kpx = self.set_Kpx
            self.param_msg.Kix = self.set_Kix
            self.param_msg.Kdx = self.set_Kdx

            self.param_msg.Kpy = self.set_Kpy
            self.param_msg.Kiy = self.set_Kiy
            self.param_msg.Kdy = self.set_Kdy

            self.param_msg.takeoff_clearance = self.set_takeoff_clearance

            self.param_pub.publish(self.param_msg)

            self.rate.sleep()


    def set_target(self):
        print('set_target started')
        self.target_x = float(self.root.ids.x_coordinate.text)
        self.target_y = float(self.root.ids.y_coordinate.text)
        self.target_z = float(self.root.ids.z_coordinate.text)
    

    def publish_target(self):  
        print('publish_target started')
        while not rospy.is_shutdown():
            self.target_msg.x               = self.target_x
            self.target_msg.y               = self.target_y
            self.target_msg.z               = self.target_z
            self.target_msg.yaw             = 0.0
            self.target_msg.header.frame_id = "Target-Pose to Quadcopter"

            self.target_msg.header.seq      = self.target_msg.header.seq + 1
            self.target_msg.header.stamp    = rospy.Time.now()

            self.target_pub.publish(self.target_msg)

            self.rate.sleep()


    def build(self):
        global kivy_string
        print('build started')
        if __name__ == "__main__":
            self.theme_cls.primary_palette = "Teal"
            rospy.init_node("teleop_GUI")

            self.param_pub  = rospy.Publisher("picopter/parameters", FlightParameters, queue_size = 10)
            self.target_pub = rospy.Publisher("picopter/target", FlightTarget, queue_size = 10)
            
            print('node created')

            self.rate        = rospy.Rate(20)

            while rospy.is_shutdown():
                rospy.sleep(0.03)
                break

            time.sleep(0.75)

            self.input_x = 0.0
            self.input_y = 0.0
            self.input_z = 0.0
            
            try:
                self.target_x = self.qc_state_msg.body_pose.position.x
                self.target_y = self.qc_state_msg.body_pose.position.y
                self.target_z = self.set_takeoff_clearance
            except:
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = self.set_takeoff_clearance

            _thread.start_new_thread(self.publish_parameters, ())                                            # may need to give self as input
            _thread.start_new_thread(self.publish_target, ())                                                # may need to give self as input

            self.root = Builder.load_string(f'{kivy_string}')

            _thread.start_new_thread(self.update, ())

            self.history_table = MDDataTable(
                column_data =[
                    ("Target No."  , dp(25)),
                    ("x coordinate", dp(25)),
                    ("y coordinate", dp(25)),
                    ("z coordinate", dp(25)),
                ],
                # row_data=[("@startup", str(self.spawn_x), str(self.spawn_y), str(self.spawn_z))],
                size_hint = (0.75, 0.6),
                pos_hint = {"center_x": 0.5, "center_y": 0.45},
                elevation = 10,
                use_pagination=True,
                anchor_x = "center"
            )
            self.root.ids.sm.get_screen('TargetHistory').add_widget(self.history_table)

            print('line 439')

            self.graph_1      = Graph(xmin = 0, xmax = 3, ymin = 0, ymax = 12, xlabel='Uptime', ylabel='Unit Dis. (x: Blue, y: Green, dis: Red)', x_grid = False, y_grid = False, padding = 7, \
                                        x_ticks_major = 1, y_ticks_major = 3, y_grid_label = True, x_grid_label = False)
            self.graph_1.background_color = [0.1529, 0.1569, 0.1333, 1]
            self.graph_1.border_color     = [1, 1, 1, 1]
            self.graph_1.tick_color       = [1, 1, 1, 1]
            self.plot_x           = LinePlot(color = [0, 0, 1, 1], line_width = 1.25)
            self.plot_y           = LinePlot(color = [0, 1, 0, 1], line_width = 1.25)
            self.plot_d           = LinePlot(color = [1, 0, 0, 1], line_width = 1.25)
            self.plots_screen = self.root.ids.plots_screen
            self.plots_screen.add_widget(self.graph_1)
            self.graph_2      = Graph(xmin = 0, xmax = 120, ymin = -3.14, ymax = 3.14, xlabel='Steps', ylabel='Turtle Orientation (radians)', x_grid = False, y_grid = False, padding = 7, \
                                        x_ticks_major = 500, y_ticks_major = 1.57, y_grid_label = True, x_grid_label = False)
            self.graph_2.background_color = [0.1529, 0.1569, 0.1333, 1]
            self.graph_2.border_color     = [1, 1, 1, 1]
            self.graph_2.tick_color       = [1, 1, 1, 1]
            self.orient           = LinePlot(color = [0, 0, 1, 1], line_width = 1.25)
            self.orientation_screen = self.root.ids.orientation_screen
            self.orientation_screen.add_widget(self.graph_2)
            self.graph_3      = Graph(xmin = 0, xmax = 120, ymin = 0, ymax = 12, xlabel='Steps', ylabel='Unit Dis. (x: Blue, y: Green, dis: Red)', x_grid = False, y_grid = False, padding = 7, \
                                        x_ticks_major = 500, y_ticks_major = 3, y_grid_label = True, x_grid_label = False)
            self.graph_3.background_color = [0.1529, 0.1569, 0.1333, 1]
            self.graph_3.border_color     = [1, 1, 1, 1]
            self.graph_3.tick_color       = [1, 1, 1, 1]
            self.fullplot_x       = LinePlot(color = [0, 0, 1, 1], line_width = 1.25)
            self.fullplot_y       = LinePlot(color = [0, 1, 0, 1], line_width = 1.25)
            self.fullplot_d       = LinePlot(color = [1, 0, 0, 1], line_width = 1.25)
            self.fullplots_screen = self.root.ids.fullplots_screen
            self.fullplots_screen.add_widget(self.graph_3)

            print('build ended')                                                # remove line when done


    def on_tab_switch(self, instance_tabs, instance_tab, instance_tab_label, tab_text):
        pass


    def addtarget(self, dt):
        sr_no = self.history_table.row_data[-1][0]
        if sr_no == "@startup":
            sr_no = 0
        else:
            sr_no = int(sr_no)
            self.history_table.update_row_data
        self.history_table.row_data.insert(len(self.history_table.row_data), (str(sr_no + 1), str(round(float(self.root.ids.x_coordinate.text), 3)), str(round(float(self.root.ids.y_coordinate.text), 3)), str(round(float(self.root.ids.z_coordinate.text), 3))))
        self.root.ids.x_coordinate.text = ''
        self.root.ids.y_coordinate.text = ''
        self.root.ids.z_coordinate.text = ''


    def textcheck(self):
        if not isfloat(self.root.ids.x_coordinate.text):
            self.root.ids.x_coordinate.helper_text = 'Invalid Input'
        elif not (float(self.root.ids.x_coordinate.text) <= 1.25) & (float(self.root.ids.x_coordinate.text) >= -1.25):
            self.root.ids.x_coordinate.helper_text = 'Input from -1.25 to 1.25'
        if not isfloat(self.root.ids.y_coordinate.text):
            self.root.ids.y_coordinate.helper_text = 'Invalid Input'
        elif not (float(self.root.ids.y_coordinate.text) <= 1.25) & (float(self.root.ids.y_coordinate.text) >= -1.25):
            self.root.ids.y_coordinate.helper_text = 'Input from -1.25 to 1.25'
        if not isfloat(self.root.ids.z_coordinate.text):
            self.root.ids.z_coordinate.helper_text = 'Invalid Input'
        elif not (float(self.root.ids.y_coordinate.text) <= 1.75) & (float(self.root.ids.z_coordinate.text) >= 0.3):
            self.root.ids.z_coordinate.helper_text = 'Input from 0.3 to 1.75'

        if (self.root.ids.x_coordinate.helper_text == '') & (self.root.ids.y_coordinate.helper_text == '') & (self.root.ids.z_coordinate.helper_text == ''):
            inp_x = float(self.root.ids.x_coordinate.text)
            inp_y = float(self.root.ids.y_coordinate.text)
            inp_z = float(self.root.ids.z_coordinate.text)
            if ((round(inp_x,2) == round(self.input_x,2)) & (round(inp_y,2) == round(self.input_y,2)) & (round(inp_z,2) == round(self.input_z,2))):                         # Need Improvement
                print('\nW: Quadcopter is already at this position. Please provide enough distance.\n')
                # popup needed to show this message
            else:
                print(f'Target received : ({self.root.ids.x_coordinate.text}, {self.root.ids.y_coordinate.text}, {self.root.ids.z_coordinate.text})')
                self.input_x = inp_x
                self.input_y = inp_y
                self.input_z = inp_z

                # self.plot_x.points      = []
                # self.plot_y.points      = []
                # self.plot_d.points      = []
                # self.orient.points      = []
                # self.fullplot_x.points  = []
                # self.fullplot_y.points  = []
                # self.fullplot_d.points  = []

                # self.graph_1.add_plot(self.plot_x)
                # self.graph_1.add_plot(self.plot_y)
                # self.graph_1.add_plot(self.plot_d)
                # self.graph_2.add_plot(self.orient)
                # self.graph_3.add_plot(self.fullplot_x)
                # self.graph_3.add_plot(self.fullplot_y)
                # self.graph_3.add_plot(self.fullplot_d)

                self.root.ids.sm.current = 'QuadPose'
                Clock.schedule_once(self.addtarget, 0)

                self.root.ids.tar_coordinates.text = f'({self.root.ids.x_coordinate.text}, {self.root.ids.y_coordinate.text}, {self.root.ids.z_coordinate.text})'

                _thread.start_new_thread(self.set_target, ())


    def csv_update(self):
        global s_no, data_file, headers
        print('update started')
        while not rospy.is_shutdown():
            if __name__ == "__main__":
                s_no = s_no + 1

                psi, theta, phi = self.quat2eul(self.qc_state_msg.body_pose.orientation.w, self.qc_state_msg.body_pose.orientation.x, self.qc_state_msg.body_pose.orientation.y, self.qc_state_msg.body_pose.orientation.z)

                with open(data_file, 'a') as csv_file:
                    csv_writer = csv.DictWriter(csv_file, fieldnames = headers)

                    row_data = {
                        'S.No.'                     : s_no,
                        'uptime'                    : self.qc_state_msg.flight_duration,
                        'battery'                   : self.battery_msg.voltage,
                        'x'                         : self.qc_state_msg.body_pose.position.x,
                        'y'                         : self.qc_state_msg.body_pose.position.y,
                        'z'                         : self.qc_state_msg.body_pose.position.z,
                        'psi'                       : psi,
                        'theta'                     : theta,
                        'phi'                       : phi,
                        'dx'                        : self.qc_state_msg.body_rate.linear.x,
                        'dy'                        : self.qc_state_msg.body_rate.linear.y,
                        'dz'                        : self.qc_state_msg.body_rate.linear.z,
                        'dpsi'                      : self.qc_state_msg.body_rate.angular.z,
                        'dtheta'                    : self.qc_state_msg.body_rate.angular.y,
                        'dphi'                      : self.qc_state_msg.body_rate.angular.x,
                        'thrust'                    : self.qc_state_msg.thrust,
                        'target_x'                  : self.target_x,
                        'target_y'                  : self.target_y,
                        'target_z'                  : self.target_z
                    }

                    csv_writer.writerow(row_data)

                # df   = pd.read_csv(f'{data_file}')

                # self.xp   = df['uptime'].to_list()
                # self.yp1  = df['x coordinate'].to_list()
                # self.yp2  = df['y coordinate'].to_list()
                # self.yp3  = df['theta'].to_list()
                # self.yp4  = df['target distance'].to_list()

                # self.plot_x.points      = [(self.xp[i], self.yp1[i]) for i in range(len(self.xp))]
                # self.plot_y.points      = [(self.xp[i], self.yp2[i]) for i in range(len(self.xp))]
                # self.plot_d.points      = [(self.xp[i], self.yp4[i]) for i in range(len(self.xp))]
                # self.orient.points      = [(i, self.yp3[i]) for i in range(len(self.xp))]
                # self.fullplot_x.points  = [(i, self.yp1[i]) for i in range(len(self.xp))]
                # self.fullplot_y.points  = [(i, self.yp2[i]) for i in range(len(self.xp))]
                # self.fullplot_d.points  = [(i, self.yp4[i]) for i in range(len(self.xp))]

                # self.graph_1.add_plot(self.plot_x)
                # self.graph_1.add_plot(self.plot_y)
                # self.graph_1.add_plot(self.plot_d)
                # self.graph_2.add_plot(self.orient)
                # self.graph_3.add_plot(self.fullplot_x)
                # self.graph_3.add_plot(self.fullplot_y)
                # self.graph_3.add_plot(self.fullplot_d)

                # self.graph_1.xmax = max(self.xp)
                # self.graph_1.xmin = max(self.xp) - 10
                # self.graph_2.xmax = len(self.xp)
                # self.graph_3.xmax = len(self.xp)

                self.rate.sleep()


    def update(self):
        global s_no, data_file, headers
        print('update started')
        if __name__ == "__main__":
            while True:
                try:
                    node_up = rospy.wait_for_message("picopter/state", FlightState, timeout = 10)
                    print('communication established to the quadcopter')
                    break
                except:
                    print('waiting for the response from quadcopter...')

            _thread.start_new_thread(self.csv_update, ())

            self.qc_state_sub = rospy.Subscriber("picopter/state", FlightState, callback = self.qc_state_cb)
            print('line 1231')
            self.battery_sub  = rospy.Subscriber("mavros/battery", BatteryStatus, callback = self.battery_cb)
            print('line 1233')

            self.spawn_x  = node_up.body_pose.position.x
            self.spawn_y  = node_up.body_pose.position.y
            self.spawn_z  = node_up.body_pose.position.z
            
            self.target_x = node_up.body_pose.position.x
            self.target_y = node_up.body_pose.position.y
            self.target_z = self.set_takeoff_clearance

            self.history_table.row_data.insert(len(self.history_table.row_data), (('@startup'), str(round(self.spawn_x, 3)), str(round(self.spawn_y, 3)), str(round(self.spawn_z, 3))))

            while not rospy.is_shutdown():
                psi, theta, phi = self.quat2eul(self.qc_state_msg.body_pose.orientation.w, self.qc_state_msg.body_pose.orientation.x, self.qc_state_msg.body_pose.orientation.y, self.qc_state_msg.body_pose.orientation.z)

                print(f'''\n\n\n\n\n\n\n
        -------------------------------------------

        Quadcopter State: -

        X       : {round(self.qc_state_msg.body_pose.position.x, 3)}
        Y       : {round(self.qc_state_msg.body_pose.position.y, 3)}
        Z       : {round(self.qc_state_msg.body_pose.position.z, 3)}
        dX      : {round(  self.qc_state_msg.body_rate.linear.x, 3)}
        dY      : {round(  self.qc_state_msg.body_rate.linear.y, 3)}
        dZ      : {round(  self.qc_state_msg.body_rate.linear.z, 3)}
        Psi     : {round(                              psi, 3)}
        Theta   : {round(                            theta, 3)}
        Phi     : {round(                              phi, 3)}
        dPsi    : {round( self.qc_state_msg.body_rate.angular.z, 3)}
        dTheta  : {round( self.qc_state_msg.body_rate.angular.y, 3)}
        dPhi    : {round( self.qc_state_msg.body_rate.angular.x, 3)}
        Thrust  : {round(              self.qc_state_msg.thrust, 3)}
        Mode    : {self.qc_state_msg.offboard_mode}
        Flytime : {round(     self.qc_state_msg.flight_duration, 3)}

        -------------------------------------------
        ''')

                self.root.ids.x.text         = str(round(self.qc_state_msg.body_pose.position.x, 3))
                self.root.ids.y.text         = str(round(self.qc_state_msg.body_pose.position.y, 3))
                self.root.ids.z.text         = str(round(self.qc_state_msg.body_pose.position.z, 3))
                self.root.ids.dx.text        = str(round(self.qc_state_msg.body_rate.linear.x, 3))
                self.root.ids.dy.text        = str(round(self.qc_state_msg.body_rate.linear.y, 3))
                self.root.ids.dz.text        = str(round(self.qc_state_msg.body_rate.linear.z, 3))
                self.root.ids.psi.text       = str(round(psi, 3))
                self.root.ids.theta.text     = str(round(theta, 3))
                self.root.ids.phi.text       = str(round(phi, 3))
                self.root.ids.dpsi.text      = str(round(self.qc_state_msg.body_rate.angular.z, 3))
                self.root.ids.dtheta.text    = str(round(self.qc_state_msg.body_rate.angular.y, 3))
                self.root.ids.dphi.text      = str(round(self.qc_state_msg.body_rate.angular.x, 3))
                self.root.ids.uptime.text    = str(round(self.qc_state_msg.flight_duration, 3))
                self.root.ids.battery.text   = str(round(self.battery_msg.voltage, 3))
                
                self.root.ids.x_2.text       = str(round(self.qc_state_msg.body_pose.position.x, 3))
                self.root.ids.y_2.text       = str(round(self.qc_state_msg.body_pose.position.y, 3))
                self.root.ids.z_2.text       = str(round(self.qc_state_msg.body_pose.position.z, 3))
                self.root.ids.dx_2.text      = str(round(self.qc_state_msg.body_rate.linear.x, 3))
                self.root.ids.dy_2.text      = str(round(self.qc_state_msg.body_rate.linear.y, 3))
                self.root.ids.dz_2.text      = str(round(self.qc_state_msg.body_rate.linear.z, 3))
                self.root.ids.psi_2.text     = str(round(psi, 3))
                self.root.ids.theta_2.text   = str(round(theta, 3))
                self.root.ids.phi_2.text     = str(round(phi, 3))
                self.root.ids.dpsi_2.text    = str(round(self.qc_state_msg.body_rate.angular.z, 3))
                self.root.ids.dtheta_2.text  = str(round(self.qc_state_msg.body_rate.angular.y, 3))
                self.root.ids.dphi_2.text    = str(round(self.qc_state_msg.body_rate.angular.x, 3))
                self.root.ids.uptime_2.text  = str(round(self.qc_state_msg.flight_duration, 3))
                self.root.ids.battery_2.text = str(round(self.battery_msg.voltage, 3))

                self.rate.sleep()

    print('App class ended')

Teleop_GUI().run()

# os.system('rosnode kill turtlesim')

print('code ended')