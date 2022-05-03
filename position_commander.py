# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
import time
import sys
from threading import Event
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E77')

deck_attached_event = Event()

position_estimate = [0, 0, 0]


def slightly_more_complex_usage(scf):
    #with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.2,
            default_height=0.4,
            controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        # Go to a coordinate
        #pc.go_to(1.0, 1.0, 1.0)

        # Move relative to the current position
        #pc.right(1.0)

        # Go to a coordinate and use default height
        #pc.go_to(0.0, 0.0)

        # Go slowly to a coordinate
        #pc.go_to(1.0, 1.0, velocity=0.2)

        # Set new default velocity and height
        #pc.set_default_velocity(0.3)
        #pc.set_default_height(1.0)
        #pc.go_to(0.0, 0.0)

        #pc.go_to(0.0, 0.0, 0.4)
        print(position_estimate[0])
        print(position_estimate[1])
        pc.forward(0.5)
        print(position_estimate[0])
        print(position_estimate[1])
        pc.right(0.5)
        print(position_estimate[0])
        print(position_estimate[1])
        pc.back(0.5)
        print(position_estimate[0])
        print(position_estimate[1])
        pc.left(0.5)
        print(position_estimate[0])
        print(position_estimate[1])
        pc.go_to(0.0, 0.0, 0.2)
        print(position_estimate[0])
        print(position_estimate[1])


def simple_sequence(scf):
    with MotionCommander(scf, default_height=0.5) as mc:
        with Multiranger(scf) as multiranger:
            time.sleep(3)
            mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
            i=0
            while(position_estimate[2]>0.49):
                i=i+1
            print("STOP!")
            landing_spot = position_estimate
            mc.stop()
            with PositionHlCommander(
                scf
                default_velocity=0.2,
                default_height=0.5,
                controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
                pc.go_to(landing_spot[0], landing_spot[1], 0.5)
                print("LANDING")

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start() 

        simple_sequence(scf)
        #slightly_more_complex_usage(scf)

        logconf.stop()
