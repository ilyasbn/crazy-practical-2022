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

prev_time = 0
prev_val = 0

position_estimate = [0, 0, 0]

D_HEIGHT = 0.4

OBST_DETECT_THRESH = 0.3
BOX_DETECT_THRESH = 0.39
X_COORD_HALF_LANDING_ZONE = 4.25
X_MAX = 5
X_DIM_LANDING_ZONE = 1.5

OFFSET_X = 1
OFFSET_Y = 1
OFFSET_Z = 0

POS_LAND_AREA = 3.5

def obst_avoidance_fwd_left(scf, mc, multiranger):

    y_init = position_estimate[1]

    #Se décale à gauche jusqu'à ce qu'il voit r
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._front_distance<0.5):
        print(multiranger._front_distance)

    save_y = position_estimate[1]
    print("save_y", save_y)
    #Se décale encore un peu à gauche de 15cm
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<0.15):
        print(position_estimate[1])

    #Avance jusqu'à voir l'obstacle à sa droite
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._right_distance>0.6):
        print(multiranger._front_distance)

    #Avance tant qu'il voit l'obstacle
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._right_distance<0.6):
        print(multiranger._right_distance)

    save_x = position_estimate[0]

    #Avance encore de 10cm
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[0]-save_x)<0.1):
        print(multiranger._right_distance)

    save_y = position_estimate[1]
    dist = abs(y_init-save_y)

    #Revient à sa position initiale en y
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<dist):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

def obst_avoidance_fwd_right(scf, mc, multiranger):
    y_init = position_estimate[1]

    #Se décale à droite jusqu'à ce qu'il voit r
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._front_distance<0.5):
        print(multiranger._front_distance)

    save_y = position_estimate[1]

    #Se décale encore un peu à droite de 15cm
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<0.15):
        print(position_estimate[1])

    #Avance jusqu'à voir l'obstacle à sa gauche
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._left_distance>0.6):
        print(multiranger._left_distance)

    #Avance tant qu'il voit l'obstacle
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._left_distance<0.6):
        print(multiranger._left_distance)

    save_x = position_estimate[0]

    #Avance encore de 10cm
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[0]-save_x)<0.1):
        print(multiranger._right_distance)

    save_y = position_estimate[1]
    dist = abs(y_init-save_y)

    #Revient à sa position initiale en y
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<dist):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

def obst_avoidance_bwd_right(scf, mc, multiranger):
    y_init = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._back_distance<0.5):
        print(multiranger._back_distance)

    save_y = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<0.15):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._left_distance>0.6):
        print(multiranger._left_distance)

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._left_distance<0.6):
        print(multiranger._left_distance)

    save_x = position_estimate[0]

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[0]-save_x)<0.1):
        print(multiranger._left_distance)

    save_y = position_estimate[1]
    dist = abs(y_init-save_y)

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<dist):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

def obst_avoidance_bwd_left(scf, mc, multiranger):
    y_init = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._back_distance<0.5):
        print(multiranger._back_distance)

    save_y = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<0.15):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._right_distance>0.6):
        print(multiranger._right_distance)

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(multiranger._right_distance<0.6):
        print(multiranger._right_distance)

    save_x = position_estimate[0]

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[0]-save_x)<0.1):
        print(multiranger._right_distance)

    save_y = position_estimate[1]
    dist = abs(y_init-save_y)

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    while(abs(position_estimate[1]-save_y)<dist):
        print(position_estimate[1])

    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

def obst_avoidance_right_side(scf, mc, multiranger):

    if(position_estimate[0]<X_COORD_HALF_LANDING_ZONE):
        mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
        while(multiranger._right_distance<0.3):
            print(multiranger._right_distance)
    else:
        mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
        while(multiranger._right_distance<0.3):
            print(multiranger._right_distance)

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)

    return 1

def go_to_landing_zone(scf, mc, multiranger):
    mc.start_linear_motion(velocity_x_m=0.4, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
    while(position_estimate[0]<POS_LAND_AREA):
            print(multiranger._front_distance)

            if(multiranger._front_distance<OBST_DETECT_THRESH):
            	if(position_estimate[1]>1.5):
            		obst_avoidance_fwd_left(scf, mc, multiranger)
            	else:
            		obst_avoidance_fwd_right(scf, mc, multiranger)

    return 0 # A atteint landing zone

def snake_sequence(scf, mc, multiranger):
	#Avancer jusqu'à la fin de la zone
    mc.start_linear_motion(velocity_x_m=0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    while(position_estimate[0]<(X_MAX-0.15)):
        print(multiranger._down_distance)
        #if landing box found
        if(check_box(scf, mc, multiranger)):
            return 1

        if(multiranger._front_distance<OBST_DETECT_THRESH):
        	if(position_estimate[1]>1.5):
        		obst_avoidance_fwd_left(scf, mc, multiranger)
        	else:
        		obst_avoidance_fwd_right(scf, mc, multiranger)

    #Go 30 cm to the right
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)
    y_init = position_estimate[1]
    while(abs(position_estimate[1]-y_init)<0.3):
        print(multiranger._right_distance)
        #if landing box found
        if(check_box(scf, mc, multiranger)):
            return 1

        if(multiranger._right_distance<OBST_DETECT_THRESH):
        	obst_avoidance_right_side(scf, mc, multiranger)

    #Avancer jusqu'au début de la zone
    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
    while(position_estimate[0]>X_MAX-X_DIM_LANDING_ZONE+0.15):
        print(multiranger._back_distance)
        #if landing box found
        if(check_box(scf, mc, multiranger)):
            return 1

        if(multiranger._back_distance<OBST_DETECT_THRESH):
            if(position_estimate[1]>1.5):
                obst_avoidance_bwd_left(scf, mc, multiranger)
            else:
                obst_avoidance_bwd_right(scf, mc, multiranger)

	#Go 30 cm to the right
    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.2, velocity_z_m=0.0, rate_yaw=0.0)
    y_init = position_estimate[1]
    while(abs(position_estimate[1]-y_init)<0.3):
        print(multiranger._right_distance)
        #if landing box found
        if(check_box(scf, mc, multiranger)):
            return 1

        if(multiranger._right_distance<OBST_DETECT_THRESH):
        	obst_avoidance_right_side(scf, mc, multiranger)

    return 0

def go_to_starting_zone(scf, mc, multiranger):
    mc.start_linear_motion(velocity_x_m=-0.2, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)

    x_init = position_estimate[0]

    while(position_estimate[0]>0):
	    print(multiranger._back_distance)

	    if(multiranger._back_distance<OBST_DETECT_THRESH):
	    	if(position_estimate[1]>1.5):
	    		obst_avoidance_bwd_left(scf, mc, multiranger)
	    	else:
	    		obst_avoidance_bwd_right(scf, mc, multiranger)

    return 0

def land_on_box(scf, mc, multiranger):

    save_x = position_estimate[0]

    mc.start_linear_motion(velocity_x_m=0.1, velocity_y_m=0.0, velocity_z_m=0.0, rate_yaw=0.0)
    while(position_estimate[0]<save_x+0.25):
        print(multiranger._down_distance)

    save_y = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=0.1, velocity_z_m=0.0, rate_yaw=0.0)
    while(position_estimate[1]<save_y+0.3):
        print(multiranger._down_distance)

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.1, velocity_z_m=0.0, rate_yaw=0.0)

    while(check_box(scf, mc, multiranger)):
        print(multiranger._down_distance)

    save_y = position_estimate[1]

    mc.start_linear_motion(velocity_x_m=0.0, velocity_y_m=-0.1, velocity_z_m=0.0, rate_yaw=0.0)
    while(position_estimate[1]>save_y-0.3):
        print(multiranger._down_distance)

    print("STOP!")

    landing_spot = position_estimate
    mc.stop()
    print("LANDING")
    time.sleep(1)
    print(position_estimate[0])


def slightly_more_complex_usage(scf):

    print("TAKE OFF")
    with MotionCommander(scf, default_height=D_HEIGHT) as mc:
        with Multiranger(scf) as multiranger:
            time.sleep(2)
            print("START")

            go_to_landing_zone(scf, mc, multiranger)

            save_x = position_estimate[0]

            box_found = 0
            while(not box_found):
                box_found = snake_sequence(scf, mc, multiranger)
            
            land_on_box(scf, mc, multiranger)

            #go_to_starting_zone(scf, mc, multiranger)

            #land_on_box(scf, mc, multiranger)

def check_box(scf, mc, multiranger):
    global prev_time
    global prev_val
    delta_t = time.time()-prev_time
    if(delta_t>0.2):
        prev_time = time.time()
        new_val = multiranger._down_distance
        print("DIFF")
        print(prev_val - new_val)
        if( (prev_val - new_val) > 0.03 ):
            return 1
        prev_val = new_val

    return 0

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate

    position_estimate[0] = data['stateEstimate.x'] + OFFSET_X
    position_estimate[1] = data['stateEstimate.y'] + OFFSET_Y
    position_estimate[2] = data['stateEstimate.z'] + OFFSET_Z

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

        slightly_more_complex_usage(scf)

        logconf.stop()
