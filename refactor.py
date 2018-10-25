#!/usr/bin/env python

import datetime
import json
import logging
import logging.handlers
import os
import ptvsd
import pymavlink.mavutil
import requests
import socket
import subprocess
import sys
import time
# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..'))

# Allow other computers to attach to ptvsd at this IP address and port.
#ptvsd.enable_attach(address=('172.29.0.142', 5678), redirect_output=True)

#ptvsd.enable_attach()

# Pause the program until a remote debugger is attached
#ptvsd.wait_for_attach()
#ptvsd.break_into_debugger()

logger = logging.getLogger('logger')
logger.setLevel(logging.DEBUG)
handler = logging.handlers.SysLogHandler(address='/dev/log')
formatter = logging.Formatter('%(name)s: %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class Field:
    def __init__(self, field_id):
        self.field_id = field_id
        self.value = ''
        self.changed = False


class FenceStatus:
    def __init__(self):
        self.breach_time = ''
        self.breach_count = ''
        self.breach_status = ''
        self.breach_type = ''
        self.changed = False


class Vehicle_fields:
    def __init__(self):
        self.ground_speed = Field(1)
        self.heading = Field(2)
        self.battery_voltage = Field(3)
        self.armed = Field(4)
        self.latitude = Field(5)
        self.longitude = Field(6)
        self.altitude = Field(7)
        self.flightmode = Field(8)
        self.ekf_health = Field(9)


class Hardware:
    def __init__(self, uuid):
        self.uuid = uuid
        self.vehicle_fields = Vehicle_fields()
        self.fence_status = FenceStatus()
        self.most_recent_vehicle_field_log_update = time.time()

    def set_internal_vehicle_field(self, to_update, new_value):
        #FENCE_STATUS messages are a different format.  How to properly check?
        #Only getting value, can't tell if it's dictionary key was something like 'self.fence_status.breach_count'
        #        if (to_update is self.fence_status.breach_count) or (to_update is self.fence_status.breach_status):
        if self.fence_status.breach_count in [to_update]:
            #        if to_update in [ self.fence_status.breach_count, self.fence_status.breach_status ] :
            to_update = new_value
            self.fence_status.changed = True
            return None

        to_update.value = new_value
        to_update.changed = True

        return None

    def compare_and_update_internal_states(self, message, armed=None, flightmode=None):
        if message.name == 'EKF_STATUS_REPORT':
            fieldnames = {
                self.vehicle_fields.ekf_health: message.flags
            }
        elif message.name == 'FENCE_STATUS':
            fieldnames = {
                self.fence_status.breach_count: message.breach_count,
                self.fence_status.breach_status: message.breach_status
            }
        elif message.name == 'GLOBAL_POSITION_INT':
            fieldnames = {
                self.vehicle_fields.latitude: message.lat,
                self.vehicle_fields.longitude: message.lon,
                self.vehicle_fields.altitude: message.relative_alt,
                self.vehicle_fields.heading: message.hdg
            }
        elif message.name == 'GPS_RAW_INT':
            fieldnames = {
                self.vehicle_fields.ground_speed: message.vel
            }
        elif message.name == 'HEARTBEAT':
            fieldnames = {
                self.vehicle_fields.armed: armed,
                self.vehicle_fields.flightmode: flightmode
            }
        elif message.name == 'SYS_STATUS':
            fieldnames = {
                self.vehicle_fields.battery_voltage: message.voltage_battery
            }

        if message.name != 'FENCE_STATUS':
            for internal, incoming in fieldnames.iteritems():
                if internal.value != incoming:
                    self.set_internal_vehicle_field(internal, incoming)
        else:
            #These 'FENCE_STATUS' messages need to be handled differently (if sticking with dictionary method in block above)
            #I think the block above is not a good choice though
            for internal, incoming in fieldnames.iteritems():
                if internal != incoming:
                    self.set_internal_vehicle_field(internal, incoming)

        return None

    def api_post_vehicle(self, update, route):
        headers = {"Authorization": "Bearer 12345",
                   "Content-Type": "application/json", "Origin": "http://localhost"}
        path = "vehicle/" + self.uuid + "/" + route
        address = API_ROOT + path

        response = requests.post(
            address, data=json.dumps(update), headers=headers)
        logger.debug("API call: %(address)s body: %(update)s response: %(response)s",
                     {'address': address, 'update': update, 'response': response})
        return response

    def api_vehicle_event_log_update(self, event_type_id, description):
        json_object = [
            {
                "event_type_id": "" + str(event_type_id) + "",
     		         "description": "" + str(description) + ""
            }
        ]

        response = self.api_post_vehicle(json_object, 'event')

        if response.status_code is not 200:
            logger.debug("API update of vehicle_event_logs failed for vehicle UUID " +
                         self.uuid + " : " + response.text)

            return False
        else:
            return True

    def api_vehicle_field_log_update(self):
        json_object = []
        #Is there a better way to do this? 
        attrs = [attr for attr in dir(self.vehicle_fields) if attr[:2] != '__']

        for attr in attrs:
            field = getattr(self.vehicle_fields, attr)

            if field.changed == True:
                json_object.append(
                    {
                        "field_id": "" + str(field.field_id) + "",
                        "value": "" + str(field.value) + "",
                        "set_by": "" + 1234 + ""
                    }
                )

        response = self.api_post_vehicle(json_object, 'field')

        if response.status_code is not 200:
            logger.debug("API update of vehicle_field_logs failed for vehicle UUID " +
                         self.uuid + " : " + response.text)

            return False
        else:
            for attr in attrs:
                field = getattr(self.vehicle_fields, attr)
                field.changed = False
            return True


fleet = {}


def add_hardware_to_fleet(uuid):
    device = Hardware(uuid)

    fleet[uuid] = device

    print ("Added to fleet: " + str(fleet[uuid].uuid) + " " + str(device))
    return device

def parse_data():
    mav = pymavlink.mavutil.mavlink_connection('udpin: 0.0.0.0:14550')

    print("Parsing mavlink stream from 0.0.0.0:14550")
    mav.wait_heartbeat()

    POLLING_CYCLE = 10
    VEHICLE_EVENT_TYPE_FENCE_STATUS = 8

    while True:
        message = mav.recv_match(type=['EKF_STATUS_REPORT', 'FENCE_STATUS', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'HEARTBEAT', 'STATUSTEXT', 'SYS_STATUS'],
                                 condition='str(MAV.target_system)=="%s"' % '1', blocking=True)

        if (message is None):
            return

        device_name = 'dmSITL'

        vehicle_uuid = '12345'

        if vehicle_uuid in fleet:
            device = fleet[vehicle_uuid]
        else:
            device = add_hardware_to_fleet(vehicle_uuid)

        if (type(message) is not pymavlink.dialects.v10.ardupilotmega.MAVLink_bad_data):
            if message.name in ['EKF_STATUS_REPORT', 'FENCE_STATUS', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'SYS_STATUS']:
                device.compare_and_update_internal_states(message)
            elif message.name == 'HEARTBEAT':
                device.compare_and_update_internal_states(
                    message, pymavlink.mavutil.mavfile.motors_armed(mav), mav.flightmode)

        elapsed = (time.time() - device.most_recent_vehicle_field_log_update)

        if (elapsed > POLLING_CYCLE) or device.vehicle_fields.armed.changed or device.vehicle_fields.flightmode.changed:
            if device.api_vehicle_field_log_update():
                device.most_recent_vehicle_field_log_update = time.time()


API_ROOT = "https://google.com/"

parse_data()
