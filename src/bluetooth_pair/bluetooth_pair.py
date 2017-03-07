# Software License Agreement (BSD)
#
# @author    Mike O'Driscoll <modriscoll@clearpathrobotics.com>
# @copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import absolute_import, print_function, unicode_literals

import gtk
import rospy
import dbus
import dbus.service
import dbus.mainloop.glib
from bluetooth_pair.msg import BluetoothDevice, BluetoothDevices
from bluetooth_pair.srv import PairDevice, PairDeviceResponse, SearchDevices, SearchDevicesResponse


class Agent(dbus.service.Object):
    pass


class BluetoothPairNode(object):
    def __init__(self, node_name='bluetooth_pair'):
        rospy.init_node(node_name)
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

        self.paired = False
        self.pair_error = False
        self.found_devices = dict()
        self.search_filter = ""

        # Setup dbus for interacting with bluez
        self.bus = dbus.SystemBus()
        self.dbus_path = '/ros/bluetooth_pair'
        self.agent = Agent(self.bus, self.dbus_path)
        self.manager = dbus.Interface(self.bus.get_object('org.bluez', '/'),
                                      'org.bluez.Manager')
        self.adapter = dbus.Interface(self.bus.get_object('org.bluez', self.manager.DefaultAdapter()),
                                      'org.bluez.Adapter')

        # Add signal receivers for discovery
        self.bus.add_signal_receiver(self.device_found,
                                     dbus_interface='org.bluez.Adapter',
                                     signal_name='DeviceFound')
        self.bus.add_signal_receiver(self.property_changed,
                                     dbus_interface='org.bluez.Adapter',
                                     signal_name='PropertyChanged')

        # Configure ROS services
        self.request_scan_service = rospy.Service('search_devices',
                                                  SearchDevices, self.search_devices)
        self.pair_service = rospy.Service('pair_device',
                                          PairDevice, self.pair_device)
        self.remove_service = rospy.Service('remove_device',
                                            PairDevice, self.remove_device)

        # Configure ROS publishers
        self.list_devices_pub = rospy.Publisher('discovered_devices', BluetoothDevices, queue_size=1, latch=True)
        self.list_paired_pub = rospy.Publisher('paired_devices', BluetoothDevices, queue_size=1, latch=True)

    def run(self):
        rospy.Timer(rospy.Duration(10), self.pub_paired_devices)
        rospy.loginfo('Starting bluetooth_pair')
        while not rospy.is_shutdown():
            self.wait()

    def wait(self):
        # Run GTK events and sleep as the same rate as rospy.spin(), do not block
        # on the GTK iterate or rospy callbacks won't execute.
        gtk.main_iteration(block=False)
        rospy.rostime.wallsleep(0.5)

    def pub_paired_devices(self, event):
        paired = BluetoothDevices()
        for path in self.adapter.ListDevices():
            device = dbus.Interface(self.bus.get_object('org.bluez', path),
                                    'org.bluez.Device')
            properties = device.GetProperties()
            paired.devices.append(BluetoothDevice(mac_address=str(properties['Address']),
                                                  device_name=str(properties['Name'])))

        self.list_paired_pub.publish(paired)

    # ROS Service calls
    def search_devices(self, req):
        self.found_devices = dict()  # New search requested, clear old results.
        self.search_filter = req.device_name
        rospy.loginfo('Starting discovery')
        self.adapter.StartDiscovery()
        return SearchDevicesResponse()

    def pair_device(self, req):
        # Remove an already paired device, as the pair key may be
        # no longer valid, don't worry about any errors, just pass.
        try:
            device = self.adapter.FindDevice(req.device_mac)
            self.adapter.RemoveDevice(device)
            return PairDeviceResponse(True)
        except dbus.exceptions.DBusException:
            pass

        self.paired = False
        self.pair_error = False
        rospy.loginfo('Pairing with %s', req.device_mac)
        self.adapter.CreatePairedDevice(req.device_mac,
                                        self.dbus_path,
                                        'NoInputNoOutput',
                                        reply_handler=self.create_device_reply,
                                        error_handler=self.create_device_error)

        while not self.paired and not self.pair_error:
            self.wait()  # Keep GTK lib spinning

        if self.paired:
            rospy.loginfo('Trusting %s', req.device_mac)
            path = self.adapter.FindDevice(req.device_mac)
            device = dbus.Interface(self.bus.get_object('org.bluez', path),
                                    'org.bluez.Device')
            device.SetProperty('Trusted', dbus.Boolean(1))
            return PairDeviceResponse(True)
        elif self.pair_error:
            return PairDeviceResponse(False)

        # Fallthrough error
        return PairDeviceResponse(False)

    def remove_device(self, req):
        try:
            device = self.adapter.FindDevice(req.device_mac)
            self.adapter.RemoveDevice(device)
            return PairDeviceResponse(True)
        except dbus.exceptions.DBusException as e:
            rospy.logerr('Failed to remove device: %s', e)
            return PairDeviceResponse(False)

    # dbus callbacks
    def create_device_reply(self, device):
        rospy.loginfo('Paired with new device %s', device)
        self.paired = True

    def create_device_error(self, error):
        if 'Already Exists' in error.get_dbus_message():
            rospy.loginfo('Device already paired')
            self.paired = True
        else:
            rospy.logerr('Pairing failure: %s', error)
            self.pair_error = True

    def device_found(self, address, properties):
        if 'Name' in properties.keys():
            rospy.loginfo('Found %s:%s', properties['Name'], properties['Address'])
            if self.search_filter in properties['Name']:
                self.found_devices[address] = BluetoothDevice(mac_address=str(properties['Address']),
                                                              device_name=str(properties['Name']))
            elif not self.search_filter:
                self.found_devices[address] = BluetoothDevice(mac_address=str(properties['Address']),
                                                              device_name=str(properties['Name']))
            self.list_devices_pub.publish([v for (k, v) in self.found_devices.items()])

    def property_changed(self, name, value):
        if (name == 'Discovering' and not value):
            self.adapter.StopDiscovery()
            rospy.loginfo('Discovery Complete')
