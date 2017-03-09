# Bluetooth Pairing Node

This node provides the following services to assist in pairing a bluetooth devices

Services:
* `search_devices`: Runs bluetooth discovery looking for bluetooth devices. Lists discovered devices on topic.
* `pair_device`: Given a MAC address for a device it will attempt to pair with it. Device must be in pairing mode.
* `remove_device`: Given a MAC address for a device it will remove it from the paired and trusted devices.

This node only pairs and trusts devices, any further configuration to make the device work with other nodes
must be configured elsewhere.


Topics:
* `discovered_devices`: A latched topic providing a list of all devices currently seen by the discovery service.
* `paired_devices`: A list of devices paired with the system.
