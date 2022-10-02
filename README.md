# Smart Glove

This package aims to manage the communication via Wifi (WebSocket) of a smart glove (equipped with ESP-32 microcontroller) and make it a wrapper for the ROS environment.

The angular position of the fingers is thus published via ROS message of type JointState.

<br>

<p align="center">
  <img height="200" src="http://schoolcommunity.altervista.org/uni/immagini/Smart_Glove_Icon.jpg">
</p>



## Launch parameters
The launch file smart_glove.launch is in charge to make the system run in easy way. The following parameter is available:
-  <code>ip_address_left_glove</code> to specify the ip address of Smart-Glove.
- <code>smart_glove_ns</code> the "name" of the glove (default: right), that is usefull in case of more than one glove.

## Requirements
- **WebSocket Client Library** (follow this link if needed https://websocket-client.readthedocs.io/en/latest/installation.html):
  ```pip3 install websocket-client
  ```

## Maintainers
- Samuele Sandrini, [SamueleSandrini](https://github.com/SamueleSandrini)

