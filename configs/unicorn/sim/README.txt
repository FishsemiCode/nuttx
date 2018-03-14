Assume your NIC as below and 10.235.242.200 is free:
  eth0  Link encap:Ethernet  HWaddr f0:4d:a2:dd:ab:4c
        inet addr:10.235.242.41  Bcast:10.235.242.255  Mask:255.255.255.0

First, create the network bridge:
  sudo apt-get install bridge-utils
  sudo brctl addbr nuttx0
  sudo brctl addif nuttx0 eth0
  sudo ifconfig nuttx0 10.235.242.41/24
  sudo route add -net default gw 10.235.242.41

Second, start nuttx simulator:
  sudo nuttx
  ifconfig eth0 10.235.242.200 gw 10.235.242.41
  ping 10.235.242.41
  PING 10.235.242.41 56 bytes of data
  56 bytes from 10.235.242.41: icmp_seq=0 time=0 ms
  56 bytes from 10.235.242.41: icmp_seq=1 time=0 ms
  ...
  10 packets transmitted, 10 received, 0% packet loss, time 10000 ms

That's it.
