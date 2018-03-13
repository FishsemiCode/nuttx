CONNECT NETWORK TRAFFIC TO THE BRIDGE:

    Run following commands on !!HOST!! to routing nuttx local traffic to the bridge:

    1. Install bridge-utils extensions:
        >> sudo apt-get install bridge-utils

    2. Creates a new instance of the ethernet bridge:
    (Bridge name "nuttx0" must be same with CONFIG_SIM_NET_BRIDGE_DEVICE)
        >> sudo brctl addbr nuttx0

    3. Make the interface <eth0> a port of the bridge <nuttx0>
    (please check the network interface name whether is "eth0" through $ifconfig)
        >> sudo brctl addif nuttx0 eth0

    4. Configure nuttx0 ip address to host <eth0> address
        >> sudo ifconfig nuttx0 [addr]

    5. Add route gateway (same as ip address is fine):
        >> sudo route add -net default gw [addr]

    6. Start Nuttx and configure the eth0 address at same network segment
        !!NUTTX!! >> ifconfig eth0 [addr]
