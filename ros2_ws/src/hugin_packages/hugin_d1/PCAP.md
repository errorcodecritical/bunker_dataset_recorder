# Replay PCAP data into a ROS2 environment

You can replay PCAP data into a ROS2 environment using the \`udpreplay\`
tool. This is useful for testing and debugging purposes.

Start the control node **BUT DO NOT START TX**. If you do, the control
node will receive data from both the PCAP file and the radar, which
can cause confusion.

    # Clone and build udpreplay
    git clone https://github.com/rigtorp/udpreplay.git
    cd udpreplay
    mkdir build
    cd build
    cmake ..
    make -j

    # udpreplay 1.0.0 © 2020 Erik Rigtorp <erik@rigtorp.se> https://github.com/rigtorp/udpreplay
    # usage: udpreplay [-i iface] [-l] [-s speed] [-c millisec] [-r repeat] [-t ttl] pcap

    #   -i iface    interface to send packets through
    #   -l          enable loopback
    #   -c millisec constant milliseconds between packets
    #   -r repeat   number of times to loop data (-1 for infinite loop)
    #   -s speed    replay speed relative to pcap timestamps
    #   -t ttl      packet ttl
    #   -b          enable broadcast (SO_BROADCAST)

    # -i has to be the interface that your radar is on.
    ./udpreplay -i enxa0cec8f59326 -l -r -1 ~/Downloads/4Dradar.pcap

    # Assuming your radar is on the 10.20.30.0/24 subnet, you can find the
    # interface name using the command:
    ip addr | grep "10.20.30"

In some cases, you may need to modify the PCAP file to change the
destination IP address. You can use \`tcprewrite\` for this purpose.

    tcprewrite \
        --infile=4Dradar.pcap \
        --outfile=4Dradar_2.pcap \
        --dstipmap=224.0.0.1/32:10.20.30.1/32 \
        --fixcsum

# Footer

Copyright (c) Sensrad 2025
