#!/bin/bash

NOOFAGENTS=$1
> ../Firmware/launch/multiUAVLaunch.launch

defaultUDP=14500
defaultLocalHost=14570
defaultMavLinkUDP=14600
defaultMavLinkTCP=4560

sin ()
{
    echo "scale=5;10*s($1*0.017453293)" | bc -l
}

cos ()
{
    echo "scale=5;10*c($1*0.017453293)" | bc -l
}

divInc=$((360/$NOOFAGENTS))

echo "<?xml version=\"1.0\"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name=\"est\" default=\"ekf2\"/>
    <arg name=\"vehicle\" default=\"iris\"/>
    <arg name=\"world\" default=\"\$(find mavlink_sitl_gazebo)/worlds/empty.world\"/>
    <!-- gazebo configs -->
    <arg name=\"gui\" default=\"true\"/>
    <arg name=\"debug\" default=\"false\"/>
    <arg name=\"verbose\" default=\"false\"/>
    <arg name=\"paused\" default=\"false\"/>
    <!-- Gazebo sim -->
    <include file=\"\$(find gazebo_ros)/launch/empty_world.launch\">
        <arg name=\"gui\" value=\"\$(arg gui)\"/>
        <arg name=\"world_name\" value=\"\$(arg world)\"/>
        <arg name=\"debug\" value=\"\$(arg debug)\"/>
        <arg name=\"verbose\" value=\"\$(arg verbose)\"/>
        <arg name=\"paused\" value=\"\$(arg paused)\"/>
    </include>" >> ../Firmware/launch/multiUAVLaunch.launch
for i in `seq 1 $NOOFAGENTS`
do
   echo "
    <!-- UAV$i -->
    <group ns=\"uav$i\">
        <!-- MAVROS and vehicle configs -->
        <arg name=\"ID\" value=\"$i\"/>
        <arg name=\"fcu_url\" default=\"udp://:$(($defaultUDP+$i))@localhost:$(($defaultLocalHost+$i))\"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file=\"\$(find px4)/launch/single_vehicle_spawn.launch\">
            <!--arg name=\"x\" value=\"${pos_x[${i}-1]}\"/>
            <arg name=\"y\" value=\"${pos_y[${i}-1]}\"/-->
            <arg name=\"x\" value=\" $(sin $i*$divInc) \"/>
            <arg name=\"y\" value=\" $(cos $i*$divInc) \"/>
            <arg name=\"z\" value=\"0\"/>
            <arg name=\"R\" value=\"0\"/>
            <arg name=\"P\" value=\"0\"/>
            <arg name=\"Y\" value=\"0\"/>
            <arg name=\"vehicle\" value=\"\$(arg vehicle)\"/>
            <arg name=\"mavlink_udp_port\" value=\"$(($defaultMavLinkUDP+$i))\"/>
            <arg name=\"mavlink_tcp_port\" value=\"$(($defaultMavLinkTCP+$i))\"/>
            <arg name=\"ID\" value=\"\$(arg ID)\"/>
            <!--arg name=\"node_start_delay\" value=\"5\"/-->
        </include>
        <!-- MAVROS -->
        <include file=\"\$(find mavros)/launch/px4.launch\">
            <arg name=\"fcu_url\" value=\"\$(arg fcu_url)\"/>
            <arg name=\"gcs_url\" value=\"\"/>
            <arg name=\"tgt_system\" value=\"\$(eval 1 + arg('ID'))\"/>
            <arg name=\"tgt_component\" value=\"1\"/>
        </include>
    </group>" >> ../Firmware/launch/multiUAVLaunch.launch
done
echo "</launch>" >> ../Firmware/launch/multiUAVLaunch.launch
