import React, { useEffect, useState } from "react";
import ROSLIB from 'roslib';

function DroneData() {
    const [data, setData] = useState({
        groundspeed: null,
        altitude: null,
        latitude: null,
        longitude: null,
        yaw: null,
        pitch: null,
        roll: null,
        mode: null,
        armed: null,
        battery: null,
    });

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090',
        });

        ros.on('connection', function () {
            console.log('Connected to websocket server.');
        });

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
        });

        ros.on('close', function () {
            console.log('Connection to websocket server closed.');
        });

        // State Topic
        const stateTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/state',
            messageType: 'mavros_msgs/State'
        });

        stateTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                mode: message.mode,
                armed: message.armed,
            }));
        });

        // Battery Topic
        const batteryTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/battery',
            messageType: 'sensor_msgs/BatteryState'
        });

        batteryTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                battery: parseFloat(message.percentage * 100).toFixed(1),
            }));
        });

        // Pose Topic
        const poseTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/local_position/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        poseTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                altitude: parseFloat(message.pose.position.z).toFixed(2),
                latitude: message.pose.position.x,
                longitude: message.pose.position.y,
                yaw:  parseFloat(message.pose.orientation.z).toFixed(2),
                pitch:  parseFloat(message.pose.orientation.y).toFixed(2),
                roll:  parseFloat(message.pose.orientation.x).toFixed(2),
            }));
        });

        // Velocity Topic
        const velocityTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/vfr_hud',
            messageType: 'mavros_msgs/VFR_HUD'
        });

        velocityTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                groundspeed: parseFloat(message.groundspeed).toFixed(2),
            }));
        });

    }, []);
    
    return (
        <div style={{ margin: '20px', padding: '20px', border: '1px solid #ddd' }}>
            <table style={{ width: '100%', backgroundColor: '#f9f9f9', borderCollapse: 'collapse', borderSpacing: '0' }}>
                <tbody>
                    <tr>
                        {Object.entries(data).map(([key, value]) => (
                            <React.Fragment key={key}>
                                <td style={{ padding: '8px', borderBottom: '1px solid #ddd', textAlign: 'center' }}>
                                    {key === 'battery' ? (
                                        <img src="battery.svg" alt={`Battery level ${value}%`} style={{ width: '24px', height: '24px' }} />
                                    ) : key === 'groundspeed' ? (
                                        <img src="speed.svg" alt="Groundspeed" style={{ width: '24px', height: '24px' }} />
                                    ) : (
                                        key.charAt(0).toUpperCase() + key.slice(1)
                                    )}
                                </td>
                                <td style={{ padding: '8px', borderBottom: '1px solid #ddd' }}>
                                    {key === 'armed' ? (value ? 'ARMED' : 'DISARMED') : value}
                                </td>
                            </React.Fragment>
                        ))}
                    </tr>
                </tbody>
            </table>
        </div>
    );
}

export default DroneData;
