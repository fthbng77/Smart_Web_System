import React, { useEffect, useState, createContext } from "react";
import ROSLIB from 'roslib';

export const DroneDataContext = createContext(null);


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

        const batteryTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/battery',
            messageType: 'sensor_msgs/BatteryState'
        });

        batteryTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                // Örnek olarak, voltage ve current kullanıldı.
                battery: message.percentage * 100,
            }));
        });

        const poseTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/local_position/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        poseTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                altitude: message.pose.position.z,
                latitude: message.pose.position.x,
                longitude: message.pose.position.y,
                yaw: message.pose.orientation.z,
                pitch: message.pose.orientation.y,
                roll: message.pose.orientation.x,
            }));
        });

        const velocityTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/vfr_hud',
            messageType: 'mavros_msgs/VFR_HUD'
        });

        velocityTopic.subscribe(function (message) {
            setData(prevData => ({
                ...prevData,
                groundspeed: message.groundspeed,
            }));
        });

    }, []);


    return (
        <div style={{ margin: '20px', padding: '20px', border: '1px solid #ddd' }}>
            <table style={{ width: '100%', backgroundColor: '#f9f9f9', borderCollapse: 'separate', borderSpacing: '0 10px' }}>
                <thead>
                    <tr>
                        <th style={{ borderBottom: '1px solid #ddd' }}>Parameter</th>
                        <th style={{ borderBottom: '1px solid #ddd' }}>Value</th>
                        <th style={{ borderBottom: '1px solid #ddd' }}>Parameter</th>
                        <th style={{ borderBottom: '1px solid #ddd' }}>Value</th>
                    </tr>
                </thead>
                <tbody>
                    {[
                        ['GroundSpeed', 'Altitude'],
                        ['Latitude', 'Longitude'],
                        ['Yaw', 'Pitch'],
                        ['Roll', 'Mode'],
                        ['Armed', 'Battery'],
                    ].map((paramPair, rowIdx) => (
                        <tr key={rowIdx} style={{ backgroundColor: '#f9f9f9', border: '1px solid #ddd', padding: '8px' }}>
                            {paramPair.map((param, colIdx) => (
                                <React.Fragment key={colIdx}>
                                    <td style={{ padding: '8px' }}>{param}</td>
                                    <td style={{ padding: '8px' }}>
                                        {param === 'Armed' ? (data.armed ? 'ARMED' : 'DISARMED') : param === 'Battery' ? `${data.battery} %` : data[param.toLowerCase()]}
                                    </td>
                                </React.Fragment>
                            ))}
                        </tr>
                    ))}
                </tbody>
            </table>
        </div>
    );
}

export default DroneData;