import React, { useEffect, useState, createContext } from "react";
import ROSLIB from 'roslib';

export const DroneDataContext = createContext();

export const useDroneData = () => {
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

        ros.on('connection', () => console.log('Connected to websocket server.'));
        ros.on('error', (error) => console.log('Error connecting to websocket server: ', error));
        ros.on('close', () => console.log('Connection to websocket server closed.'));

        // Function to subscribe to a topic
        const subscribeToTopic = (topicName, messageType, callback) => {
            const topic = new ROSLIB.Topic({
                ros,
                name: topicName,
                messageType
            });

            topic.subscribe(callback);
        };

        // Define callbacks for each topic
        const updateStateData = (topicKey) => (message) => {
            setData(prevData => ({
                ...prevData,
                [topicKey]: message
            }));
        };

        subscribeToTopic('/mavros/state', 'mavros_msgs/State', updateStateData('state'));
        subscribeToTopic('/mavros/battery', 'sensor_msgs/BatteryState', updateStateData('battery'));
        subscribeToTopic('/mavros/local_position/pose', 'geometry_msgs/PoseStamped', updateStateData('pose'));
        subscribeToTopic('/mavros/vfr_hud', 'mavros_msgs/VFR_HUD', updateStateData('hud'));

    }, []);

    const renderDataRows = () => {
        const paramPairs = [
            ['GroundSpeed', 'Altitude'],
            ['Latitude', 'Longitude'],
            ['Yaw', 'Pitch'],
            ['Roll', 'Mode'],
            ['Armed', 'Battery'],
        ];

        return paramPairs.map((paramPair, rowIdx) => (
            <tr key={rowIdx} style={{ backgroundColor: '#f9f9f9', border: '1px solid #ddd', padding: '8px' }}>
                {paramPair.map((param, colIdx) => (
                    <React.Fragment key={colIdx}>
                        <td style={{ padding: '8px' }}>{param}</td>
                        <td style={{ padding: '8px' }}>
                            {param === 'Armed' ? (data.armed ? 'ARMED' : 'DISARMED') : 
                             param === 'Battery' ? `${data.battery} %` : 
                             data[param.toLowerCase()]}
                        </td>
                    </React.Fragment>
                ))}
            </tr>
        ));
    };

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
                    {renderDataRows()}
                </tbody>
            </table>
        </div>
    );
}

export default DroneData;
