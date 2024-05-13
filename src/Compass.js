import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const Compass = () => {
    const [degree, setDegree] = useState(0); // Derece için başlangıç değeri

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090' // ROS bridge sunucunuzun adresi
        });

        ros.on('connection', () => {
            console.log('Connected to ROS bridge.');
        });

        ros.on('error', (error) => {
            console.error('Error connecting to ROS bridge: ', error);
        });

        ros.on('close', () => {
            console.log('Connection to ROS bridge closed.');
        });

        const compassTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/global_position/compass_hdg',
            messageType: 'std_msgs/Float64'
        });

        compassTopic.subscribe((message) => {
            setDegree(message.data); // Gelen derece değerini güncelle
        });

        return () => {
            compassTopic.unsubscribe();
            ros.close();
        };
    }, []);

    return (
        <div className="compass-container" style={{ width: '100px', height: '100px' }}>
            <img 
                src="/compass.png" 
                alt="Compass" 
                style={{ 
                    width: '100%', 
                    height: '100%', 
                    transform: `rotate(${degree}deg)`, 
                    transition: 'transform 0.2s ease-out' // Dönüş animasyonu için
                }} 
            />
        </div>
    );
};

export default Compass;

