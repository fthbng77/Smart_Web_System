import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import compassImage from './compass.png';

const Compass = () => {
  const [heading, setHeading] = useState(0);


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
            setHeading(message.data);
        });

        return () => {
            compassTopic.unsubscribe();
            ros.close();
        };
    }, []);
    
  const compassStyle = {
    transform: `rotate(${heading}deg)`,
    width: '150px',
    height: '150px',
    transition: 'transform 0.5s ease-in-out',
  };

  return (
    <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
      <img src={compassImage} alt="Compass" style={compassStyle} />
    </div>
  );
};

export default Compass;
