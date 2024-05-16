import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import DroneData from './DroneData';
import DroneControl from './DroneControl';
import MyMapComponent from './MyMapComponent';
import JoystickControl from './JoystickControl';
import Compass from './Compass'; // Compass bileşenini import edin
import { Link } from 'react-router-dom'; // Import Link from react-router-dom

function App() {
  const [imgSrc, setImgSrc] = useState(null);
  const [dimensions, setDimensions] = useState({
    height: window.innerHeight,
    width: window.innerWidth,
  });
  const [isMenuVisible, setIsMenuVisible] = useState(false);
  const [isJoystickVisible, setIsJoystickVisible] = useState(false);

  useEffect(() => {
    function handleResize() {
      setDimensions({
        height: window.innerHeight,
        width: window.innerWidth,
      });
    }

    window.addEventListener('resize', handleResize);

    // ROSLIB ile ilgili kodlar
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    const connectToRos = () => {
      ros.connect('ws://localhost:9090');
    };

    ros.on('connection', function () {
      console.log('Connected to websocket server.');
    });

    ros.on('error', function (error) {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
      console.log('Connection to websocket server closed.');
      setTimeout(connectToRos, 3000);
    });

    connectToRos();

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/webcam/image_raw/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

    topic.subscribe(function (message) {
      console.log('Received message on ' + topic.name + ': ', message);

      if (message.data) {
        const imageUrl = `data:image/jpg;base64,${message.data}`;
        setImgSrc(imageUrl);
      }
    });

    return () => {
      window.removeEventListener('resize', handleResize);
      ros.close();
    };
  }, []);

  const toggleMenu = () => {
    setIsMenuVisible(!isMenuVisible);
  };

  const toggleJoystickVisibility = () => {
    setIsJoystickVisible(!isJoystickVisible);
  };

  return (
    <div style={{ height: dimensions.height, width: dimensions.width, position: 'relative' }}>
      <img src="/menu.svg" alt="Menu" onClick={toggleMenu} style={{ cursor: 'pointer', position: 'absolute', top: 0, left: 0, zIndex: 4 }}/>
      {isMenuVisible && (
        <nav style={{ position: 'absolute', top: 50, left: 0, right: 0, zIndex: 3 }}>
          <ul>
            <li><Link to="/app">Home</Link></li>
            <li><Link to="/ros-image">ROS Image</Link></li>
          </ul>
        </nav>
      )}
      <div style={{ position: 'absolute', bottom: 0, left: 0, zIndex: 3 }}>
        {imgSrc && <img src={imgSrc} alt="From ROS" style={{ width: '70%', height: 'auto', maxHeight: '100%' }} />}
      </div>

      {/* DroneControl ve JoystickControl ile Switch Toggle */}
      <div style={{
        position: 'absolute',
        bottom: '0%',
        right: '0%',
        zIndex: 3,
        backgroundColor: 'rgba(255, 255, 255, 0.8)',
        padding: '35px',
        borderRadius: '20px',
        maxWidth: '300px',
        width: '100%',
        maxHeight:'350px',
        height: '100%',
        boxSizing: 'border-box',
      }}>
        <div style={{ position: 'absolute', top: '0px', left: '0px', zIndex: 4 }}>
          <label className="switch">
            <input type="checkbox" checked={isJoystickVisible} onChange={toggleJoystickVisibility} />
            <span className="slider round"></span>
          </label>
        </div>

        <div style={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          {!isJoystickVisible ? <DroneControl /> : <JoystickControl />}
        </div>
      </div>

      <div style={{ position: 'absolute', top: -40, left: 0, right: 0, zIndex: 3 }}>
        <DroneData />
      </div>
      <div style={{ position: 'absolute', top: 0, left: 0, bottom: 0, right: 0, zIndex: 2 }}>
        <MyMapComponent />
      </div>

      {/* Compass bileşenini ekleyin */}
      <div style={{ position: 'absolute', top: -370, right: 10, zIndex: 3 }}>
        <Compass />
      </div>
    </div>
  );
}

export default App;
