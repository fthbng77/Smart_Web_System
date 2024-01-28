import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import DroneData from './DroneData';
import DroneControl from './DroneControl';
import JoystickControl from './JoystickControl';
import JoystickComponent from './JoystickComponent';
import MyMapComponent from './MyMapComponent';
import { Link } from 'react-router-dom';


function App() {
    const [imgSrc, setImgSrc] = useState(null);

    useEffect(() => {
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
            ros.close();
        };

    }, []);
    
    return (
        <div>
            {/* Navigasyon Bar */}
            <nav>
                <ul>
                    <li><Link to="/app">Home</Link></li>
                    <li><Link to="/ros-image">ROS Image</Link></li>
                </ul>
            </nav>

            {/* Uygulama içeriği */}
            <div style={{ display: 'grid', gridTemplateRows: '40% 40% 20%', gridTemplateColumns: '1fr 1fr', height: '100vh', backgroundImage: `url('./gokmenwp.jpg')`, backgroundSize: 'cover', backgroundPosition: 'center center' }}>
                {/* Üst Sol - Görüntüler */}
                <div style={{ gridRow: '1', gridColumn: '1', overflow: 'hidden' }}>
                    {imgSrc && <img src={imgSrc} alt="From ROS" style={{ width: '640px', height: '480px', objectFit: 'cover' }} />}
                </div>

                {/* Üst Sağ - Drone Kontrolü ve Joystick */}
                <div style={{ gridRow: '1', gridColumn: '2', padding: '10px', background: 'rgba(255, 255, 255, 1)', boxShadow: '0px 0px 10px rgba(0,0,0,0.1)', display: 'flex', flexDirection: 'column', alignItems: 'stretch', gap: '10px' }}>
                    <div style={{ padding: '10px', border: '1px solid #ddd', borderRadius: '8px', flex: 1, display: 'flex', flexDirection: 'column', gap: '10px', boxSizing: 'border-box' }}>
                        <DroneControl />
                    </div>
                    <div style={{ padding: '0px', border: '1px solid #ddd', borderRadius: '8px', display: 'flex', flexDirection: 'row', gap: '100px', boxSizing: 'border-box', height: '130px' }}>
                        <div style={{ flex: 1 }}>
                            <JoystickControl />
                        </div>
                        <div style={{ flex: 1, overflow: 'auto', marginTop: '10px' }}>
                            <JoystickComponent />
                        </div>
                    </div>
                </div>

                {/* Orta - Drone Verileri */}
                <div style={{ gridRow: '2', gridColumn: 'span 2', background: 'rgba(249, 249, 249, 0)', padding: '20px' }}>
                    <DroneData />
                </div>

                {/* Alt - Harita */}
                <div style={{ gridRow: '3', gridColumn: 'span 2' }}>
                    <MyMapComponent />
                </div>
            </div>
        </div>
    );
}

export default App;
