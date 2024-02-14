import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import DroneData from './DroneData';
import MyMapComponent from './MyMapComponent';
import { Link } from 'react-router-dom';

function App() {
    // Ekran boyutlarını state olarak tutmak için useState kullanılıyor.
    const [dimensions, setDimensions] = useState({
        height: window.innerHeight,
        width: window.innerWidth,
    });

    useEffect(() => {
        // Pencere boyutu değiştiğinde çalışacak fonksiyon
        function handleResize() {
            setDimensions({
                height: window.innerHeight,
                width: window.innerWidth,
            });
        }

        // Pencere boyutu değişikliği dinleyicisi ekleniyor.
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
                //const imageUrl = `data:image/jpg;base64,${message.data}`;
                //setImgSrc(imageUrl);
            }
        });

        return () => {
            window.removeEventListener('resize', handleResize); // Dinleyiciyi kaldır
            ros.close(); // ROS bağlantısını kapat
        };
    }, []);

    return (
        <div style={{ height: dimensions.height, width: dimensions.width, position: 'relative' }}>
            {/* Navigasyon Bar */}
            <nav style={{ position: 'absolute', top: 500, left: 0, right: 0, zIndex: 3 }}>
                <ul>
                    <li><Link to="/app">Home</Link></li>
                    <li><Link to="/ros-image">ROS Image</Link></li>
                </ul>
            </nav>

            {/* Drone Verileri - Uygulamanın En Üstünde */}
            <div style={{ position: 'absolute', top: -40, left: 0, right: 0, zIndex: 3 }}>
                <DroneData />
            </div>

            {/* MyMapComponent burada ekranın tamamını kaplayacak şekilde yerleştiriliyor. */}
            <div style={{ position: 'absolute', top: 0, left: 0, bottom: 0, right: 0, zIndex: 2 }}>
                <MyMapComponent />
            </div>
        </div>
    );
}

export default App;
