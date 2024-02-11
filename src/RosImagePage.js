// src/RosImagePage.js
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function RosImagePage() {
    const [imgSrc, setImgSrc] = useState(null);

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });
    
        const connectToRos = () => {
            ros.connect('ws://localhost:9090');
        };
    
        ros.on('connection', () => {
            console.log('Connected to websocket server.');
        });
    
        ros.on('error', (error) => {
            console.log('Error connecting to websocket server: ', error);
        });
    
        ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            // Bağlantıyı yeniden kurmak için tekrar deneyin
            setTimeout(connectToRos, 3000);
        });
    
        // Bağlantıyı hemen kurmaya çalış
        connectToRos();
    
        const topic = new ROSLIB.Topic({
            ros: ros,
            name: '/detected_faces/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        });
    
        // Topic'ten mesaj geldiğinde bu fonksiyon çalışacak
        topic.subscribe((message) => {
            console.log('Received message on ' + topic.name + ': ', message);
    
            // Mesajın data alanını kullanarak bir görüntü URL'si oluşturun
            if (message.data) {
                const imageUrl = `data:image/jpg;base64,${message.data}`;
                setImgSrc(imageUrl);
            }
        });

        // Cleanup fonksiyonu
        return () => {
            topic.unsubscribe();
            ros.close();
        };

    }, []);
    
    return (
        <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
            {imgSrc && <img src={imgSrc} alt="From ROS" style={{ maxWidth: '100%', height: 'auto' }} />}
        </div>
    );
}

export default RosImagePage;
