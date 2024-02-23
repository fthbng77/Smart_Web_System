import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { Link } from 'react-router-dom';

function RosImagePage() {
    const [imgSrc, setImgSrc] = useState(null);
    const [dimensions, setDimensions] = useState({
        height: window.innerHeight,
        width: window.innerWidth,
    });
    const [isMenuVisible, setIsMenuVisible] = useState(false);
    const [selectedModel, setSelectedModel] = useState('DetectNet');

    useEffect(() => {
        function handleResize() {
            setDimensions({
                height: window.innerHeight,
                width: window.innerWidth,
            });
        }

        window.addEventListener('resize', handleResize);

        const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

        ros.on('connection', () => {
            console.log('Connected to websocket server.');
        });
        ros.on('error', (error) => {
            console.log('Error connecting to websocket server:', error);
        });
        ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            setTimeout(() => ros.connect('ws://localhost:9090'), 3000);
        });

        // WebSocket'e bağlan
        ros.connect('ws://localhost:9090');

        return () => {
            window.removeEventListener('resize', handleResize);
            ros.close();
        };
    }, []);

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        console.log(`Subscribing to /${selectedModel}/compressed`);

        const topic = new ROSLIB.Topic({
            ros,
            name: `/${selectedModel}/compressed`,
            messageType: 'sensor_msgs/CompressedImage'
        });

        topic.subscribe((message) => {
            if (message.data) {
                const imageUrl = `data:image/jpg;base64,${message.data}`;
                setImgSrc(imageUrl);
            }
        });

        return () => {
            topic.unsubscribe();
        };
    }, [selectedModel]);

    const toggleMenu = () => {
        setIsMenuVisible(!isMenuVisible);
    };

    const handleModelChange = (model) => {
        console.log(`Model changed to: ${model}`);
        setSelectedModel(model);
    };
    
    const startModel = () => {
        fetch('http://localhost:5000/start-ai-model', { method: 'POST' })
        .then(response => response.json())
        .then(data => alert(data.message))
        .catch(error => console.error('Error:', error));
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

            <div className="page-container">
                <div style={{ position: 'absolute', top: 10, left: 0, right: 0, textAlign: 'center' }}>
                    {/* Model seçimi için butonlar */}
                    {['DetectNet', 'PoseNet', 'DepthNet', 'Age_GenderNet', 'ImageNet', 'FaceNet'].map((model) => (
                        <button key={model} className="button" onClick={() => handleModelChange(model)} style={{ margin: '0 5px' }}>
                            {model}
                        </button>
                    ))}
                </div>

                <div className="page-container">
            <button onClick={startModel} className="button">Yapay Zeka Modelini Başlat</button>
            {imgSrc && (
                <div className="img-container">
                    <img src={imgSrc} alt="Detected Faces" style={{ maxWidth: '100%', height: 'auto' }} />
                </div>
            )}
            </div>
            </div>
        </div>
    );
}

export default RosImagePage;
