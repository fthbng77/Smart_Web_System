import React, { useEffect } from 'react';
import ROSLIB from 'roslib';
//import DroneData from './DroneData';
//import DroneControl from './DroneControl';
//import JoystickControl from './JoystickControl';
//import JoystickComponent from './JoystickComponent';
import MyMapComponent from './MyMapComponent';
//import { Link } from 'react-router-dom';


function App() {
    //const [imgSrc, setImgSrc] = useState(null);

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
                //const imageUrl = `data:image/jpg;base64,${message.data}`;
                //setImgSrc(imageUrl);
            }
        });

        return () => {
            ros.close();
        };

    }, []);
    
    return (
    
            {/* Navigasyon Bar 
            <nav style={{ position: 'absolute', top: 0, left: 0, right: 0, zIndex: 2 }}>
                <ul>
                    <li><Link to="/app">Home</Link></li>
                    <li><Link to="/ros-image">ROS Image</Link></li>
                </ul>
            </nav>
    
            {/*
            <div style={{ position: 'absolute', top: 50, left: 0, right: 0, zIndex: 2, display: 'grid', gridTemplateColumns: '1fr 1fr', gridTemplateRows: 'auto 1fr auto', height: 'calc(100% - 50px)' }}>
                {/* Üst Sol - Görüntüler 
                <div style={{ gridColumn: '1', gridRow: '1', padding: '10px', zIndex: 2 }}>
                    {imgSrc && <img src={imgSrc} alt="From ROS" style={{ width: '100%', height: 'auto', maxHeight: '100%' }} />}
                </div>
    
                <div style={{ gridColumn: '2', gridRow: '1', display: 'flex', flexDirection: 'column', alignItems: 'stretch', gap: '10px', padding: '10px', background: 'rgba(255, 255, 255, 0.8)', zIndex: 2 }}>
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
    
                {/* Orta - Drone Verileri 
                <div style={{ gridColumn: '1 / -1', gridRow: '2', padding: '20px', background: 'rgba(249, 249, 249, 0.8)', zIndex: 2 }}>
                    <DroneData />
                </div>
            </div>*/}
    );
}

export default App;
    

return (
    <div style={{ height: dimensions.height, width: dimensions.width, position: 'relative' }}>
        <div style={{ position: 'absolute', top: 0, left: 0, bottom: 0, right: 0, zIndex: 0 }}>
            {/* MyMapComponent burada ekranın tamamını kaplayacak şekilde yerleştiriliyor. */}
            <MyMapComponent />
        </div>
    </div>
);
}