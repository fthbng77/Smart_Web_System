import React, { useState, useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import ROSLIB from 'roslib';

const customIcon = L.icon({
    iconUrl: '/marker.png',
    iconSize: [48, 48],
    iconAnchor: [16, 32],
    popupAnchor: [0, -32]
});

function useDroneData() {
    const [data, setData] = useState({
        latitude: 41.016593, 
        longitude: 28.951300,
    });

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090' // ROS websocket sunucusunun URL'si
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

        // Drone konum verilerini alacak ROS topic aboneliği
        const poseTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/local_position/pose', // Değiştirilebilir, drone'un konum verilerinin alındığı ROS topic'i
            messageType: 'geometry_msgs/PoseStamped'
        });

        poseTopic.subscribe(function (message) {
            setData({
                latitude: message.pose.position.x, // Konum verileri burada güncelleniyor
                longitude: message.pose.position.y, // Konum verileri burada güncelleniyor
            });
        });

        // Cleanup fonksiyonu
        return () => {
            poseTopic.unsubscribe();
        };
    }, []);

    return data;
}

function MyMapComponent() {
    const droneData = useDroneData();
    const [map, setMap] = useState(null);
    const markerRef = useRef(null);

    useEffect(() => {
        if (droneData.latitude && droneData.longitude && markerRef.current) {
            const newLatLng = new L.LatLng(droneData.latitude, droneData.longitude);
            markerRef.current.setLatLng(newLatLng);
            if (map) {
                map.panTo(newLatLng);
            }
        }
    }, [droneData, map]);

    return (
        <MapContainer 
            center={[droneData.latitude, droneData.longitude]} 
            zoom={13} 
            whenCreated={setMap}
            style={{ width: '100%', height: '400px' }}>
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; OpenStreetMap contributors'
            />
            <Marker ref={markerRef} position={[droneData.latitude, droneData.longitude]} icon={customIcon}>
                <Popup>Drone's Current Location</Popup>
            </Marker>
        </MapContainer>
    );
}

export default MyMapComponent;
