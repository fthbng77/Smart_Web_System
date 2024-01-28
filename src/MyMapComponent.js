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

function localToGlobal(localX, localY, homeLatitude, homeLongitude) {
    const metersPerLat = 111320;
    const metersPerLng = 40075000 * Math.cos(homeLatitude * Math.PI / 180) / 360;

    const deltaLat = localY / metersPerLat;
    const deltaLng = localX / metersPerLng;

    return {
        latitude: homeLatitude + deltaLat,
        longitude: homeLongitude + deltaLng
    };
}

function useDroneData(homeLat, homeLng) {
    const [data, setData] = useState({
        latitude: homeLat, 
        longitude: homeLng,
    });

    useEffect(() => {
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
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

        const poseTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/local_position/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        poseTopic.subscribe(function (message) {
            const globalCoords = localToGlobal(message.pose.position.x, message.pose.position.y, homeLat, homeLng);
            setData({
                latitude: globalCoords.latitude,
                longitude: globalCoords.longitude
            });
        });

        return () => {
            poseTopic.unsubscribe();
        };
    }, [homeLat, homeLng]);

    return data;
}

function MyMapComponent() {
    const homeLatitude = 41.016593;
    const homeLongitude = 28.951300;

    const droneData = useDroneData(homeLatitude, homeLongitude);
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
                <Popup>Drones Current Location</Popup>
            </Marker>
        </MapContainer>
    );
}

export default MyMapComponent;