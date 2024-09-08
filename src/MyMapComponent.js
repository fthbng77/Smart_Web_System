import React, { useState, useEffect, useRef } from 'react';
import PropTypes from 'prop-types';
import { MapContainer, TileLayer, Marker, Popup, useMap } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import ROSLIB from 'roslib';

const customIcon = L.icon({
    iconUrl: '/marker.png',
    iconSize: [48, 48],
    iconAnchor: [24, 48],
    popupAnchor: [0, -48]
});

const customIcon2 = L.icon({
    iconUrl: '/marker2.png',
    iconSize: [48, 48],
    iconAnchor: [24, 48],
    popupAnchor: [0, -48]
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

function MyMapComponent() {
    const homeLatitude = 41.016593;
    const homeLongitude = 28.951300;

    const [gpsData, setGpsData] = useState({
        latitude: homeLatitude,
        longitude: homeLongitude,
    });

    const [visionData, setVisionData] = useState({
        latitude: homeLatitude,
        longitude: homeLongitude,
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

        const gpsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/local_position/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        gpsTopic.subscribe(function (message) {
            console.log('Received GPS message:', message);
            const globalCoords = localToGlobal(message.pose.position.x, message.pose.position.y, homeLatitude, homeLongitude);
            setGpsData({
                latitude: globalCoords.latitude,
                longitude: globalCoords.longitude
            });
        });

        const visionTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/mavros/vision_pose/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        visionTopic.subscribe(function (message) {
            console.log('Received Vision message:', message);
            const globalCoords = localToGlobal(message.pose.position.x, message.pose.position.y, homeLatitude, homeLongitude);
            setVisionData({
                latitude: globalCoords.latitude,
                longitude: globalCoords.longitude
            });
        });

        return () => {
            gpsTopic.unsubscribe();
            visionTopic.unsubscribe();
        };
    }, [homeLatitude, homeLongitude]);

    const [map, setMap] = useState(null);
    const [showVisualOdometry, setShowVisualOdometry] = useState(false);
    const markerRef1 = useRef(null);
    const markerRef2 = useRef(null);

    useEffect(() => {
        console.log('GPS Data:', gpsData);
        if (gpsData.latitude && gpsData.longitude && markerRef1.current) {
            const newLatLng = new L.LatLng(gpsData.latitude, gpsData.longitude);
            markerRef1.current.setLatLng(newLatLng);
            if (map) {
                map.panTo(newLatLng);
            }
        }
    }, [gpsData, map]);
    
    useEffect(() => {
        console.log('Vision Data:', visionData);
        if (visionData.latitude && visionData.longitude && markerRef2.current) {
            const newLatLng = new L.LatLng(visionData.latitude, visionData.longitude);
            markerRef2.current.setLatLng(newLatLng);
        }
    }, [visionData]);

    const handleButtonClick = () => {
        setShowVisualOdometry(!showVisualOdometry);
    };

    return (
        <div style={{ width: '100%', height: '100%' }}>
            <button 
                onClick={handleButtonClick} 
                className="map-button"
            >
                {showVisualOdometry ? "GPS'siz veriyi gizle" : "GPS'siz veriyi göster"}
            </button>
            <MapContainer 
                center={[gpsData.latitude, gpsData.longitude]} 
                zoom={15} 
                whenCreated={setMap}
                style={{ width: '100%', height: '100%' }}
                zoomControl={false}
            >
                <TileLayer
                    url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                    attribution='&copy; OpenStreetMap contributors'
                />
                <Marker ref={markerRef1} position={[gpsData.latitude, gpsData.longitude]} icon={customIcon}>
                    <Popup>Drone 1 - GPS</Popup>
                </Marker>
                {showVisualOdometry && (
                    <Marker ref={markerRef2} position={[visionData.latitude, visionData.longitude]} icon={customIcon2}>
                        <Popup>Drone 2 - Visual Odometry</Popup>
                    </Marker>
                )}
                <ZoomControlComponent position="topright" />
            </MapContainer>
        </div>
    );
}

function ZoomControlComponent({ position }) {
    const map = useMap();
    useEffect(() => {
        if (map && map.zoomControl) {
            map.zoomControl.remove();
            L.control.zoom({ position: position }).addTo(map);
        }
    }, [map, position]);

    return null;
}

ZoomControlComponent.propTypes = {
    position: PropTypes.string.isRequired,
};

export default MyMapComponent;
