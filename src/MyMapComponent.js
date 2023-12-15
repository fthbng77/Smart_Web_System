import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMap } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import { useDroneData } from './DroneData';

const customIcon = L.icon({
  iconUrl: '/marker.png',
  iconSize: [48, 48],
  iconAnchor: [16, 32], 
  popupAnchor: [0, -32]   
});
function MyMapComponent() {
    const { droneData } = useDroneData();  

    useEffect(() => {
        const interval = setInterval(() => {
            setPosition(prev => [prev[0], prev[1] + 0.001]);
        }, 2000);

        return () => clearInterval(interval); 
    }, []);

    return (
        <MapContainer center={position} zoom={13} style={{ width: '100%', height: '400px' }}>
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            />
            <MarkerUpdater position={position} />
            <Marker position={position} icon={customIcon}>
                <Popup>
                    Drone'un bulunduğu yer
                </Popup>
            </Marker>
        </MapContainer>
    );
}

function MarkerUpdater({ position }) {
    const map = useMap();

    useEffect(() => {
        map.flyTo(position);  
    }, [position, map]);

    return null;
}

export default MyMapComponent;