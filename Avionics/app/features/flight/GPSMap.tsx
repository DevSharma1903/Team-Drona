"use client";
import { MapContainer, TileLayer, Marker, Popup, useMap, Polyline, Circle } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import { useEffect } from 'react';

// Fix for default Leaflet marker icons in Next.js
const icon = L.icon({
    iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png",
    iconRetinaUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png",
    shadowUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png",
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowSize: [41, 41]
});

interface Props {
    lat: number;
    lon: number;
    // THIS IS THE MISSING PART CAUSING YOUR ERROR
    trajectory?: [number, number][]; 
}

// Helper to auto-center map when props change
function MapUpdater({ lat, lon }: { lat: number, lon: number }) {
    const map = useMap();
    useEffect(() => {
        if (lat !== 0 && lon !== 0) {
            // "flyTo" is smoother than setView for tracking
            map.flyTo([lat, lon], map.getZoom(), { animate: false });
        }
    }, [lat, lon, map]);
    return null;
}

export default function GPSMap({ lat, lon, trajectory = [] }: Props) {
    // Default: Manipal (or your base location)
    const defaultCenter: [number, number] = [13.345103, 74.794628];
    
    // If no live data (0,0), use default. Otherwise use live data.
    const center: [number, number] = (lat === 0 && lon === 0) ? defaultCenter : [lat, lon];
    const hasFix = lat !== 0 && lon !== 0;

    return (
        <MapContainer 
            center={center} 
            zoom={16} 
            style={{ height: "100%", width: "100%", background: "#0f172a" }}
            zoomControl={false}
        >
            <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                className="map-tiles"
            />
            
            <MapUpdater lat={lat} lon={lon} />

            {/* DRAW THE TRAJECTORY LINE */}
            {trajectory.length > 1 && (
                <Polyline 
                    positions={trajectory} 
                    pathOptions={{ color: '#f59e0b', weight: 4, opacity: 0.8 }} 
                />
            )}

            {/* Base Station Marker */}
            <Marker position={defaultCenter} icon={icon}>
                <Popup>
                    <div className="text-slate-900 font-bold">BASE STATION</div>
                </Popup>
            </Marker>

            {/* Rocket Marker */}
            {hasFix && (
                <>
                    <Marker position={[lat, lon]} icon={icon}>
                        <Popup>
                            <div className="text-slate-900 font-bold">ROCKET LIVE</div>
                            <div className="text-xs text-slate-600">
                                {lat.toFixed(6)}, {lon.toFixed(6)}
                            </div>
                        </Popup>
                    </Marker>
                    <Circle 
                        center={[lat, lon]} 
                        pathOptions={{ fillColor: 'blue', color: 'blue', opacity: 0.2 }} 
                        radius={20} 
                    />
                </>
            )}
        </MapContainer>
    );
}