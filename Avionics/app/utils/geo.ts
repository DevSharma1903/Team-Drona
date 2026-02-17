// utils/geo.ts

// Converts GPS (Lat/Lon/Alt) to 3D World Coordinates (Meters) relative to an Origin point.
export const getRelativePos = (
    lat: number, 
    lon: number, 
    alt: number, 
    origin: { lat: number, lon: number }
): [number, number, number] => {
    // Earth Radius approximation
    const R = 6371000; 
    
    // Convert to Radians
    const latRad = lat * (Math.PI / 180);
    const lonRad = lon * (Math.PI / 180);
    const originLatRad = origin.lat * (Math.PI / 180);
    const originLonRad = origin.lon * (Math.PI / 180);

    // Calculate displacement in meters (Simplified Equirectangular projection for local area)
    // x = East/West (Longitude)
    const x = R * (lonRad - originLonRad) * Math.cos(originLatRad);
    
    // z = North/South (Latitude) - Negative Z is North in Three.js standard
    const z = -R * (latRad - originLatRad);

    // y = Altitude (Up)
    return [x, alt, z]; 
};