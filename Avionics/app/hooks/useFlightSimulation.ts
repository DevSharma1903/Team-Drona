import { useState, useCallback, useRef } from 'react';
import { FlightDataPoint } from './useFlightData';

export type SimScenario = 'REAL_FLIGHT' | 'DATASET' | 'NOMINAL' | 'UNSTABLE';

type AddDataFn = (
    t: number, alt: number, vel: number, acc: number, 
    lat: number, lon: number, state: number, 
    extra?: Partial<FlightDataPoint>
) => void;

export const useFlightSimulation = (addData: AddDataFn) => {
    const [isSimulating, setIsSimulating] = useState(false);
    const timerRef = useRef<NodeJS.Timeout | null>(null);

    // Simulation State
    const simState = useRef({
        startTime: 0,
        altitude: 0,
        velocity: 0, // Vertical Velocity
        accel: 0,
        
        // COORDINATES (Manipal, India)
        lat: 13.3451, 
        lon: 74.7946,
        
        // HORIZONTAL MOTION (Ground Speed)
        velNorth: 0, // m/s
        velEast: 0,  // m/s
        
        state: 0, // 0=Pad, 1=Boost, 2=Coast, 3=Drogue, 4=Main, 5=Landed
        airbrake: 0,
        maxAlt: 0
    });

    const startSimulation = useCallback((scenario: SimScenario) => {
        if (timerRef.current) clearInterval(timerRef.current);
        
        setIsSimulating(true);
        simState.current = {
            startTime: Date.now(),
            altitude: 0,
            velocity: 0,
            accel: 0,
            lat: 13.3451, 
            lon: 74.7946,
            velNorth: 0,
            velEast: 0,
            state: 0,
            airbrake: 0,
            maxAlt: 0
        };

        const FREQUENCY_HZ = 20; 
        const DT = 1.0 / FREQUENCY_HZ;

        // PHYSICS CONSTANTS
        const GRAVITY = 9.81;
        const LAUNCH_DELAY = 3.0;
        const BURN_DURATION = 3.5; 
        const MAIN_DEPLOY_ALT = 400; 

        // TRAJECTORY SETTINGS
        // Launch Angle: Rocket leans slightly East (Positive Lon)
        const LAUNCH_ANGLE_ACCEL = 4.0; // Accelerate 4m/s² East during boost
        
        // Wind: Pushes North-West during descent
        const WIND_SPEED_NORTH = 5.0; 
        const WIND_SPEED_EAST = -3.0; 

        timerRef.current = setInterval(() => {
            const now = Date.now();
            const t = (now - simState.current.startTime) / 1000.0;
            const s = simState.current;

            let thrust = 0;
            let dragCoeff = 0.005; 

            // --- 1. STATE MACHINE ---
            if (t < LAUNCH_DELAY) {
                s.state = 0;
            } 
            else if (t < (LAUNCH_DELAY + BURN_DURATION)) {
                s.state = 1; // BOOST
                thrust = 180.0; 
                dragCoeff = 0.02;
                
                // Horizontal Acceleration (Angled Launch)
                s.velEast += LAUNCH_ANGLE_ACCEL * DT; 
                s.velNorth += (Math.random() - 0.5) * DT; // Minor wobble
            } 
            else if (s.velocity > 0) {
                s.state = 2; // COAST
                thrust = 0;
                dragCoeff = 0.01; 
                // Momentum carries it horizontally, air resistance slows it slightly
                s.velEast *= 0.999;
                s.velNorth *= 0.999;
            } 
            else if (s.altitude > MAIN_DEPLOY_ALT) {
                s.state = 3; // DROGUE
                thrust = 0;
                dragCoeff = 0.03; 
                
                // Drift with Wind (Lerp towards wind speed)
                s.velEast = s.velEast * 0.98 + WIND_SPEED_EAST * 0.02;
                s.velNorth = s.velNorth * 0.98 + WIND_SPEED_NORTH * 0.02;
            } 
            else if (s.altitude > 0) {
                s.state = 4; // MAIN
                thrust = 0;
                dragCoeff = 0.5; 
                
                // Stronger Wind Effect (Chute acts as a sail)
                s.velEast = s.velEast * 0.95 + WIND_SPEED_EAST * 0.05;
                s.velNorth = s.velNorth * 0.95 + WIND_SPEED_NORTH * 0.05;
            } 
            else {
                s.state = 5; // LANDED
            }

            // --- 2. VERTICAL KINEMATICS ---
            if (s.state === 0 || s.state === 5) {
                s.velocity = 0;
                s.accel = 0;
                s.velEast = 0;
                s.velNorth = 0;
                if (s.state === 5) s.altitude = 0;
            } else {
                const dragForce = dragCoeff * s.velocity * s.velocity;
                const dragDirection = s.velocity > 0 ? -1 : 1; 
                const netForce = thrust - GRAVITY + (dragForce * dragDirection);

                s.accel = netForce;
                s.velocity += s.accel * DT;
                s.altitude += s.velocity * DT;
            }

            // Safety Clamps
            if (s.altitude < 0 && t > 5) s.altitude = 0;
            if (s.altitude > s.maxAlt) s.maxAlt = s.altitude;

            // --- 3. GEOGRAPHIC COORDINATE UPDATES ---
            // Earth Radius ~ 6,371,000 meters
            // 1 deg Lat ~= 111,320 meters
            // 1 deg Lon ~= 111,320 * cos(lat) meters
            
            const metersPerDegLat = 111320;
            const metersPerDegLon = 111320 * Math.cos(s.lat * (Math.PI / 180));

            const dLat = s.velNorth * DT / metersPerDegLat;
            const dLon = s.velEast * DT / metersPerDegLon;

            s.lat += dLat;
            s.lon += dLon;

            // --- 4. SENSOR GENERATION ---
            // IMU Noise
            const vibration = (s.state === 1) ? 5.0 : 0.2; 
            const noise = () => (Math.random() - 0.5) * vibration;

            const ax = noise();
            const ay = noise();
            const az = s.accel + 9.81 + noise(); 
            
            // Rotation Physics
            // Tilt the rocket orientation based on horizontal velocity
            const tiltAngle = -s.velEast * 0.5; // Simple fake tilt
            
            const gx = (s.state === 1) ? Math.sin(t * 10) : noise(); 
            const gy = (s.state === 1) ? 360 : noise(); 
            const gz = tiltAngle; // Lean into the turn

            const pressure = 101325 * Math.pow((1 - 2.25577e-5 * s.altitude), 5.25588);
            const temp = 25 - (0.0065 * s.altitude);

            addData(
                t, 
                s.altitude, 
                s.velocity, 
                s.accel, 
                s.lat, 
                s.lon, 
                s.state,
                {
                    airbrake_extension: 0,
                    pressure: pressure + (Math.random() * 10),
                    temp: temp,
                    gps_alt: s.altitude + 50, // Approx Base Alt
                    sats: 10,
                    fix: 1,
                    ax: Math.round(ax * 100),
                    ay: Math.round(ay * 100),
                    az: Math.round(az * 100),
                    gx: Math.round(gx * 10),
                    gy: Math.round(gy * 10),
                    gz: Math.round(gz * 10),
                    mx: 0, my: 0, mz: 0
                }
            );

        }, 1000 / FREQUENCY_HZ);

    }, [addData]);

    const stopSimulation = useCallback(() => {
        if (timerRef.current) clearInterval(timerRef.current);
        setIsSimulating(false);
    }, []);

    return { isSimulating, startSimulation, stopSimulation };
};