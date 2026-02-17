import { useState, useRef, useCallback } from 'react';
import uPlot from 'uplot';

// --- 1. DATA INTERFACE ---
// Matches the "RocketState" struct from your firmware
export interface FlightDataPoint {
    timestamp: number;
    
    // Estimator
    altitude: number;
    velocity: number;
    accel_z: number;
    
    // GPS
    lat: number;
    lon: number;
    gps_alt?: number;
    gps_iTOW?: number;
    sats?: number;
    fix?: number;
    
    // State
    state: number;

    // --- Extended Data ---
    airbrake_extension?: number;
    mach_number?: number;
    airspeed_tas?: number;
    tilt_angle?: number;

    // IMU
    ax?: number; ay?: number; az?: number;
    gx?: number; gy?: number; gz?: number;
    mx?: number; my?: number; mz?: number;

    // Environment
    pressure?: number;
    temp?: number;
    diff_pressure?: number;

    // Continuity
    drogue_continuity?: number;
    main_continuity?: number;
}

// --- 2. HOOK ---
export const useFlightData = () => {
    // State for the "Live" dashboard numbers (Gauge/Text)
    const [currentPacket, setCurrentPacket] = useState<FlightDataPoint | null>(null);

    // Refs for history (Charts) to avoid re-rendering loop issues
    // We map the new fields to specific indices for uPlot
    /*
       INDEX MAPPING:
       0: Time         11: Mag X
       1: Altitude     12: Mag Y
       2: Velocity     13: Mag Z
       3: Accel Z      14: Pressure (Pa)
       4: Airbrake     15: Temp (C)
       5: Mach         16: Diff Press
       6: Airspeed     17: Drogue Cont
       7: Tilt         18: Main Cont
       8: Gyro X       19: GPS Alt
       9: Gyro Y       20: Sats
       10: Gyro Z      21: Fix
    */
    const plotDataRef = useRef<number[][]>([
        [], [], [], [], [], [], [], [], [], [], [], 
        [], [], [], [], [], [], [], [], [], [], []
    ]);

    // State for the Charts (Updated less frequently if needed)
    const [plotData, setPlotData] = useState<uPlot.AlignedData>(plotDataRef.current as unknown as uPlot.AlignedData);

    // Trajectory for 3D Map [lat, alt, lon]
    const trajectoryRef = useRef<[number, number, number][]>([]);
    const [trajectory, setTrajectory] = useState<[number, number, number][]>([]);

    // --- ADD DATA FUNCTION ---
    const addData = useCallback((
        t: number,
        alt: number,
        vel: number,
        acc: number,
        lat: number,
        lon: number,
        state: number,
        extra: Partial<FlightDataPoint> = {}
    ) => {
        // 1. Update Current Packet (Live UI)
        const point: FlightDataPoint = {
            timestamp: t,
            altitude: alt,
            velocity: vel,
            accel_z: acc,
            lat,
            lon,
            state,
            ...extra
        };
        setCurrentPacket(point);

        // 2. Update Plot History (Charts)
        const p = plotDataRef.current;
        p[0].push(t);
        p[1].push(alt);
        p[2].push(vel);
        p[3].push(acc);
        
        // Extended Fields (Default to 0 if missing)
        p[4].push(extra.airbrake_extension ?? 0);
        p[5].push(extra.mach_number ?? 0);
        p[6].push(extra.airspeed_tas ?? 0);
        p[7].push(extra.tilt_angle ?? 0);
        p[8].push(extra.gx ?? 0);
        p[9].push(extra.gy ?? 0);
        p[10].push(extra.gz ?? 0);
        p[11].push(extra.mx ?? 0);
        p[12].push(extra.my ?? 0);
        p[13].push(extra.mz ?? 0);
        p[14].push(extra.pressure ?? 0);
        p[15].push(extra.temp ?? 0);
        p[16].push(extra.diff_pressure ?? 0);
        p[17].push(extra.drogue_continuity ?? 0);
        p[18].push(extra.main_continuity ?? 0);
        p[19].push(extra.gps_alt ?? 0);
        p[20].push(extra.sats ?? 0);
        p[21].push(extra.fix ?? 0);

        // Trigger Chart Update
        setPlotData([...p] as unknown as uPlot.AlignedData);

        // 3. Update Trajectory (3D Map)
        // Filter out (0,0) coordinates which happen before GPS lock
        if (Math.abs(lat) > 0.1 && Math.abs(lon) > 0.1) {
            trajectoryRef.current.push([lat, alt, lon]);
            
            // Optimization: Update map state every 5th packet to save React cycles
            if (trajectoryRef.current.length % 5 === 0) {
                setTrajectory([...trajectoryRef.current]);
            }
        }
    }, []);

    // --- RESET FUNCTION ---
    const resetData = useCallback(() => {
        // Clear all arrays
        plotDataRef.current = Array(22).fill(null).map(() => []);
        trajectoryRef.current = [];
        
        // Reset States
        setPlotData(plotDataRef.current as unknown as uPlot.AlignedData);
        setTrajectory([]);
        setCurrentPacket(null);
    }, []);

    // --- DOWNLOAD CSV ---
    const downloadLog = useCallback(() => {
        if (plotDataRef.current[0].length === 0) return alert("No data to download.");
    
        // UPDATED HEADERS TO MATCH NEW PACKET STRUCTURE
        const headers = [
            "Time_s",           // 0
            "Altitude_m",       // 1
            "Velocity_ms",      // 2
            "AccelZ_G",         // 3
            "Airbrake_pct",     // 4
            "Mach",             // 5
            "Airspeed_TAS",     // 6
            "Tilt_deg",         // 7
            "GyroX_dps",        // 8
            "GyroY_dps",        // 9
            "GyroZ_dps",        // 10
            "MagX_uT",          // 11
            "MagY_uT",          // 12
            "MagZ_uT",          // 13
            "Pressure_Pa",      // 14
            "Temp_C",           // 15
            "DiffPress_Pa",     // 16
            "Drogue_Cont",      // 17
            "Main_Cont",        // 18
            "GPS_Alt_m",        // 19
            "Sats",             // 20
            "GPS_Fix"           // 21
        ];
    
        // Transpose data for CSV
        const rows = plotDataRef.current[0].map((_, i) => 
            plotDataRef.current.map(col => col[i]).join(",")
        );
    
        const csvContent = "data:text/csv;charset=utf-8," 
            + headers.join(",") + "\n" 
            + rows.join("\n");
    
        const encodedUri = encodeURI(csvContent);
        const link = document.createElement("a");
        link.setAttribute("href", encodedUri);
        link.setAttribute("download", `flight_log_${new Date().toISOString()}.csv`);
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
    }, []);

    return {
        currentPacket,
        plotData,
        trajectory,
        addData,
        resetData,
        downloadLog
    };
};