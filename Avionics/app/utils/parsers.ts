// --- CSV PARSING (Standard) ---
export type RawCSV = {
    headers: string[];
    columns: number[][];
};

export function parseGenericCSV(csvText: string): RawCSV {
    const lines = csvText.trim().split(/\r?\n/);
    if (lines.length < 2) return { headers: [], columns: [] };

    const headers = lines[0].split(',').map(h => h.trim());
    const numCols = headers.length;
    const columns: number[][] = Array.from({ length: numCols }, () => []);

    for (let i = 1; i < lines.length; i++) {
        const row = lines[i].split(',').map(v => parseFloat(v.trim()));
        if (row.length === numCols && !row.some(isNaN)) {
            row.forEach((val, colIdx) => columns[colIdx].push(val));
        }
    }

    return { headers, columns };
}

// PACKET SIZE = 101 Bytes
// 4 (Time) + 1 (State) + 12 (Est) + 4 (Airbrake) + 12 (Physics) + 36 (IMU) + 12 (Env) + 2 (Cont) + 18 (GPS)
export const PACKET_SIZE = 101;

export interface RocketPacket {
    timestamp: number;
    state: number;
    // Estimator
    altitude: number;
    velocity: number;
    accel_z: number;
    // Control
    airbrake_extension: number;
    // Physics
    mach_number: number;
    airspeed_tas: number;
    tilt_angle: number;
    // IMU
    ax: number; ay: number; az: number;
    gx: number; gy: number; gz: number;
    mx: number; my: number; mz: number;
    // Environment
    pressure: number;
    temp: number;
    diff_pressure: number;
    // Continuity
    drogue_continuity: number;
    main_continuity: number;
    // GPS
    lat: number;
    lon: number;
    gps_alt: number;
    gps_iTOW: number;
    sats: number;
    fix: number;
}

export function parseRocketPacket(buffer: ArrayBuffer): RocketPacket | null {
    if (buffer.byteLength < PACKET_SIZE) return null;
    const view = new DataView(buffer);
    let offset = 0;

    const packet: any = {};

    // 1. Header (Timestamp & State)
    packet.timestamp = view.getUint32(offset, true); offset += 4;
    packet.state = view.getUint8(offset); offset += 1;

    // 2. Estimator State
    packet.altitude = view.getFloat32(offset, true); offset += 4;
    packet.velocity = view.getFloat32(offset, true); offset += 4;
    packet.accel_z = view.getFloat32(offset, true); offset += 4;

    // 3. Control System
    packet.airbrake_extension = view.getFloat32(offset, true); offset += 4;

    // 4. Physics Data
    packet.mach_number = view.getFloat32(offset, true); offset += 4;
    packet.airspeed_tas = view.getFloat32(offset, true); offset += 4;
    packet.tilt_angle = view.getFloat32(offset, true); offset += 4;

    // 5. Raw Sensors (IMU)
    packet.ax = view.getFloat32(offset, true); offset += 4;
    packet.ay = view.getFloat32(offset, true); offset += 4;
    packet.az = view.getFloat32(offset, true); offset += 4;
    
    packet.gx = view.getFloat32(offset, true); offset += 4;
    packet.gy = view.getFloat32(offset, true); offset += 4;
    packet.gz = view.getFloat32(offset, true); offset += 4;
    
    packet.mx = view.getFloat32(offset, true); offset += 4;
    packet.my = view.getFloat32(offset, true); offset += 4;
    packet.mz = view.getFloat32(offset, true); offset += 4;

    // 6. Environment
    packet.pressure = view.getFloat32(offset, true); offset += 4;
    packet.temp = view.getFloat32(offset, true); offset += 4;
    packet.diff_pressure = view.getFloat32(offset, true); offset += 4;

    // 7. Continuity (NEW)
    packet.drogue_continuity = view.getUint8(offset); offset += 1;
    packet.main_continuity = view.getUint8(offset); offset += 1;

    // 8. GPS
    packet.lat = view.getInt32(offset, true) / 1e7; offset += 4; // Int32 scaled
    packet.lon = view.getInt32(offset, true) / 1e7; offset += 4; // Int32 scaled
    packet.gps_alt = view.getFloat32(offset, true); offset += 4;
    packet.gps_iTOW = view.getUint32(offset, true); offset += 4;
    packet.sats = view.getUint8(offset); offset += 1;
    packet.fix = view.getUint8(offset); offset += 1;

    return packet as RocketPacket;
}