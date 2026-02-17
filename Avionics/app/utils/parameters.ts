export type Unit = { label: string; multiplier: number }; 

export type ParameterType = 
    | "TIMESTAMP" 
    | "ALTITUDE" 
    | "PRESSURE" | "TEMPERATURE" | "THRUST" 
    | "ACCEL_X" | "ACCEL_Y" | "ACCEL_Z" | "ACCEL_NET"
    | "VEL_X" | "VEL_Y" | "VEL_Z" | "VEL_NET"
    | "GYRO_X" | "GYRO_Y" | "GYRO_Z"
    | "MAG_X" | "MAG_Y" | "MAG_Z"
    // NEW: GPS Types
    | "GPS_LAT" | "GPS_LON"
    | "IGNORE";

// Grouping Logic
export const PARAM_GROUPS: Partial<Record<ParameterType, string>> = {
    ACCEL_X: "Acceleration", ACCEL_Y: "Acceleration", ACCEL_Z: "Acceleration", ACCEL_NET: "Acceleration",
    VEL_X: "Velocity", VEL_Y: "Velocity", VEL_Z: "Velocity", VEL_NET: "Velocity",
    GYRO_X: "Gyroscope", GYRO_Y: "Gyroscope", GYRO_Z: "Gyroscope",
    MAG_X: "Magnetometer", MAG_Y: "Magnetometer", MAG_Z: "Magnetometer",
    ALTITUDE: "Altitude",
    PRESSURE: "Pressure",
    TEMPERATURE: "Temperature",
    THRUST: "Thrust",
    GPS_LAT: "GPS", GPS_LON: "GPS"
};

export const PARAM_DEFINITIONS: Record<ParameterType, { label: string; units: Record<string, number> }> = {
    TIMESTAMP: { label: "Timestamp / Time", units: { "Seconds (s)": 1, "Milliseconds (ms)": 0.001, "Microseconds (µs)": 0.000001 } },
    ALTITUDE: { label: "Altitude", units: { "Meters (m)": 1, "Feet (ft)": 0.3048, "Kilometers (km)": 1000 } },
    PRESSURE: { label: "Pressure", units: { "Pascals (Pa)": 1, "Hectopascals (hPa)": 100, "Bar": 100000, "PSI": 6894.76 } },
    TEMPERATURE: { label: "Temperature", units: { "Celsius (°C)": 1, "Fahrenheit (°F)": 1, "Kelvin (K)": 1 } },
    THRUST: { label: "Thrust", units: { "Newtons (N)": 1, "kgf": 9.80665, "lbf": 4.44822 } },
    
    ACCEL_X: { label: "Accel X", units: { "m/s²": 1, "G": 9.80665 } },
    ACCEL_Y: { label: "Accel Y", units: { "m/s²": 1, "G": 9.80665 } },
    ACCEL_Z: { label: "Accel Z", units: { "m/s²": 1, "G": 9.80665 } },
    ACCEL_NET: { label: "Accel Net", units: { "m/s²": 1, "G": 9.80665 } },
    
    VEL_X: { label: "Vel X", units: { "m/s": 1, "km/h": 0.277778 } },
    VEL_Y: { label: "Vel Y", units: { "m/s": 1, "km/h": 0.277778 } },
    VEL_Z: { label: "Vel Z", units: { "m/s": 1, "km/h": 0.277778 } },
    VEL_NET: { label: "Vel Net", units: { "m/s": 1, "km/h": 0.277778 } },

    GYRO_X: { label: "Gyro X", units: { "deg/s": 1, "rad/s": 57.2958 } },
    GYRO_Y: { label: "Gyro Y", units: { "deg/s": 1, "rad/s": 57.2958 } },
    GYRO_Z: { label: "Gyro Z", units: { "deg/s": 1, "rad/s": 57.2958 } },

    MAG_X: { label: "Mag X", units: { "µT": 1, "Gauss": 100 } },
    MAG_Y: { label: "Mag Y", units: { "µT": 1, "Gauss": 100 } },
    MAG_Z: { label: "Mag Z", units: { "µT": 1, "Gauss": 100 } },

    // ... (rest of the file remains the same)

    // UPDATED GPS DEFINITIONS
    GPS_LAT: { 
        label: "GPS Latitude", 
        units: { 
            "Degrees (°)": 1, 
            "Raw Int (1e7)": 1e-7 // This multiplies by 0.0000001
        } 
    },
    GPS_LON: { 
        label: "GPS Longitude", 
        units: { 
            "Degrees (°)": 1, 
            "Raw Int (1e7)": 1e-7 // This multiplies by 0.0000001
        } 
    },

    IGNORE: { label: "Ignore", units: {} }
};