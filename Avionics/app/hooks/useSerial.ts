"use client";
import { useState, useRef } from 'react';

// Maps the 101-byte Hex payload to your TypeScript Interface
// OFFSETS based on: Arjuna FC/include/flight_data.h (Packed Struct)
const parseHexToRocketState = (hex: string) => {
    // 1. Convert Hex String to Byte Array
    const matches = hex.match(/.{1,2}/g);
    if (!matches) return null;
    const bytes = new Uint8Array(matches.map(byte => parseInt(byte, 16)));
    
    // Safety check for packet size
    if (bytes.length < 101) return null;

    const view = new DataView(bytes.buffer);
    
    return {
        timestamp_ms:      view.getUint32(0, true), // Little Endian
        fsm_state:         view.getUint8(4),
        
        // Estimator
        altitude:          view.getFloat32(5, true),
        velocity:          view.getFloat32(9, true),
        acc_z_earth:       view.getFloat32(13, true),
        
        // Control
        airbrake_ext:      view.getFloat32(17, true),
        
        // Physics
        mach:              view.getFloat32(21, true),
        tas:               view.getFloat32(25, true),
        tilt:              view.getFloat32(29, true),
        
        // Raw IMU
        ax: view.getFloat32(33, true), ay: view.getFloat32(37, true), az: view.getFloat32(41, true),
        gx: view.getFloat32(45, true), gy: view.getFloat32(49, true), gz: view.getFloat32(53, true),
        mx: view.getFloat32(57, true), my: view.getFloat32(61, true), mz: view.getFloat32(65, true),
        
        // Environment
        pres:              view.getFloat32(69, true),
        temp:              view.getFloat32(73, true),
        diff_pres:         view.getFloat32(77, true),
        
        // Continuity
        cont_drogue:       view.getUint8(81),
        cont_main:         view.getUint8(82),
        
        // GPS (SCALING FIX: Divide by 10,000,000)
        gps_lat:           view.getInt32(83, true) / 10000000.0, 
        gps_lon:           view.getInt32(87, true) / 10000000.0,
        gps_alt:           view.getFloat32(91, true),
        gps_itow:          view.getUint32(95, true),
        gps_sats:          view.getUint8(99),
        gps_fix:           view.getUint8(100)
    };
};

export const useSerial = (onData: any) => {
    const [isConnected, setIsConnected] = useState(false);
    const portRef = useRef<any>(null);
    const readerRef = useRef<any>(null);

    const processLine = (line: string) => {
        const clean = line.trim();
        // Detect RYLR Packet: "+RCV=..." OR "AT+SEND=..." (Local Echo)
        // FC sends: AT+SEND=0,202,<HEX_DATA>
        if (clean.includes(',')) {
            const parts = clean.split(',');
            // The Hex payload is always the last part of the string
            const hexPayload = parts[parts.length - 1];
            
            try {
                const data = parseHexToRocketState(hexPayload);
                if (data) {
                    onData(
                        data.timestamp_ms / 1000, 
                        data.altitude, 
                        data.velocity, 
                        data.acc_z_earth, 
                        data.gps_lat, 
                        data.gps_lon, 
                        data.fsm_state,
                        {
                            airbrake_extension: data.airbrake_ext,
                            mach_number: data.mach,
                            airspeed_tas: data.tas,
                            tilt_angle: data.tilt,
                            pressure: data.pres,
                            temp: data.temp,
                            diff_pressure: data.diff_pres,
                            drogue_continuity: data.cont_drogue,
                            main_continuity: data.cont_main,
                            gps_alt: data.gps_alt,
                            sats: data.gps_sats,
                            fix: data.gps_fix,
                            // Mapped IMU for raw plots
                            ax: data.ax, ay: data.ay, az: data.az,
                            gx: data.gx, gy: data.gy, gz: data.gz,
                            mx: data.mx, my: data.my, mz: data.mz
                        }
                    );
                }
            } catch (e) {
                // Ignore parse errors from non-telemetry messages
            }
        }
    };

    const connect = async () => {
        try {
            const port = await (navigator as any).serial.requestPort();
            // FC is configured to 115200 on UART0 (USB) 
            // Note: If using LoRa dongle, ensure this matches dongle baud (usually 115200 or 9600)
            await port.open({ baudRate: 115200 }); 
            portRef.current = port;
            setIsConnected(true);

            const decoder = new TextDecoderStream();
            port.readable.pipeTo(decoder.writable);
            const reader = decoder.readable.getReader();
            readerRef.current = reader;

            let buffer = "";
            while (true) {
                const { value, done } = await reader.read();
                if (done) break;
                buffer += value;
                const lines = buffer.split('\n');
                buffer = lines.pop() || ""; // Keep incomplete line
                lines.forEach(processLine);
            }
        } catch (err) {
            console.error("Serial Connection Failed:", err);
            setIsConnected(false);
        }
    };

    const disconnect = async () => {
        if (readerRef.current) await readerRef.current.cancel();
        if (portRef.current) await portRef.current.close();
        setIsConnected(false);
    };

    return { isConnected, connect, disconnect };
};