export interface MotorUpdate {
    type: 'DATA' | 'STATUS' | 'ALERT';
    timestamp?: number;
    packetTime?: number;
    thrust?: number;    // A0 (Volts)
    pressure?: number;  // A1 (Volts)
    voltage?: number;   // A2 (Volts)
    valve?: number;     // A3 (Volts)
    continuity?: boolean;
    isSafe?: boolean;
    raw?: string;
}

// --- CALIBRATION CONFIG ---
// Formula: Voltage = ADC * (3.3 / 1023)
const ADC_REF_VOLTAGE = 3.3;
const ADC_RESOLUTION = 1023.0; // 10-bit as per requirements
const ADC_SCALE = ADC_REF_VOLTAGE / ADC_RESOLUTION; 

export function parseMotorSerialLine(line: string): MotorUpdate | null {
    const clean = line.trim();
    if (!clean) return null;

    // Expected Packet: "T:12345,C:1,M:100,200,300,400"
    if (clean.startsWith("T:")) {
        const parts = clean.split(',');

        if (parts.length < 6) return null;

        const mcuTime = parseInt(parts[0].split(':')[1]);
        const contVal = parseInt(parts[1].split(':')[1]);
        
        const rawA0 = parseFloat(parts[2].split(':')[1]);
        const rawA1 = parseFloat(parts[3]);
        const rawA2 = parseFloat(parts[4]);
        const rawA3 = parseFloat(parts[5]);

        return {
            type: 'DATA',
            timestamp: Date.now(),
            packetTime: mcuTime,
            continuity: contVal === 1,
            // Apply 10-bit Voltage Scaling
            thrust:   rawA0 * ADC_SCALE, 
            pressure: rawA1 * ADC_SCALE,
            voltage:  rawA2 * ADC_SCALE,
            valve:    rawA3 * ADC_SCALE
        };
    }

    if (clean.includes("SAFE")) return { type: 'STATUS', isSafe: true };
    if (clean.includes("ARMED") || clean.includes("ARM")) return { type: 'STATUS', isSafe: false };

    return { type: 'ALERT', raw: clean };
}