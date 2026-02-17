"use client";
import { useState, useEffect } from 'react';
import { 
    Gauge, 
    Wind, 
    Zap, 
    Thermometer, 
    Navigation, 
} from 'lucide-react';
import { FlightDataPoint } from '../../hooks/useFlightData';

interface Props {
    data: FlightDataPoint | null;
    onHoverMetric: (metric: string | null) => void;
}

// Helper Card Component
const MetricCard = ({ 
    label, 
    value, 
    unit, 
    icon: Icon, 
    color, 
    metricKey, 
    onHover 
}: any) => (
    <div 
        className="bg-slate-900/90 backdrop-blur-md border border-slate-800 p-4 rounded-xl flex flex-col justify-between hover:border-slate-600 transition-all cursor-crosshair group relative overflow-hidden shadow-xl"
        onMouseEnter={() => onHover(metricKey)}
        onMouseLeave={() => onHover(null)}
    >
        {/* Glow Effect */}
        <div className={`absolute top-0 right-0 p-8 opacity-5 group-hover:opacity-10 transition-opacity bg-[radial-gradient(circle_at_center,var(--tw-gradient-stops))] from-${color}-500 to-transparent`} />

        <div className="flex items-center gap-2 mb-2">
            <Icon size={16} className={`text-${color}-400`} />
            <span className="text-[10px] font-bold uppercase tracking-wider text-slate-500">{label}</span>
        </div>
        
        <div className="flex items-baseline gap-1 relative z-10">
            <span className="text-3xl font-black font-mono text-white tracking-tight">
                {value}
            </span>
            <span className="text-xs font-bold text-slate-500">{unit}</span>
        </div>
    </div>
);

export default function SlideInTelemetry({ data, onHoverMetric }: Props) {
    const [isVisible, setIsVisible] = useState(false);

    // --- 1. MOUSE TRACKING LOGIC ---
    useEffect(() => {
        const handleMouseMove = (e: MouseEvent) => {
            // Activation Zone: Left 300px of the screen (covers the card width + padding)
            // If mouse is within 300px of left edge, SHOW it.
            if (e.clientX < 300) {
                setIsVisible(true);
            } else {
                setIsVisible(false);
            }
        };

        window.addEventListener('mousemove', handleMouseMove);
        return () => window.removeEventListener('mousemove', handleMouseMove);
    }, []);
    
    // --- 2. SAFE G-FORCE CALCULATION (NaN Fix) ---
    let gForceDisplay = "0.00";
    if (data) {
        // Fallback to 0 if accel_z is missing or NaN
        const az = isNaN(Number(data.accel_z)) ? 0 : Number(data.accel_z);
        gForceDisplay = Math.abs(az).toFixed(2);
    }

    return (
        <div 
            className={`fixed left-4 top-24 bottom-24 w-[240px] flex flex-col justify-center z-50 transition-transform duration-300 ease-out ${
                isVisible ? 'translate-x-0' : '-translate-x-[200%]'
            }`}
        >
            <div className="flex flex-col gap-3">
                
                {/* ALTITUDE */}
                <MetricCard 
                    label="Altitude" 
                    value={data?.altitude?.toFixed(1) || "0.0"} 
                    unit="M" 
                    icon={Navigation} 
                    color="blue" 
                    metricKey="alt"
                    onHover={onHoverMetric} 
                />

                {/* VELOCITY */}
                <MetricCard 
                    label="Velocity" 
                    value={data?.velocity?.toFixed(1) || "0.0"} 
                    unit="M/S" 
                    icon={Wind} 
                    color="emerald" 
                    metricKey="vel"
                    onHover={onHoverMetric} 
                />

                {/* G-FORCE */}
                <MetricCard 
                    label="G-Force" 
                    value={gForceDisplay} 
                    unit="G" 
                    icon={Zap} 
                    color="rose" 
                    metricKey="acc"
                    onHover={onHoverMetric} 
                />

                {/* PRESSURE */}
                <MetricCard 
                    label="Pressure" 
                    value={( (data?.pressure || 0) / 1000 ).toFixed(1)} 
                    unit="kPa" 
                    icon={Gauge} 
                    color="amber" 
                    metricKey="pres"
                    onHover={onHoverMetric} 
                />

                {/* TEMP */}
                <MetricCard 
                    label="Temp" 
                    value={data?.temp?.toFixed(1) || "0.0"} 
                    unit="°C" 
                    icon={Thermometer} 
                    color="indigo" 
                    metricKey="temp"
                    onHover={onHoverMetric} 
                />

            </div>
        </div>
    );
}