"use client";
import React, { useMemo } from 'react';
import UPlotChart from '../../components/UPlotChart';
import uPlot from 'uplot';
import { Zap, MapPin, Wind, Activity, Crosshair, Radio, Gauge, Disc } from 'lucide-react';
import { FlightDataPoint } from '../../hooks/useFlightData';

interface Props {
    isOpen: boolean;
    data: FlightDataPoint | null;
    history: uPlot.AlignedData;
    isDark: boolean;
}

// Helper for Digital Badges
const StatusBadge = ({ label, value, unit, icon: Icon, color = "text-slate-400", alert = false }: any) => (
    <div className={`flex flex-col px-3 py-2 bg-slate-900/80 border border-slate-700 rounded backdrop-blur-sm ${alert ? "border-red-500/50 bg-red-900/20" : ""}`}>
        <div className="flex items-center gap-2 text-[10px] font-bold uppercase tracking-wider text-slate-500 mb-1">
            {Icon && <Icon size={12} />} {label}
        </div>
        <div className={`text-base font-mono font-black tracking-tight ${color}`}>
            {value} <span className="text-[10px] text-slate-600 font-bold ml-0.5">{unit}</span>
        </div>
    </div>
);

export default function FlightConsole({ isOpen, data, history, isDark }: Props) {
    
    // --- GYRO CHART CONFIGURATION ---
    // Extracting Gyro Data (Indices 8, 9, 10 based on useFlightData)
    const gyroData = useMemo(() => {
        // Ensure history exists and has enough columns (we need up to index 10)
        if (!history || history.length < 11 || history[0].length === 0) return null;
        
        // Return [Time, GX, GY, GZ]
        return [
            history[0], // Time
            history[8], // GX
            history[9], // GY
            history[10] // GZ
        ] as uPlot.AlignedData;
    }, [history]);

    const gyroOptions = useMemo<uPlot.Options>(() => ({
        title: "GYROSCOPE (ANGULAR RATES)",
        width: 400, // Will autosize via UPlotChart
        height: 180,
        series: [
            {}, // Time
            { stroke: "#f87171", label: "GX", width: 1.5, value: (u, v) => v?.toFixed(1) },
            { stroke: "#4ade80", label: "GY", width: 1.5, value: (u, v) => v?.toFixed(1) },
            { stroke: "#60a5fa", label: "GZ", width: 1.5, value: (u, v) => v?.toFixed(1) },
        ],
        axes: [
            { stroke: isDark ? "#94a3b8" : "#64748b", grid: { stroke: isDark ? "#334155" : "#e2e8f0" }, size: 20 },
            { stroke: isDark ? "#94a3b8" : "#64748b", grid: { stroke: isDark ? "#334155" : "#e2e8f0" }, size: 40 }
        ],
        scales: { x: { time: false, auto: true }, y: { auto: true } },
        legend: { show: true, stroke: isDark ? "#cbd5e1" : "#1e293b" },
        padding: [10, 10, 0, 0]
    }), [isDark]);

    // Derived Status Strings
    const drogueStatus = data?.drogue_continuity ? "READY" : "OPEN";
    const mainStatus = data?.main_continuity ? "READY" : "OPEN";
    const gpsFix = ["NO FIX", "2D FIX", "3D FIX"][data?.fix || 0] || "WAIT";

    return (
        <div className={`fixed bottom-0 left-0 right-0 bg-slate-950/95 border-t border-slate-800 shadow-[0_-10px_40px_rgba(0,0,0,0.5)] transition-transform duration-300 ease-out z-40 ${isOpen ? 'translate-y-0' : 'translate-y-full'}`} style={{ height: '240px' }}>
            <div className="h-full container mx-auto p-3 flex gap-4">
                
                {/* LEFT: GYRO PLOT */}
                <div className="flex-[2] bg-slate-900/50 rounded-xl border border-slate-800 p-2 flex flex-col relative min-w-[300px]">
                    <div className="absolute top-2 left-3 z-10 pointer-events-none flex items-center gap-2">
                         <Disc size={12} className="text-slate-500" />
                         <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest">Rotational Dynamics</span>
                    </div>
                    <div className="flex-1 w-full relative mt-2">
                        {gyroData ? (
                            <UPlotChart data={gyroData} options={gyroOptions} />
                        ) : (
                            <div className="h-full flex items-center justify-center text-xs text-slate-600 font-mono animate-pulse">Waiting for Gyro Data...</div>
                        )}
                    </div>
                </div>

                {/* RIGHT: DIGITAL STATUS GRID */}
                <div className="flex-[3] grid grid-cols-4 gap-2 overflow-y-auto">
                    
                    {/* COL 1: AERODYNAMICS */}
                    <div className="flex flex-col gap-2">
                        <StatusBadge icon={Wind} label="MACH" value={data?.mach_number?.toFixed(2) || "0.00"} color="text-amber-400" />
                        <StatusBadge icon={Activity} label="AIRSPEED" value={data?.airspeed_tas?.toFixed(0) || "0"} unit="m/s" color="text-amber-400" />
                        <StatusBadge icon={Crosshair} label="TILT ANGLE" value={data?.tilt_angle?.toFixed(1) || "0.0"} unit="°" color="text-blue-400" />
                    </div>

                    {/* COL 2: GPS POSITION */}
                    <div className="flex flex-col gap-2">
                        <StatusBadge icon={MapPin} label="LATITUDE" value={data?.lat?.toFixed(5) || "0.00000"} color="text-emerald-400" />
                        <StatusBadge icon={MapPin} label="LONGITUDE" value={data?.lon?.toFixed(5) || "0.00000"} color="text-emerald-400" />
                        <StatusBadge icon={Activity} label="GPS ALT" value={data?.gps_alt?.toFixed(0) || "0"} unit="m" color="text-emerald-400" />
                    </div>

                    {/* COL 3: GPS STATUS & ENV */}
                    <div className="flex flex-col gap-2">
                        <StatusBadge icon={Radio} label="SATELLITES" value={data?.sats || "0"} unit={`(${gpsFix})`} color="text-emerald-400" />
                        <StatusBadge icon={Gauge} label="PRESSURE" value={data?.pressure?.toFixed(0) || "0"} unit="Pa" color="text-indigo-400" />
                        <StatusBadge icon={Activity} label="DIFF PRESS" value={data?.diff_pressure?.toFixed(0) || "0"} unit="Pa" color="text-slate-400" />
                    </div>

                    {/* COL 4: SYSTEM HEALTH */}
                    <div className="flex flex-col gap-2">
                        <StatusBadge 
                            icon={Zap} label="DROGUE" value={drogueStatus} 
                            color={drogueStatus === "READY" ? "text-green-500" : "text-red-500"} 
                            alert={drogueStatus === "OPEN"}
                        />
                        <StatusBadge 
                            icon={Zap} label="MAIN" value={mainStatus} 
                            color={mainStatus === "READY" ? "text-green-500" : "text-red-500"} 
                            alert={mainStatus === "OPEN"}
                        />
                         <StatusBadge icon={Radio} label="PACKET ID" value={data?.state || "0"} color="text-slate-500" />
                    </div>

                </div>

            </div>
        </div>
    );
}