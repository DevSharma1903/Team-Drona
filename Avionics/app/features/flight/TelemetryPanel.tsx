"use client";
import { FlightDataPoint } from '../../hooks/useFlightData';
import { Activity, Wind, Gauge, Navigation } from 'lucide-react';
import RocketStatus from '../../components/RocketStatus';

interface Props {
    data: FlightDataPoint | null;
}

export default function TelemetryPanel({ data }: Props) {
    if (!data) return (
        <div className="h-full flex items-center justify-center bg-slate-900/30 rounded-xl border border-dashed border-slate-800 text-slate-500 text-[10px] uppercase tracking-[0.3em]">
            Waiting for uplink...
        </div>
    );

    // Helper to determine state color
    const getStateColor = (state: number) => {
        const colors = ["text-slate-400", "text-amber-500", "text-emerald-500", "text-blue-500", "text-indigo-500", "text-rose-500"];
        return colors[state] || "text-white";
    };

    return (
        <div className="h-full grid grid-cols-4 gap-4">
            
            {/* 1. ALTITUDE (Primary) */}
            <div className="bg-slate-900/50 rounded-2xl border border-slate-800 p-3 flex flex-col justify-center relative overflow-hidden group">
                <div className="absolute top-0 left-0 w-1 h-full bg-blue-500" />
                <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest mb-1 flex items-center gap-2">
                    <Navigation size={12} className="text-blue-500" /> Altitude (AGL)
                </span>
                <div className="flex items-baseline gap-1">
                    <span className="text-3xl font-black text-white tabular-nums tracking-tighter">
                        {data.altitude.toFixed(1)}
                    </span>
                    <span className="text-xs font-bold text-slate-400">m</span>
                </div>
            </div>

            {/* 2. ACCELERATION */}
            <div className="bg-slate-900/50 rounded-2xl border border-slate-800 p-3 flex flex-col justify-center relative overflow-hidden">
                <div className="absolute top-0 left-0 w-1 h-full bg-rose-500" />
                <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest mb-1 flex items-center gap-2">
                    <Activity size={12} className="text-rose-500" /> Vertical G-Force
                </span>
                <div className="flex items-baseline gap-1">
                    <span className="text-3xl font-black text-white tabular-nums tracking-tighter">
                        {(data.accel_z / 9.81).toFixed(2)}
                    </span>
                    <span className="text-xs font-bold text-slate-400">G</span>
                </div>
            </div>

            {/* 3. PRESSURE */}
            <div className="bg-slate-900/50 rounded-2xl border border-slate-800 p-3 flex flex-col justify-center relative overflow-hidden">
                <div className="absolute top-0 left-0 w-1 h-full bg-amber-500" />
                <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest mb-1 flex items-center gap-2">
                    <Wind size={12} className="text-amber-500" /> Baro Pressure
                </span>
                <div className="flex items-baseline gap-1">
                    <span className="text-2xl font-black text-white tabular-nums tracking-tighter">
                        {(data.pressure || 0).toFixed(0)}
                    </span>
                    <span className="text-[10px] font-bold text-slate-400">Pa</span>
                </div>
            </div>

            {/* 4. MISSION STATE */}
            <RocketStatus state={data.state} />
            {/* <div className="bg-slate-900/50 rounded-2xl border border-slate-800 p-3 flex flex-col justify-center relative overflow-hidden">
                <div className="absolute top-0 left-0 w-1 h-full bg-emerald-500" />
                <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest mb-1 flex items-center gap-2">
                    <Gauge size={12} className="text-emerald-500" /> Mission State
                </span>
                <div className={`text-xl font-black uppercase italic tracking-tighter ${getStateColor(data.state)}`}>
                    {["STANDBY", "POWERED", "COASTING", "DROGUE", "MAIN", "LANDED"][data.state] || "UNKNOWN"}
                </div>
            </div> */}

        </div>
    );
}