"use client";
import { Gauge, Zap, ArrowUp, ArrowDown, CheckCircle, Wind, Lock } from 'lucide-react';

interface Props {
    state: number; 
}

export default function RocketStatus({ state }: Props) {
    // Exact mapping to Arjuna FC/include/flight_data.h
    const getStatusStyles = () => {
        switch (state) {
            case 0: return { label: "STANDBY",    color: "text-slate-400",   glow: "shadow-slate-500/20", icon: <CheckCircle size={14}/> };
            case 1: return { label: "BOOST",      color: "text-orange-500",  glow: "shadow-orange-500/50", icon: <ArrowUp size={14}/> };
            case 2: return { label: "COAST (HI)", color: "text-blue-400",    glow: "shadow-blue-500/40", icon: <Zap size={14}/> }; // Mach > 0.7
            case 3: return { label: "AIRBRAKES",  color: "text-cyan-400",    glow: "shadow-cyan-400/50", icon: <Wind size={14}/> }; // Mach 0.3-0.7 (Active Control)
            case 4: return { label: "COAST (LO)", color: "text-indigo-400",  glow: "shadow-indigo-500/40", icon: <Zap size={14}/> }; // Mach < 0.3
            case 5: return { label: "DROGUE",     color: "text-purple-400",  glow: "shadow-purple-500/40", icon: <ArrowDown size={14}/> };
            case 6: return { label: "MAIN",       color: "text-emerald-400", glow: "shadow-emerald-500/40", icon: <ArrowDown size={14}/> };
            case 7: return { label: "RECOVERY",   color: "text-red-500",     glow: "shadow-red-500/20", icon: <Lock size={14}/> }; // Landed
            default: return { label: "UNKNOWN",   color: "text-slate-600",   glow: "none", icon: <Zap size={14}/> };
        }
    };

    const s = getStatusStyles();

    return (
        <div className={`h-full bg-slate-900/50 rounded-2xl border border-slate-800 p-2 flex items-center justify-between relative overflow-hidden group transition-all duration-500 ${state === 1 ? 'border-orange-500/30' : ''}`}>
            {/* Background Pulse Effect */}
            <div className={`absolute -right-4 -top-4 w-24 h-24 bg-gradient-to-br from-transparent to-white/5 rounded-full blur-2xl animate-pulse ${s.color.replace('text', 'bg')}/10`} />

            {/* Left: Text Info */}
            <div className="flex flex-col justify-center z-10 pl-2">
                <span className="text-[9px] font-bold text-slate-500 uppercase tracking-widest mb-1 flex items-center gap-1">
                    {s.icon} STATUS
                </span>
                <span className={`text-xl font-black italic tracking-tighter ${s.color} drop-shadow-md`}>
                    {s.label}
                </span>
            </div>

            {/* Right: Visual Indicator */}
            <div className="h-full w-12 flex flex-col items-center justify-center relative z-10 mr-2">
                <div className={`w-3 h-8 my-[1px] rounded-sm transition-all duration-300
                    ${state === 1 ? 'bg-orange-500 drop-shadow-[0_0_15px_rgba(249,115,22,1)] animate-pulse' : 
                      state === 3 ? 'bg-cyan-500 drop-shadow-[0_0_10px_rgba(34,211,238,0.8)]' : 'bg-slate-700'}`}
                />
            </div>
        </div>
    );
}