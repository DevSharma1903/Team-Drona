"use client";
import { useState, useMemo } from 'react'; // Added useMemo
import { Activity, Usb, StopCircle, Download, ChevronUp, ChevronDown, Compass, Move, Disc, Radio } from 'lucide-react';
import Header from '../../components/Header';
import SlideInTelemetry from './SlideInTelemetry';
import PeekGraph from '../analysis/PeekGraph';
import RocketStatus from '../../components/RocketStatus';
import RocketModel from './RocketModel';
import GPSMap from './GPSMap';
import ClientOnly from '../../components/ClientOnly';
import GraphGrid, { GraphConfig } from '../../components/GraphGrid'; 
import { useFlightData } from '../../hooks/useFlightData';
import { useSerial } from '../../hooks/useSerial';
import { useFlightSimulation } from '../../hooks/useFlightSimulation';

interface Props {
    isDark: boolean;
    toggleTheme: () => void;
    activeTab: string;
    onTabChange: (tab: string) => void;
}

// --- FLIGHT MODE GRAPH CONFIGURATION ---
const FLIGHT_GRAPHS: GraphConfig[] = [
    { 
        id: "flight_dynamics", 
        title: "FLIGHT DYNAMICS", 
        type: 'chart',
        series: [
            { label: "ALT", idx: 1, color: "#3b82f6", unit: "m" },
            { label: "VEL", idx: 2, color: "#10b981", unit: "m/s" }
        ]
    },
    { 
        id: "forces", 
        title: "G-FORCE LOAD", 
        type: 'chart',
        series: [
            { label: "ACCEL", idx: 3, color: "#ef4444", unit: "G" }
        ]
    },
    { 
        id: "environment", 
        title: "ATMOSPHERE", 
        type: 'chart',
        series: [
            { label: "PRESS", idx: 14, color: "#f59e0b", unit: "Pa" }
        ]
    },
];

const SensorCard = ({ title, icon: Icon, data, unit, color }: any) => (
    <div className="bg-slate-900/50 border border-slate-800 rounded p-2 flex flex-col gap-1">
        <div className="flex items-center gap-2 text-[10px] uppercase font-bold text-slate-400">
            <Icon size={10} /> {title}
        </div>
        <div className="grid grid-cols-3 gap-1">
            {['x', 'y', 'z'].map((axis) => (
                <div key={axis} className="flex flex-col">
                    <span className="text-[9px] text-slate-500 uppercase">{axis.toUpperCase()}</span>
                    <span className={`text-xs font-mono font-bold ${color}`}>
                        {data ? data[axis] : '0'} <span className="text-[8px] text-slate-600">{unit}</span>
                    </span>
                </div>
            ))}
        </div>
    </div>
);

export default function FlightMode({ isDark, toggleTheme, activeTab, onTabChange }: Props) {
    const { plotData, trajectory, currentPacket, addData, resetData, downloadLog } = useFlightData();
    const { isSimulating, startSimulation, stopSimulation } = useFlightSimulation(addData);
    const { isConnected, connect, disconnect } = useSerial(addData);
    
    const [isDeckOpen, setIsDeckOpen] = useState(false);
    const [mapResetKey, setMapResetKey] = useState(0);
    const [hoveredMetric, setHoveredMetric] = useState<string | null>(null);

    // Inside FlightMode component
    const getRotation = () => {
        // FIX: Use calculated Tilt (Attitude) instead of Gyro Rates (Speed)
        // We map tilt to the X-axis to show the rocket 'leaning' relative to vertical.
        // Yaw (Y) and Roll (Z) are set to 0 because the FC doesn't transmit them in the standard packet.
        return {
            x: currentPacket?.tilt_angle || 0, 
            y: 0, 
            z: 0, 
        };
    };

    // --- NEW: Convert 3D Trajectory (Lat, Alt, Lon) to 2D Map Path (Lat, Lon) ---
    const mapTrajectory = useMemo(() => {
        return trajectory.map(p => [p[0], p[2]] as [number, number]);
    }, [trajectory]);

    const HeaderActions = (
        <div className="flex items-center gap-3">
            {!isConnected ? (
                <button onClick={async () => { resetData(); setMapResetKey(k => k+1); await connect(); }} className="flex items-center gap-2 px-3 py-1.5 bg-blue-600 hover:bg-blue-700 text-white font-bold rounded-md text-xs transition-all">
                    <Usb size={14} /> CONNECT
                </button>
            ) : (
                <button onClick={() => disconnect()} className="flex items-center gap-2 px-3 py-1.5 bg-red-500 hover:bg-red-600 text-white font-bold rounded-md text-xs transition-all">
                    <StopCircle size={14} /> STOP
                </button>
            )}

            <button 
                onClick={() => { resetData(); setMapResetKey(k => k+1); isSimulating ? stopSimulation() : startSimulation('NOMINAL'); }} 
                className={`flex items-center gap-2 px-3 py-1.5 rounded-md text-xs font-bold transition-all border ${isSimulating ? "bg-amber-600 border-amber-400 text-white" : "bg-slate-800 border-slate-700 text-slate-300"}`}
            >
                <Activity size={14} /> {isSimulating ? "END SIM" : "RUN SIM"}
            </button>

            <button onClick={downloadLog} className="p-1.5 bg-slate-900 hover:bg-slate-800 rounded text-slate-400 border border-slate-800 flex items-center gap-2 text-[10px] font-bold uppercase transition-colors">
                <Download size={12} /> CSV
            </button>

            <div className="h-6 w-[1px] bg-slate-800 mx-1" />

            <button 
                onClick={() => setIsDeckOpen(!isDeckOpen)}
                className={`flex items-center gap-2 px-3 py-1.5 rounded-md text-xs font-bold transition-all border ${isDeckOpen ? 'bg-indigo-600 border-indigo-400 text-white' : 'bg-slate-900 border-slate-700 text-slate-400'}`}
            >
                {isDeckOpen ? <ChevronDown size={14}/> : <ChevronUp size={14}/>}
                {isDeckOpen ? "HIDE CONSOLE" : "SHOW CONSOLE"}
            </button>
        </div>
    );

    return (
        <div className="h-screen flex flex-col bg-slate-950 text-white overflow-hidden relative">
            <Header actions={HeaderActions} isDark={isDark} activeTab={activeTab} onTabChange={onTabChange} />

            <SlideInTelemetry data={currentPacket} onHoverMetric={setHoveredMetric} />
            <PeekGraph data={plotData as any} activeMetric={hoveredMetric} isDark={isDark} />

            <div className="flex-1 flex flex-col relative overflow-hidden">
                <div className={`w-full flex flex-col transition-all duration-500 ease-in-out overflow-hidden ${isDeckOpen ? 'h-[calc(100vh-220px)] pb-2' : 'h-full'}`}>
                    
                    <div className="flex-1 p-0 grid grid-cols-1 lg:grid-cols-12 gap-0 min-h-0 relative">
                        <div className="lg:col-span-8 bg-black relative overflow-hidden flex flex-col border-r border-slate-800/50 group">
                            <div className="absolute top-4 right-4 z-20 w-48 opacity-80 group-hover:opacity-100 transition-opacity">
                                <RocketStatus state={currentPacket?.state || 0} />
                            </div>
                            <div className="flex-1">
                                <ClientOnly>
                                    <RocketModel 
                                        altitude={currentPacket?.altitude || 0} 
                                        trajectory={trajectory}
                                        rotation={getRotation()} 
                                        state={currentPacket?.state || 0} 
                                    />
                                </ClientOnly>
                            </div>
                        </div>

                        <div className="lg:col-span-4 bg-slate-900 relative z-0 flex flex-col">
                            <div className="flex-1 w-full h-full relative z-0">
                                <ClientOnly>
                                    {/* UPDATED: Passing the mapTrajectory prop now! */}
                                    <GPSMap 
                                        key={mapResetKey} 
                                        lat={currentPacket?.lat || 0} 
                                        lon={currentPacket?.lon || 0}
                                        trajectory={mapTrajectory} 
                                    />
                                </ClientOnly>
                            </div>
                        </div>
                    </div>
                </div>

                {/* BOTTOM CONSOLE DECK */}
                <div className={`fixed bottom-0 left-0 right-0 h-[220px] bg-slate-950/90 backdrop-blur-xl border-t border-white/10 shadow-[0_-10px_50px_rgba(0,0,0,0.5)] transition-transform duration-500 ease-in-out z-40 ${isDeckOpen ? 'translate-y-0' : 'translate-y-full'}`}>
                    <div className="h-full flex p-3 gap-3">
                        
                        {/* LEFT: GRAPHS */}
                        <div className="flex-[2] overflow-hidden rounded border border-slate-800/50">
                            <GraphGrid data={plotData as any} isDark={isDark} customConfigs={FLIGHT_GRAPHS} />
                        </div>

                        {/* RIGHT: RAW SENSORS */}
                        <div className="flex-1 flex flex-col gap-2 overflow-y-auto pr-1">
                            
                            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-2 gap-2">
                                <SensorCard 
                                    title="Gyroscope" 
                                    icon={Disc} 
                                    data={{ x: currentPacket?.gx, y: currentPacket?.gy, z: currentPacket?.gz }} 
                                    unit="°/s" 
                                    color="text-cyan-400" 
                                />
                                <SensorCard 
                                    title="Accelerometer" 
                                    icon={Move} 
                                    data={{ x: currentPacket?.ax, y: currentPacket?.ay, z: currentPacket?.az }} 
                                    unit="m/s²" 
                                    color="text-emerald-400" 
                                />
                            </div>
                            
                            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-2 gap-2">
                                <SensorCard 
                                    title="Magnetometer" 
                                    icon={Compass} 
                                    data={{ x: currentPacket?.mx, y: currentPacket?.my, z: currentPacket?.mz }} 
                                    unit="µT" 
                                    color="text-amber-400" 
                                />
                                
                                <div className="bg-slate-900/50 border border-slate-800 rounded p-2 flex flex-col gap-1">
                                    <div className="flex items-center gap-2 text-[10px] uppercase font-bold text-slate-400">
                                        <Radio size={10} /> GPS & ENV
                                    </div>
                                    <div className="grid grid-cols-2 gap-y-2 gap-x-1">
                                        <div className="flex justify-between items-center border-b border-white/5 pb-0.5">
                                            <span className="text-[9px] text-slate-500">SATS</span>
                                            <span className="text-xs font-mono font-bold text-white">{currentPacket?.sats || 0}</span>
                                        </div>
                                        <div className="flex justify-between items-center border-b border-white/5 pb-0.5">
                                            <span className="text-[9px] text-slate-500">FIX TYPE</span>
                                            <span className="text-xs font-mono font-bold text-white">{currentPacket?.fix || 0}</span>
                                        </div>
                                        <div className="flex justify-between items-center">
                                            <span className="text-[9px] text-slate-500">TEMP</span>
                                            <span className="text-xs font-mono font-bold text-orange-400">{currentPacket?.temp?.toFixed(1) || 0}°C</span>
                                        </div>
                                        <div className="flex justify-between items-center">
                                            <span className="text-[9px] text-slate-500">PRESS</span>
                                            <span className="text-xs font-mono font-bold text-indigo-400">{currentPacket?.pressure?.toFixed(0) || 0}Pa</span>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
}