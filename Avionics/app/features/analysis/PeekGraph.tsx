"use client";
import React, { useMemo } from 'react';
import UPlotChart from '../../components/UPlotChart';
import uPlot from 'uplot';

interface Props {
    data: uPlot.AlignedData;
    activeMetric: string | null;
    isDark: boolean;
}

export default function PeekGraph({ data, activeMetric, isDark }: Props) {
    const axisColor = isDark ? "#94a3b8" : "#475569";
    const gridColor = isDark ? "rgba(255,255,255,0.05)" : "rgba(0,0,0,0.05)";

    const config: Record<string, any> = {
        'alt':  { idx: 1,  color: "#3b82f6", unit: "m" },
        'vel':  { idx: 2,  color: "#10b981", unit: "m/s" },
        'acc':  { idx: 3,  color: "#ef4444", unit: "G" },
        'pres': { idx: 14, color: "#f59e0b", unit: "Pa" },
        'temp': { idx: 16, color: "#a855f7", unit: "°C" },
    };

    const activeConfig = activeMetric ? config[activeMetric] : null;

    // Safety: Ensure data exists before trying to slice it
    const hasData = data && data[0] && data[0].length > 0;
    
    // Prepare data slice
    const chartData = (activeConfig && hasData)
        ? [ data[0], data[activeConfig.idx] || [] ] as uPlot.AlignedData
        : null;

    // Get latest value safely
    const liveValue = chartData && chartData[1].length > 0
        ? chartData[1][chartData[1].length - 1]?.toFixed(2) 
        : "0.00";

    const options = useMemo<uPlot.Options>(() => {
        if (!activeConfig) return {} as any;

        return {
            // REDUCED DIMENSIONS: 300x200
            width: 300,
            height: 200,
            series: [
                {}, // Time (x-axis)
                {
                    stroke: activeConfig.color,
                    width: 3, 
                    points: { show: false },
                    // FIXED GRADIENT LOGIC
                    fill: (u: uPlot, seriesIdx: number) => {
                        // 'u' is the chart instance. We get ctx from u.ctx
                        const ctx = u.ctx;
                        // Use the chart's drawing area height for the gradient
                        const gradient = ctx.createLinearGradient(0, u.bbox.top, 0, u.bbox.top + u.bbox.height);
                        gradient.addColorStop(0, `${activeConfig.color}40`); // 25% opacity
                        gradient.addColorStop(1, `${activeConfig.color}05`); // 5% opacity
                        return gradient;
                    },
                }
            ],
            axes: [
                { stroke: axisColor, grid: { stroke: gridColor }, font: "9px monospace", size: 20 },
                { stroke: axisColor, grid: { stroke: gridColor }, font: "9px monospace", size: 40 } // Reduced Y-axis margin slightly
            ],
            scales: { x: { time: false, auto: true }, y: { auto: true } },
            cursor: { show: true },
            legend: { show: false },
            padding: [10, 5, 5, 0]
        };
    }, [activeConfig, axisColor, gridColor]);

    return (
        <div 
            className={`fixed top-24 right-4 z-50 transition-all duration-300 ease-out transform ${
                activeMetric ? 'translate-x-0 opacity-100' : 'translate-x-[120%] opacity-0'
            }`}
        >
            {activeConfig && chartData && (
                // CONTAINER: Fixed 3:2 Aspect Ratio (300px x 200px)
                <div className="w-[300px] h-[200px] bg-slate-950/95 border border-slate-800 rounded-md shadow-2xl overflow-hidden relative group">
                    
                    {/* Floating Live Value Badge (Top Right) */}
                    <div className="absolute top-2 right-2 z-10 pointer-events-none">
                         <div className="bg-slate-900/80 backdrop-blur-md px-1.5 py-0.5 rounded border border-white/10 shadow-sm flex items-baseline gap-1">
                            <span className="text-sm font-mono font-black" style={{ color: activeConfig.color }}>
                                {liveValue}
                            </span>
                            <span className="text-[9px] font-bold text-slate-500">
                                {activeConfig.unit}
                            </span>
                        </div>
                    </div>

                    {/* Graph Body */}
                    <div className="w-full h-full flex justify-center items-center p-1">
                        <UPlotChart data={chartData} options={options} />
                    </div>
                </div>
            )}
        </div>
    );
}