"use client";
import React, { useEffect, useRef } from 'react';
import uPlot from 'uplot';
import "uplot/dist/uPlot.min.css";

interface UPlotChartProps {
    data: uPlot.AlignedData;
    options: uPlot.Options;
}

export default function UPlotChart({ data, options }: UPlotChartProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const uPlotInst = useRef<uPlot | null>(null);

    // 1. Initialize Chart
    useEffect(() => {
        if (!containerRef.current) return;

        // Create chart
        // We pass the container, but we will handle sizing via Observer
        const instance = new uPlot(options, data, containerRef.current);
        uPlotInst.current = instance;

        // 2. Setup Resize Observer
        const observer = new ResizeObserver((entries) => {
            if (!uPlotInst.current) return;
            for (let entry of entries) {
                const { width, height } = entry.contentRect;
                // Only resize if dimensions are valid to prevent 0x0 crash
                if (width > 0 && height > 0) {
                    uPlotInst.current.setSize({ width, height });
                }
            }
        });

        observer.observe(containerRef.current);

        return () => {
            observer.disconnect();
            instance.destroy();
            uPlotInst.current = null;
        };
    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [options]); 

    // 3. Fast Data Updates
    useEffect(() => {
        if (uPlotInst.current) {
            uPlotInst.current.setData(data);
        }
    }, [data]);

    return (
        // ABSOLUTE POSITIONING FIX:
        // 'absolute inset-0' takes this div out of the layout flow.
        // It will perfectly fill the parent 'relative' container without pushing it.
        <div ref={containerRef} className="absolute inset-0 w-full h-full" />
    );
}