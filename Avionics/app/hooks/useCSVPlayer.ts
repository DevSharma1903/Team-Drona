import { useState, useRef, useEffect, useCallback } from 'react';
import uPlot from 'uplot';
import { parseGenericCSV } from '../utils/parsers';

export type CSVColumnConfig = {
    colIdx: number;
    type: string;
    multiplier: number; 
};

export type PlotMode = 'REALTIME' | 'COMPLETE';

export const useCSVPlayer = () => {
    const [plotMode, setPlotMode] = useState<PlotMode>('REALTIME');
    const [isPlaying, setIsPlaying] = useState(false);
    const [currentTime, setCurrentTime] = useState(0);
    const [duration, setDuration] = useState(0);
    const [playbackSpeed, setPlaybackSpeed] = useState(1);
    const [plotData, setPlotData] = useState<uPlot.AlignedData>([]);
    const [fullData, setFullData] = useState<uPlot.AlignedData | null>(null);
    
    const masterDataRef = useRef<number[][]>([]);
    const rafRef = useRef<number>();
    const lastUpdateRef = useRef<number>(0);

    const updatePlotData = useCallback((time: number, mode: PlotMode = plotMode) => {
        if (masterDataRef.current.length === 0) return;

        if (mode === 'COMPLETE') {
            setPlotData(masterDataRef.current as uPlot.AlignedData);
            return;
        }

        const timeCol = masterDataRef.current[0];
        let endIdx = 0;
        for (let i = 0; i < timeCol.length; i++) {
            if (timeCol[i] > time) break;
            endIdx = i;
        }

        const sliced = masterDataRef.current.map(col => col.slice(0, endIdx + 1));
        setPlotData(sliced as uPlot.AlignedData);
    }, [plotMode]); 

    const loadCSV = useCallback((csvText: string, configs: CSVColumnConfig[]) => {
        const { columns } = parseGenericCSV(csvText);
        if (columns.length === 0) return;

        const activeConfigs = configs.filter(c => c.type !== 'IGNORE');
        const timeConfig = activeConfigs.find(c => c.type === 'TIMESTAMP');
        if (!timeConfig) return alert("No Timestamp column configured!");

        const newMasterData: number[][] = [];
        const rawTime = columns[timeConfig.colIdx];
        const scaledTime = rawTime.map(t => t * timeConfig.multiplier);
        newMasterData.push(scaledTime);

        const dataConfigs = activeConfigs.filter(c => c.type !== 'TIMESTAMP');
        dataConfigs.forEach(c => {
            newMasterData.push(columns[c.colIdx].map(v => v * c.multiplier));
        });

        masterDataRef.current = newMasterData;
        setFullData(newMasterData as uPlot.AlignedData);

        const startTime = scaledTime[0];
        const endTime = scaledTime[scaledTime.length - 1];
        setDuration(endTime);
        setCurrentTime(startTime);
        
        // FIXED: Added updatePlotData to deps
        updatePlotData(startTime, 'REALTIME'); 
        setIsPlaying(false);
    }, [updatePlotData]); 

    useEffect(() => {
        if (isPlaying && plotMode === 'REALTIME') {
            let lastRealTime = performance.now();
            const loop = (now: number) => {
                const delta = (now - lastRealTime) / 1000;
                lastRealTime = now;
                setCurrentTime(prev => {
                    const next = prev + (delta * playbackSpeed);
                    if (now - lastUpdateRef.current > 50) {
                        updatePlotData(next, 'REALTIME');
                        lastUpdateRef.current = now;
                    }
                    if (next >= duration) {
                        setIsPlaying(false);
                        return duration;
                    }
                    return next;
                });
                rafRef.current = requestAnimationFrame(loop);
            };
            rafRef.current = requestAnimationFrame(loop);
        } else {
            if (rafRef.current) cancelAnimationFrame(rafRef.current);
        }
        return () => { if (rafRef.current) cancelAnimationFrame(rafRef.current); };
    }, [isPlaying, duration, playbackSpeed, plotMode, updatePlotData]); // FIXED: Added updatePlotData to deps

    const toggleMode = (mode: PlotMode) => {
        setPlotMode(mode);
        if (mode === 'COMPLETE') updatePlotData(duration, 'COMPLETE');
        else updatePlotData(currentTime, 'REALTIME');
    };

    const seek = (val: number) => { 
        setCurrentTime(val); 
        if (plotMode === 'REALTIME') updatePlotData(val, 'REALTIME');
    };

    const reset = () => { 
        setIsPlaying(false);
        if (masterDataRef.current.length > 0) {
            const start = masterDataRef.current[0][0];
            setCurrentTime(start);
            if (plotMode === 'REALTIME') updatePlotData(start, 'REALTIME');
        }
    };

    return { 
        loadCSV, plotData, fullData, isPlaying, togglePlay: () => setIsPlaying(p => !p), 
        currentTime, duration, seek, reset, setPlaybackSpeed, plotMode, toggleMode
    };
};