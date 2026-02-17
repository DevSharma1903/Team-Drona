"use client";
import { useState } from 'react';
import { Play, Pause, RotateCcw, Settings, FileSpreadsheet, Activity, BarChart3 } from 'lucide-react';
import { parseGenericCSV } from '../../utils/parsers';
import { useCSVPlayer, CSVColumnConfig } from '../../hooks/useCSVPlayer';
import GraphGrid, { GraphConfig } from '../../components/GraphGrid';
import Header from '../../components/Header'; 
import { PARAM_DEFINITIONS, ParameterType } from '../../utils/parameters';

interface Props {
    isDark: boolean;
    toggleTheme: () => void;
    activeTab: string;
    onTabChange: (tab: string) => void;
}

export default function CSVMode({ isDark, toggleTheme, activeTab, onTabChange }: Props) {
    const [step, setStep] = useState<'UPLOAD' | 'CONFIG' | 'PLOT'>('UPLOAD');
    const [rawFile, setRawFile] = useState<string>("");
    const [headers, setHeaders] = useState<string[]>([]);
    
    const [configs, setConfigs] = useState<{ colIdx: number, type: ParameterType, unit: string }[]>([]);
    const [generatedGraphConfigs, setGeneratedGraphConfigs] = useState<GraphConfig[]>([]);

    const { loadCSV, plotData, isPlaying, togglePlay, currentTime, duration, seek, reset, plotMode, toggleMode } = useCSVPlayer();
    const [mapResetKey, setMapResetKey] = useState(0);

    const handleFileUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (!file) return;
        
        const reader = new FileReader();
        reader.onload = (evt) => {
            const text = evt.target?.result as string;
            setRawFile(text);
            
            const { headers } = parseGenericCSV(text);
            setHeaders(headers);
            
            const initialConfigs = headers.map((h, i) => {
                const lower = h.toLowerCase();
                let type: ParameterType = 'IGNORE';
                
                if (lower.includes('time')) type = 'TIMESTAMP';
                else if (lower.includes('alt')) type = 'ALTITUDE';
                else if (lower.includes('vel')) type = 'VEL_NET';
                else if (lower.includes('acc')) type = 'ACCEL_NET';
                else if (lower.includes('pres')) type = 'PRESSURE';
                else if (lower.includes('lat')) type = 'GPS_LAT'; 
                else if (lower.includes('lon') || lower.includes('lng')) type = 'GPS_LON';
                
                const defUnit = Object.keys(PARAM_DEFINITIONS[type]?.units || {})[0] || "";
                return { colIdx: i, type, unit: defUnit };
            });
            
            setConfigs(initialConfigs);
            setStep('CONFIG');
        };
        reader.readAsText(file);
    };

    const handleVisualize = () => {
        const hookConfigs: CSVColumnConfig[] = configs.map(c => ({
            colIdx: c.colIdx, 
            type: c.type, 
            multiplier: PARAM_DEFINITIONS[c.type]?.units[c.unit] || 1 
        }));

        const activeCols = hookConfigs.filter(c => c.type !== 'IGNORE' && c.type !== 'TIMESTAMP');
        
        const latIndex = activeCols.findIndex(c => c.type === 'GPS_LAT');
        const lonIndex = activeCols.findIndex(c => c.type === 'GPS_LON');
        const hasMap = latIndex !== -1 && lonIndex !== -1;

        const graphConfigs: GraphConfig[] = [];

        if (hasMap) {
            graphConfigs.push({
                id: 'gps_map',
                title: 'GPS TRACK',
                type: 'map',
                series: [],
                dataIdx: [latIndex + 1, lonIndex + 1] 
            });
        }

        activeCols.forEach((c, i) => {
            if (hasMap && (c.type === 'GPS_LAT' || c.type === 'GPS_LON')) return;

            const def = PARAM_DEFINITIONS[c.type as ParameterType];
            
            graphConfigs.push({ 
                id: `col_${c.colIdx}`, 
                title: def?.label || c.type, 
                type: 'chart', 
                series: [{ 
                    label: def?.label || c.type, 
                    stroke: '#3b82f6', 
                    unit: c.type 
                }], 
                dataIdx: [i + 1] 
            });
        });

        if (graphConfigs.length === 0) return alert("Select at least one valid data column.");

        setGeneratedGraphConfigs(graphConfigs);
        loadCSV(rawFile, hookConfigs);
        setMapResetKey(p => p + 1);
        setStep('PLOT');
    };

    const CSVActions = step === 'PLOT' ? (
        <div className="flex items-center gap-2">
            <button onClick={() => setStep('CONFIG')} className="flex items-center gap-2 px-3 py-1.5 text-xs font-bold text-slate-500 hover:text-blue-500 bg-slate-100 dark:bg-slate-800 rounded-md">
                <Settings size={14} /> CONFIG
            </button>
            <div className="w-[1px] h-6 bg-slate-200 dark:bg-slate-700 mx-1"></div>
            {plotMode === 'REALTIME' && (
                <>
                     <span className="text-xs font-mono text-slate-500 w-16 text-right">{currentTime.toFixed(1)}s</span>
                     <button onClick={togglePlay} className="p-1.5 bg-blue-100 dark:bg-blue-900/30 text-blue-600 rounded-md hover:bg-blue-200 dark:hover:bg-blue-900/50">
                        {isPlaying ? <Pause size={14} /> : <Play size={14} />}
                    </button>
                    <button onClick={reset} className="p-1.5 hover:bg-slate-100 dark:hover:bg-slate-800 rounded-md">
                        <RotateCcw size={14} />
                    </button>
                </>
            )}
            <div className="flex bg-slate-100 dark:bg-slate-800 rounded-md p-1 ml-2">
                <button onClick={() => toggleMode('REALTIME')} className={`px-2 py-1 text-[10px] font-bold rounded flex items-center gap-1 ${plotMode === 'REALTIME' ? 'bg-white dark:bg-slate-700 text-blue-600 shadow-sm' : 'text-slate-500'}`}>
                    <Activity size={12} /> LIVE
                </button>
                <button onClick={() => toggleMode('COMPLETE')} className={`px-2 py-1 text-[10px] font-bold rounded flex items-center gap-1 ${plotMode === 'COMPLETE' ? 'bg-white dark:bg-slate-700 text-blue-600 shadow-sm' : 'text-slate-500'}`}>
                    <BarChart3 size={12} /> ALL
                </button>
            </div>
        </div>
    ) : null;

    return (
        <div className="h-full flex flex-col bg-slate-50 dark:bg-black">
            <Header actions={CSVActions} isDark={isDark} toggleTheme={toggleTheme} activeTab={activeTab} onTabChange={onTabChange} />
            
            <div className="flex-1 min-h-0 p-4 overflow-hidden flex flex-col">
                {step === 'UPLOAD' && (
                    <div className="flex-1 flex flex-col items-center justify-center border-2 border-dashed border-slate-300 dark:border-slate-700 rounded-xl bg-slate-50 dark:bg-slate-900/50">
                        <div className="text-center p-10">
                            <FileSpreadsheet size={64} className="mx-auto text-blue-500 mb-4" />
                            <h2 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">Upload Mission Log</h2>
                            <input type="file" id="csv-upload" accept=".csv" onChange={handleFileUpload} className="hidden" />
                            <label htmlFor="csv-upload" className="px-6 py-3 bg-blue-600 hover:bg-blue-700 text-white font-bold rounded-lg cursor-pointer shadow-lg transition-all">SELECT CSV FILE</label>
                        </div>
                    </div>
                )}

                {step === 'CONFIG' && (
                    <div className="flex-1 flex flex-col gap-4 overflow-hidden">
                        <div className="flex justify-between items-center bg-white dark:bg-slate-900 p-4 rounded-xl border border-slate-200 dark:border-slate-800 shadow-sm shrink-0">
                            <h2 className="text-xl font-bold text-slate-900 dark:text-white">Configure Columns</h2>
                            <button onClick={handleVisualize} className="px-6 py-2 bg-blue-600 text-white rounded-lg font-bold hover:bg-blue-700">VISUALIZE</button>
                        </div>
                        
                        {/* FIX IS HERE: Added 'pb-32' (Large bottom padding) to allow scrolling past the end */}
                        <div className="flex-1 overflow-auto bg-white dark:bg-slate-900 rounded-xl border border-slate-200 dark:border-slate-800 p-4 pb-32">
                            <table className="w-full text-left border-collapse">
                                <thead className="text-slate-500 border-b border-slate-200 dark:border-slate-700 sticky top-0 bg-white dark:bg-slate-900 z-10 shadow-sm">
                                    <tr><th className="p-3">Header</th><th className="p-3">Type</th><th className="p-3">Unit</th></tr>
                                </thead>
                                <tbody>
                                    {headers.map((h, i) => (
                                        <tr key={i} className="border-b border-slate-100 dark:border-slate-800">
                                            <td className="p-3 font-mono text-sm text-slate-700 dark:text-slate-300">{h}</td>
                                            <td className="p-3">
                                                <select className="bg-slate-100 dark:bg-slate-800 border-none rounded p-2 text-sm w-full" value={configs[i].type} onChange={(e) => { const newType = e.target.value as ParameterType; const newConfigs = [...configs]; newConfigs[i].type = newType; newConfigs[i].unit = Object.keys(PARAM_DEFINITIONS[newType]?.units || {})[0] || ""; setConfigs(newConfigs); }}>
                                                    {Object.keys(PARAM_DEFINITIONS).map(k => <option key={k} value={k}>{PARAM_DEFINITIONS[k as ParameterType].label}</option>)}
                                                </select>
                                            </td>
                                            <td className="p-3">
                                                {configs[i].type !== 'IGNORE' && PARAM_DEFINITIONS[configs[i].type]?.units && (
                                                    <select className="bg-slate-100 dark:bg-slate-800 border-none rounded p-2 text-sm w-full" value={configs[i].unit} onChange={(e) => { const newConfigs = [...configs]; newConfigs[i].unit = e.target.value; setConfigs(newConfigs); }}>
                                                        {Object.keys(PARAM_DEFINITIONS[configs[i].type].units).map(u => <option key={u} value={u}>{u}</option>)}
                                                    </select>
                                                )}
                                            </td>
                                        </tr>
                                    ))}
                                </tbody>
                            </table>
                        </div>
                    </div>
                )}

                {step === 'PLOT' && (
                    <div className="flex-1 bg-white dark:bg-slate-900 rounded-xl border border-slate-200 dark:border-slate-800 shadow-sm p-4 relative overflow-hidden">
                        <GraphGrid 
                            customConfigs={generatedGraphConfigs} 
                            data={plotData} 
                            isDark={isDark} 
                            mapResetKey={mapResetKey} 
                        />
                    </div>
                )}
            </div>
        </div>
    );
}