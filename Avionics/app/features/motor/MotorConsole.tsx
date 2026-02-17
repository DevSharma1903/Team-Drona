"use client";
import { useState, useEffect, useRef } from 'react';
import { X, Send, ShieldAlert, Rocket, Lock, Terminal } from 'lucide-react';

interface Props {
    isOpen: boolean;
    onClose: () => void;
    sendCommand: (cmd: string) => void;
    logs: string[];
}

export default function MotorConsole({ isOpen, onClose, sendCommand, logs }: Props) {
    const [input, setInput] = useState("");
    const [captcha, setCaptcha] = useState<{ challenge: string, answer: string, cmd: string } | null>(null);
    const [captchaInput, setCaptchaInput] = useState("");
    const scrollRef = useRef<HTMLDivElement>(null);

    // Auto-scroll logs
    useEffect(() => {
        if (scrollRef.current) {
            scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
    }, [logs, isOpen]);

    const handleSend = () => {
        if (!input.trim()) return;
        sendCommand(input);
        setInput("");
    };

    // --- SAFETY CAPTCHA LOGIC ---
    const initiateSafetyCommand = (cmd: string) => {
        // Generate a random 4-digit code
        const code = Math.floor(1000 + Math.random() * 9000).toString();
        setCaptcha({ challenge: code, answer: code, cmd: cmd });
        setCaptchaInput("");
    };

    const confirmCaptcha = () => {
        if (!captcha) return;
        if (captchaInput === captcha.answer) {
            sendCommand(captcha.cmd);
            setCaptcha(null); // Close modal
        } else {
            alert("Incorrect Code. Command Aborted.");
            setCaptcha(null);
        }
    };

    return (
        <>
            {/* OVERLAY BACKDROP */}
            {isOpen && <div className="fixed inset-0 bg-black/50 z-40" onClick={onClose} />}

            {/* SLIDE-IN PANEL */}
            <div className={`fixed top-0 right-0 h-full w-96 bg-slate-900 border-l border-slate-700 shadow-2xl z-50 transform transition-transform duration-300 ease-in-out flex flex-col ${isOpen ? 'translate-x-0' : 'translate-x-full'}`}>
                
                {/* HEADER */}
                <div className="p-4 border-b border-slate-700 flex justify-between items-center bg-slate-950">
                    <div className="flex items-center gap-2 text-white font-bold">
                        <Terminal size={18} className="text-blue-400" /> COMMAND CONSOLE
                    </div>
                    <button onClick={onClose} className="p-1 hover:bg-slate-800 rounded text-slate-400 hover:text-white">
                        <X size={20} />
                    </button>
                </div>

                {/* LOG OUTPUT AREA */}
                <div className="flex-1 p-4 overflow-y-auto font-mono text-xs bg-black text-green-400 space-y-1" ref={scrollRef}>
                    {logs.length === 0 && <span className="text-slate-600 italic">No commands sent yet...</span>}
                    {logs.map((log, i) => (
                        <div key={i} className="break-all border-b border-slate-900 pb-0.5">{log}</div>
                    ))}
                </div>

                {/* DANGER ZONE BUTTONS */}
                <div className="p-4 bg-slate-950 border-t border-slate-800 grid grid-cols-2 gap-3">
                    <button 
                        onClick={() => initiateSafetyCommand("ARM")}
                        className="flex items-center justify-center gap-2 bg-red-600/20 hover:bg-red-600/40 border border-red-600 text-red-500 font-bold py-2 rounded text-xs transition-colors"
                    >
                        <ShieldAlert size={14} /> ARM SYSTEM
                    </button>
                    
                    <button 
                        onClick={() => initiateSafetyCommand("LAUNCH")}
                        className="flex items-center justify-center gap-2 bg-orange-600/20 hover:bg-orange-600/40 border border-orange-600 text-orange-500 font-bold py-2 rounded text-xs transition-colors"
                    >
                        <Rocket size={14} /> IGNITE / LAUNCH
                    </button>

                    <button 
                        onClick={() => sendCommand("SAFE")}
                        className="col-span-2 flex items-center justify-center gap-2 bg-green-600/20 hover:bg-green-600/40 border border-green-600 text-green-500 font-bold py-2 rounded text-xs transition-colors"
                    >
                        <Lock size={14} /> SYSTEM SAFE (DISARM)
                    </button>
                </div>

                {/* RAW INPUT AREA */}
                <div className="p-4 bg-slate-900 border-t border-slate-700">
                    <div className="text-[10px] text-slate-500 mb-1 uppercase font-bold">Raw Command (RYLR / AT)</div>
                    <div className="flex gap-2">
                        <input 
                            type="text" 
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyDown={(e) => e.key === 'Enter' && handleSend()}
                            placeholder="Type AT command..."
                            className="flex-1 bg-slate-800 border border-slate-600 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 font-mono"
                        />
                        <button onClick={handleSend} className="bg-blue-600 hover:bg-blue-700 text-white p-2 rounded">
                            <Send size={16} />
                        </button>
                    </div>
                </div>
            </div>

            {/* CAPTCHA MODAL */}
            {captcha && (
                <div className="fixed inset-0 z-[60] flex items-center justify-center bg-black/80 backdrop-blur-sm">
                    <div className="bg-slate-900 border border-slate-600 rounded-xl p-6 w-80 shadow-2xl transform scale-100 animate-in fade-in zoom-in duration-200">
                        <h3 className="text-lg font-bold text-white mb-2 flex items-center gap-2">
                            <ShieldAlert className="text-red-500" /> SAFETY LOCK
                        </h3>
                        <p className="text-slate-400 text-sm mb-4">
                            You are about to send <strong>{captcha.cmd}</strong>. This is a critical action.
                        </p>
                        
                        <div className="bg-black/50 p-3 rounded text-center mb-4 border border-dashed border-slate-600">
                            <span className="text-slate-500 text-xs block mb-1">TYPE THIS CODE:</span>
                            <span className="text-2xl font-mono font-black text-white tracking-widest select-none">{captcha.challenge}</span>
                        </div>

                        <input 
                            type="text"
                            value={captchaInput}
                            onChange={(e) => setCaptchaInput(e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded p-2 text-center text-white font-bold tracking-widest mb-4 focus:border-red-500 outline-none"
                            placeholder="Enter Code"
                            autoFocus
                        />

                        <div className="flex gap-2">
                            <button onClick={() => setCaptcha(null)} className="flex-1 py-2 bg-slate-800 text-slate-300 rounded font-bold text-xs hover:bg-slate-700">CANCEL</button>
                            <button onClick={confirmCaptcha} className="flex-1 py-2 bg-red-600 text-white rounded font-bold text-xs hover:bg-red-700">CONFIRM</button>
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}