"use client";
import { ReactNode } from 'react';
import Image from 'next/image';

interface Props {
    actions?: ReactNode; 
    isDark?: boolean;
    toggleTheme?: () => void;
    activeTab: string;
    onTabChange: (tab: string) => void;
}

export default function Header({ actions, isDark, toggleTheme, activeTab, onTabChange }: Props) {
    const getTabClass = (tabName: string) => {
        const isActive = activeTab === tabName;
        return `px-4 py-1.5 text-xs font-bold rounded-md transition-all ${
            isActive 
            ? "bg-white dark:bg-slate-700 text-blue-600 shadow-sm" 
            : "text-slate-500 hover:text-slate-700 dark:hover:text-slate-300"
        }`;
    };

    return (
        // Reset to a stable height (h-20) and ensure items-center keeps everything aligned vertically
        <header className="h-20 bg-white dark:bg-slate-900 border-b border-slate-200 dark:border-slate-800 grid grid-cols-[1.5fr_auto_1.5fr] items-center px-6 shrink-0 transition-colors duration-300">
            
            {/* LEFT: Logo & Brand */}
            <div className="flex items-center gap-4 overflow-hidden">
                {/* FIX: We use a fixed width/height container with 'relative'.
                    This prevents the logo from expanding and breaking the grid.
                */}
                <div className="relative w-32 h-12 shrink-0">
                    <Image 
                        src="/arjuna_logo.png" 
                        alt="Arjuna Logo"
                        fill
                        className="object-contain object-left" 
                        priority 
                    />
                </div>
                
                <div className="hidden lg:block shrink-0">
                    <h1 className="font-bold text-xl leading-tight text-slate-900 dark:text-white tracking-tight">
                        Arjuna <span className="text-blue-500">FC</span>
                    </h1>
                    <p className="text-[10px] font-bold text-slate-400 tracking-widest uppercase">Mission Control</p>
                </div>
            </div>

            {/* CENTER: Navigation (Pinned) */}
            <div className="flex items-center bg-slate-100 dark:bg-slate-800 p-1 rounded-lg justify-self-center">
                <button onClick={() => onTabChange('FLIGHT')} className={getTabClass('FLIGHT')}>FLIGHT</button>
                <button onClick={() => onTabChange('MOTOR')} className={getTabClass('MOTOR')}>MOTOR</button>
                <button onClick={() => onTabChange('ANALYZE')} className={getTabClass('ANALYZE')}>ANALYZE</button>
            </div>

            {/* RIGHT: Dynamic Actions (Aligned Right) */}
            <div className="flex items-center gap-4 justify-self-end">
                <div className="flex items-center gap-2">{actions}</div>
                {toggleTheme && (
                    <>
                        <div className="h-6 w-[1px] bg-slate-200 dark:bg-slate-700"></div>
                        <button onClick={toggleTheme} className="p-2 rounded-full hover:bg-slate-100 dark:hover:bg-slate-800 text-slate-500 transition-all">
                            {isDark ? "🌙" : "☀️"}
                        </button>
                    </>
                )}
            </div>
        </header>
    );
}