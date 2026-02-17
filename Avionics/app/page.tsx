"use client";
import { useState, useEffect } from "react";
import dynamic from "next/dynamic";

// Dynamic imports (No changes here)
const FlightMode = dynamic(() => import("./features/flight/FlightMode"), { ssr: false });
const CSVMode = dynamic(() => import("./features/analysis/CSVMode"), { ssr: false });
const MotorMode = dynamic(() => import("./features/motor/MotorMode"), { ssr: false });

export default function Home() {
  const [isDark, setIsDark] = useState(true);
  const [currentView, setCurrentView] = useState("FLIGHT"); 
  
  // NEW: A counter to force re-renders
  const [resetKey, setResetKey] = useState(0); 

  useEffect(() => {
    if (isDark) document.documentElement.classList.add("dark");
    else document.documentElement.classList.remove("dark");
  }, [isDark]);

  const toggleTheme = () => setIsDark(!isDark);

  // --- NEW TAB HANDLER ---
  const handleTabChange = (tab: string) => {
    if (tab === currentView) {
        // If clicking the ALREADY OPEN tab -> Increment key to force reset
        setResetKey(prev => prev + 1);
    } else {
        // If clicking a NEW tab -> Switch view and reset counter
        setCurrentView(tab);
        setResetKey(0);
    }
  };

  return (
    <main className="h-screen w-screen bg-slate-50 dark:bg-black text-slate-900 dark:text-slate-100 flex flex-col overflow-hidden">
      
      {/* Home / Welcome Screen Logic */}
      {currentView === "HOME" && (
        <div className="flex flex-col items-center justify-center h-full">
           <h1 className="text-4xl font-bold">Arjuna FC Plotter</h1>
           <button 
             onClick={() => handleTabChange("FLIGHT")}
             className="mt-4 px-6 py-2 bg-blue-600 rounded-lg text-white font-bold"
           >
             Enter Flight Mode
           </button>
        </div>
      )}

      {/* MAGIC SAUCE: The `key` prop!
          - Normal: key="FLIGHT-0"
          - User clicks 'FLIGHT' again: key becomes "FLIGHT-1"
          - React detects the key change -> Destroys old component -> Mounts new one.
      */}

      {currentView === "FLIGHT" && (
        <FlightMode 
          key={`FLIGHT-${resetKey}`} 
          isDark={isDark} 
          toggleTheme={toggleTheme}
          activeTab={currentView}
          onTabChange={handleTabChange}
        />
      )}

      {currentView === "ANALYZE" && (
        <CSVMode 
          key={`ANALYZE-${resetKey}`} 
          isDark={isDark} 
          toggleTheme={toggleTheme}
          activeTab={currentView}
          onTabChange={handleTabChange}
        />
      )}

      {currentView === "MOTOR" && (
        <MotorMode 
            key={`MOTOR-${resetKey}`} 
            isDark={isDark} 
            toggleTheme={toggleTheme}
            activeTab={currentView}
            onTabChange={handleTabChange}
        />
      )}
    </main>
  );
}