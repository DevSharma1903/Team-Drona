"use client";
import { useEffect, useState, ReactNode } from 'react';

// This component renders NOTHING on the server.
// It only renders 'children' after the browser has "hydrated".
export default function ClientOnly({ children }: { children: ReactNode }) {
    const [hasMounted, setHasMounted] = useState(false);

    useEffect(() => {
        setHasMounted(true);
    }, []);

    if (!hasMounted) {
        return (
            <div className="w-full h-full flex items-center justify-center bg-slate-50 dark:bg-slate-900 rounded-xl">
                <span className="text-xs font-mono text-slate-400 animate-pulse">INITIALIZING SYSTEM...</span>
            </div>
        );
    }

    return <>{children}</>;
}