import { useState, useCallback, useRef } from 'react';
import { parseMotorSerialLine, MotorUpdate } from '../utils/motorParser';

export const useMotorSerial = (onUpdate: (update: MotorUpdate) => void) => {
    const [isConnected, setIsConnected] = useState(false);
    
    // We need refs for both Reader and Writer
    const portRef = useRef<any>(null);
    const readerRef = useRef<ReadableStreamDefaultReader<string> | null>(null);
    const writerRef = useRef<WritableStreamDefaultWriter<string> | null>(null);
    const keepReading = useRef(false);

    const connect = useCallback(async () => {
        const nav = navigator as any;
        if (!nav.serial) return alert("Web Serial API not supported.");

        try {
            const port = await nav.serial.requestPort();
            await port.open({ baudRate: 115200 }); 
            
            portRef.current = port;
            keepReading.current = true;

            // 1. Setup Reader (Incoming Data)
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
            const reader = textDecoder.readable.getReader();
            readerRef.current = reader;

            // 2. Setup Writer (Outgoing Commands)
            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
            const writer = textEncoder.writable.getWriter();
            writerRef.current = writer;

            setIsConnected(true);
            readLoop(reader, onUpdate);

        } catch (err) {
            console.error("Connection Failed:", err);
            setIsConnected(false);
        }
    }, [onUpdate]);

    // NEW: Function to send text commands
    const sendCommand = useCallback(async (cmd: string) => {
        if (!writerRef.current) return;
        try {
            // Append newline as most serial devices expect it
            await writerRef.current.write(cmd + "\r\n");
            console.log("Sent:", cmd);
        } catch (err) {
            console.error("Write Error:", err);
        }
    }, []);

    const readLoop = async (reader: any, callback: (u: MotorUpdate) => void) => {
        let buffer = "";
        try {
            while (keepReading.current) {
                const { value, done } = await reader.read();
                if (done) break;
                
                buffer += value;
                const lines = buffer.split('\n');
                
                for (let i = 0; i < lines.length - 1; i++) {
                    const line = lines[i].trim();
                    // Pass raw lines for the console log
                    callback({ type: 'DATA', raw: line }); 
                    
                    // Parse data lines
                    const parsed = parseMotorSerialLine(line);
                    if (parsed) callback(parsed);
                }
                buffer = lines[lines.length - 1];
            }
        } catch (e) {
            console.error(e);
        } finally {
            reader.releaseLock();
        }
    };

    const disconnect = useCallback(async () => {
        keepReading.current = false;
        
        if (readerRef.current) await readerRef.current.cancel();
        if (writerRef.current) {
            await writerRef.current.close();
            writerRef.current = null;
        }
        
        if (portRef.current) await portRef.current.close();
        setIsConnected(false);
    }, []);

    return { isConnected, connect, disconnect, sendCommand };
};