"use client";
import { useRef, useMemo, useState, useCallback } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Line } from '@react-three/drei';
import * as THREE from 'three';

interface Props {
    altitude: number;
    trajectory: [number, number, number][]; // [lat, alt, lon]
    rotation?: { x: number, y: number, z: number }; 
    state?: number;
}

interface SceneProps extends Props {
    followMode: boolean;
}

// --- 1. FLAME COMPONENT ---
function RocketFlame() {
    const outerRef = useRef<THREE.Mesh>(null!);
    const innerRef = useRef<THREE.Mesh>(null!);
    
    useFrame(({ clock }) => {
        const t = clock.getElapsedTime();
        // Random flicker logic
        const flicker = 1 + (Math.sin(t * 50) * 0.1) + (Math.random() * 0.1); 
        
        if (outerRef.current) {
            outerRef.current.scale.set(flicker, flicker * (0.9 + Math.random() * 0.4), flicker);
        }
        if (innerRef.current) {
            innerRef.current.scale.set(flicker, flicker, flicker);
        }
    });

    return (
        <group position={[0, -2.7, 0]}>
            {/* Inner White Core */}
            <mesh ref={innerRef} position={[0, -0.6, 0]} rotation={[Math.PI, 0, 0]}>
                <coneGeometry args={[0.08, 1.5, 8]} />
                <meshBasicMaterial color="#fffbeb" transparent opacity={0.9} />
            </mesh>
            {/* Outer Orange Plume */}
            <mesh ref={outerRef} position={[0, -1.0, 0]} rotation={[Math.PI, 0, 0]}>
                <coneGeometry args={[0.25, 3.0, 16, 1, true]} />
                <meshBasicMaterial color="#f97316" transparent opacity={0.5} depthWrite={false} />
            </mesh>
            {/* Glow Light */}
            <pointLight color="#ea580c" intensity={4} distance={10} decay={2} position={[0, -1, 0]} />
        </group>
    );
}

// --- 2. ROCKET MESH (Visuals) ---
function SoundingRocketMesh({ isBoosting }: { isBoosting: boolean }) {
    return (
        <group position={[0, 3, 0]}>
            {/* Body */}
            <mesh position={[0, 0, 0]} castShadow receiveShadow>
                <cylinderGeometry args={[0.2, 0.2, 4.5, 32]} />
                <meshStandardMaterial color="#f8fafc" metalness={0.4} roughness={0.3} />
            </mesh>
            {/* Payload Band */}
            <mesh position={[0, 2.5, 0]} castShadow>
                <cylinderGeometry args={[0.205, 0.205, 0.8, 32]} />
                <meshStandardMaterial color="#1e293b" metalness={0.8} roughness={0.2} />
            </mesh>
            {/* Green Status Light */}
            <mesh position={[0, 2.5, 0]}>
                <cylinderGeometry args={[0.21, 0.21, 0.05, 32]} />
                <meshBasicMaterial color="#00ff00" toneMapped={false} />
            </mesh>
            {/* Nose Cone */}
            <mesh position={[0, 3.4, 0]} castShadow>
                <coneGeometry args={[0.205, 1.0, 32]} />
                <meshStandardMaterial color="#f97316" roughness={0.4} />
            </mesh>
            {/* Boat Tail */}
            <mesh position={[0, -2.4, 0]} castShadow>
                <cylinderGeometry args={[0.2, 0.15, 0.4, 32]} />
                <meshStandardMaterial color="#1e293b" />
            </mesh>
            {/* Nozzle */}
            <mesh position={[0, -2.7, 0]}>
                <cylinderGeometry args={[0.1, 0.18, 0.3, 32, 1, true]} />
                <meshStandardMaterial color="#0f172a" side={THREE.DoubleSide} metalness={0.5} />
            </mesh>
            {/* Fins */}
            {[0, Math.PI / 2, Math.PI, -Math.PI / 2].map((angle, i) => (
                <group key={i} rotation={[0, angle, 0]}>
                    <mesh position={[0.45, -2.0, 0]} castShadow>
                        <boxGeometry args={[0.5, 0.8, 0.05]} />
                        <meshStandardMaterial color="#ef4444" />
                    </mesh>
                </group>
            ))}
            
            {/* RENDER FLAME IF BOOSTING */}
            {isBoosting && <RocketFlame />}
        </group>
    );
}

// --- 3. SCENE CONTENT (Logic & Physics) ---
function SceneContent({ trajectory, altitude, rotation, state, followMode }: SceneProps) {
    const rocketRef = useRef<THREE.Group>(null);
    const controlsRef = useRef<any>(null);

    // State 1 = Boost Phase
    const isBoosting = state === 1;

    // --- COORDINATE MATH ---
    const origin = useMemo(() => {
        const firstValid = trajectory.find(p => Math.abs(p[0]) > 0.1 && Math.abs(p[2]) > 0.1);
        if (firstValid) return { lat: firstValid[0], lon: firstValid[2] };
        return null;
    }, [trajectory]);

    // FIXED: Wrapped in useCallback to satisfy linter
    const coordsToVector = useCallback((lat: number, alt: number, lon: number) => {
        if (!origin) return new THREE.Vector3(0, alt, 0);
        const latScale = 111320;
        const lonScale = 111320 * Math.cos(origin.lat * (Math.PI / 180));
        return new THREE.Vector3((lon - origin.lon) * lonScale, alt, (lat - origin.lat) * latScale * -1);
    }, [origin]);

    // FIXED: Added coordsToVector dependency
    const currentPos = useMemo(() => {
        if (trajectory.length === 0) return new THREE.Vector3(0, altitude, 0);
        const [lat, alt, lon] = trajectory[trajectory.length - 1];
        return coordsToVector(lat, alt, lon);
    }, [trajectory, altitude, coordsToVector]);

    // FIXED: Added coordsToVector dependency
    const trailPoints = useMemo(() => {
        if (!origin || trajectory.length < 2) return [];
        return trajectory.filter(p => Math.abs(p[0]) > 0.1).map(([lat, alt, lon]) => coordsToVector(lat, alt, lon));
    }, [trajectory, origin, coordsToVector]);

    // --- FRAME LOOP (Physics & Camera) ---
    useFrame((state, delta) => {
        if (!rocketRef.current || !controlsRef.current) return;

        // 1. Move Rocket
        rocketRef.current.position.lerp(currentPos, 0.5);

        // 2. Rotate Rocket
        if (rotation) {
            const targetEuler = new THREE.Euler(
                THREE.MathUtils.degToRad(rotation.x), 
                THREE.MathUtils.degToRad(rotation.y), 
                THREE.MathUtils.degToRad(rotation.z), 'XYZ'
            );
            const targetQuaternion = new THREE.Quaternion().setFromEuler(targetEuler);
            rocketRef.current.quaternion.slerp(targetQuaternion, 4 * delta);
        } else {
            rocketRef.current.rotation.set(0, 0, 0);
        }

        // 3. Chase Cam
        if (followMode) {
            const camera = state.camera;
            const controls = controlsRef.current;
            const offset = camera.position.clone().sub(controls.target);
            controls.target.lerp(rocketRef.current.position, 0.1);
            camera.position.copy(controls.target).add(offset);
            controls.update();
        }
    });

    return (
        <>
            <OrbitControls ref={controlsRef} makeDefault minDistance={5} maxDistance={500} enableDamping={true} dampingFactor={0.05} />
            <ambientLight intensity={0.6} />
            <directionalLight position={[50, 100, 25]} intensity={1.5} castShadow />
            <pointLight position={[-20, 20, -20]} intensity={0.5} color="#3b82f6" />
            <gridHelper args={[2000, 50, 0x334155, 0x1e293b]} position={[0, 0, 0]} />
            <axesHelper args={[20]} />

            <group ref={rocketRef}>
                <SoundingRocketMesh isBoosting={isBoosting} />
                <mesh position={[0, -currentPos.y + 0.1, 0]} rotation={[-Math.PI/2, 0, 0]}>
                    <ringGeometry args={[0.5, 2, 32]} />
                    <meshBasicMaterial color="black" opacity={0.2} transparent />
                </mesh>
            </group>

            {trailPoints.length > 1 && (
                <Line points={trailPoints} color="#fbbf24" lineWidth={2} transparent opacity={0.6} />
            )}
        </>
    );
}

// --- 4. MAIN EXPORT (Wraps Canvas) ---
export default function RocketModel(props: Props) {
    const [followMode, setFollowMode] = useState(true);

    return (
        <div className="w-full h-full relative bg-slate-950 rounded-lg overflow-hidden border border-slate-800 shadow-inner">
            <Canvas shadows camera={{ position: [20, 20, 20], fov: 50 }}>
                {/* SceneContent contains all the hooks (useFrame) */}
                <SceneContent {...props} followMode={followMode} />
            </Canvas>

            {/* Button is OUTSIDE the Canvas, so it stays fixed */}
            <div className="absolute top-24 right-4 z-10 flex flex-col items-end pointer-events-auto">
                <button 
                    onClick={() => setFollowMode(!followMode)}
                    className={`px-3 py-1.5 rounded text-[10px] font-bold uppercase border transition-colors shadow-lg backdrop-blur-md ${
                        followMode ? 'bg-blue-600/80 border-blue-400 text-white' : 'bg-slate-800/80 border-slate-600 text-slate-400'
                    }`}
                >
                    {followMode ? 'CAM LOCKED' : 'FREE CAM'}
                </button>
            </div>
        </div>
    );
}