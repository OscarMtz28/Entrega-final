/**
 * app.js
 * Lógica autocontenida de simulación de cardumen (Boids) 3D en Babylon.js.
 * Consolida todas las clases (Spatial Hashing, Física, Mesh Procedural, Shaders y Loop)
 * en un solo archivo para evitar errores de CORS al ejecutar localmente.
 */

// ============================================================================
// 1. GRID UNIFORME 3D (SPATIAL HASHING) PARA OPTIMIZACIÓN O(N)
// ============================================================================
class SpatialHash3D {
    constructor(cellSize) {
        this.cellSize = cellSize;
        this.grid = new Map();
    }

    setCellSize(newCellSize) {
        this.cellSize = newCellSize;
    }

    clear() {
        this.grid.clear();
    }

    insert(id, position) {
        const key = this._getKey(position);
        if (!this.grid.has(key)) {
            this.grid.set(key, []);
        }
        this.grid.get(key).push(id);
    }

    getNearby(position) {
        const cx = Math.floor(position.x / this.cellSize);
        const cy = Math.floor(position.y / this.cellSize);
        const cz = Math.floor(position.z / this.cellSize);

        const nearby = [];

        // Consultar cubo de 3x3x3 celdas contiguas
        for (let x = cx - 1; x <= cx + 1; x++) {
            for (let y = cy - 1; y <= cy + 1; y++) {
                for (let z = cz - 1; z <= cz + 1; z++) {
                    const key = `${x},${y},${z}`;
                    const cell = this.grid.get(key);
                    if (cell) {
                        for (let i = 0; i < cell.length; i++) {
                            nearby.push(cell[i]);
                        }
                    }
                }
            }
        }

        return nearby;
    }

    _getKey(position) {
        const gx = Math.floor(position.x / this.cellSize);
        const gy = Math.floor(position.y / this.cellSize);
        const gz = Math.floor(position.z / this.cellSize);
        return `${gx},${gy},${gz}`;
    }
}

// ============================================================================
// 2. MOTOR FÍSICO DE BOIDS (FLOCKING)
// ============================================================================
class Boid {
    constructor(id, position, velocity) {
        this.id = id;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = new BABYLON.Vector3(0, 0, 0);
        this.swimOffset = Math.random() * 100.0;
        this.swimSpeedMultiplier = 0.8 + Math.random() * 0.4;
    }
}

class BoidsSimulation {
    constructor(count, minBounds, maxBounds) {
        this.minBounds = minBounds;
        this.maxBounds = maxBounds;
        this.boids = [];
        this.count = 0;

        this.spatialHash = new SpatialHash3D(5.0);

        // Preasignación de vectores y matrices para evitar allocación en caliente
        this._tempV1 = new BABYLON.Vector3(0, 0, 0);
        this._tempV2 = new BABYLON.Vector3(0, 0, 0);
        this._cohesionSum = new BABYLON.Vector3(0, 0, 0);
        this._alignmentSum = new BABYLON.Vector3(0, 0, 0);
        this._separationSum = new BABYLON.Vector3(0, 0, 0);
        this._boundaryForce = new BABYLON.Vector3(0, 0, 0);

        this._scaling = new BABYLON.Vector3(1, 1, 1);
        this._quat = new BABYLON.Quaternion();
        this._up = new BABYLON.Vector3(0, 1, 0);
        this._tempMatrix = new BABYLON.Matrix();

        this.setCount(count);
    }

    setCount(newCount) {
        if (newCount === this.count) return;

        if (newCount < this.count) {
            this.boids.length = newCount;
        } else {
            const toAdd = newCount - this.count;
            const center = BABYLON.Vector3.Center(this.minBounds, this.maxBounds);

            for (let i = 0; i < toAdd; i++) {
                const id = this.boids.length;
                const radius = 8.0;
                const theta = Math.random() * Math.PI * 2;
                const phi = Math.acos(Math.random() * 2 - 1);

                const pos = new BABYLON.Vector3(
                    center.x + Math.sin(phi) * Math.cos(theta) * radius,
                    center.y + Math.sin(phi) * Math.sin(theta) * radius,
                    center.z + Math.cos(phi) * radius
                );

                const vel = new BABYLON.Vector3(
                    Math.random() * 2 - 1,
                    Math.random() * 2 - 1,
                    Math.random() * 2 - 1
                ).normalize().scaleInPlace(2.0);

                this.boids.push(new Boid(id, pos, vel));
            }
        }
        this.count = newCount;
    }

    reset() {
        const center = BABYLON.Vector3.Center(this.minBounds, this.maxBounds);
        for (let i = 0; i < this.count; i++) {
            const boid = this.boids[i];
            const radius = 5.0 + Math.random() * 5.0;
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos(Math.random() * 2 - 1);

            boid.position.set(
                center.x + Math.sin(phi) * Math.cos(theta) * radius,
                center.y + Math.sin(phi) * Math.sin(theta) * radius,
                center.z + Math.cos(phi) * radius
            );

            boid.velocity.set(
                Math.random() * 2 - 1,
                Math.random() * 2 - 1,
                Math.random() * 2 - 1
            ).normalize().scaleInPlace(2.0);

            boid.acceleration.set(0, 0, 0);
        }
    }

    update(dt, params) {
        dt = Math.min(dt, 0.05);
        this.spatialHash.setCellSize(params.perceptionRadius);
        this.spatialHash.clear();

        for (let i = 0; i < this.count; i++) {
            this.spatialHash.insert(this.boids[i].id, this.boids[i].position);
        }

        const maxSpeed = params.maxSpeed;
        const maxForce = params.maxForce;
        const perceptionRadiusSq = params.perceptionRadius * params.perceptionRadius;
        const separationRadiusSq = (params.perceptionRadius * 0.6) * (params.perceptionRadius * 0.6);

        for (let i = 0; i < this.count; i++) {
            const boid = this.boids[i];

            this._cohesionSum.set(0, 0, 0);
            this._alignmentSum.set(0, 0, 0);
            this._separationSum.set(0, 0, 0);

            let cohesionCount = 0;
            let alignmentCount = 0;
            let separationCount = 0;

            const neighbors = this.spatialHash.getNearby(boid.position);

            for (let j = 0; j < neighbors.length; j++) {
                const neighborId = neighbors[j];
                if (neighborId === boid.id) continue;

                const neighbor = this.boids[neighborId];
                if (!neighbor) continue;

                const dx = neighbor.position.x - boid.position.x;
                const dy = neighbor.position.y - boid.position.y;
                const dz = neighbor.position.z - boid.position.z;
                const distSq = dx * dx + dy * dy + dz * dz;

                if (distSq < perceptionRadiusSq && distSq > 0.0001) {
                    const dist = Math.sqrt(distSq);

                    this._cohesionSum.addInPlace(neighbor.position);
                    cohesionCount++;

                    this._alignmentSum.addInPlace(neighbor.velocity);
                    alignmentCount++;

                    if (distSq < separationRadiusSq) {
                        const repulsionForce = 1.0 / dist;
                        this._tempV1.set(-dx, -dy, -dz).normalize().scaleInPlace(repulsionForce);
                        this._separationSum.addInPlace(this._tempV1);
                        separationCount++;
                    }
                }
            }

            const cohesionForce = boid.acceleration;
            cohesionForce.set(0, 0, 0);
            if (cohesionCount > 0) {
                this._cohesionSum.scaleInPlace(1.0 / cohesionCount);
                this._cohesionSum.subtractToRef(boid.position, this._tempV1);
                this._steerTowards(this._tempV1, boid.velocity, maxSpeed, maxForce, cohesionForce);
            }

            const alignmentForce = this._tempV2;
            alignmentForce.set(0, 0, 0);
            if (alignmentCount > 0) {
                this._alignmentSum.scaleInPlace(1.0 / alignmentCount);
                this._steerTowards(this._alignmentSum, boid.velocity, maxSpeed, maxForce, alignmentForce);
            }

            const separationForce = this._cohesionSum;
            separationForce.set(0, 0, 0);
            if (separationCount > 0) {
                this._separationSum.scaleInPlace(1.0 / separationCount);
                this._steerTowards(this._separationSum, boid.velocity, maxSpeed, maxForce, separationForce);
            }

            this._calculateBoundaryForce(boid.position, this._boundaryForce);

            boid.acceleration.scaleInPlace(params.wCohesion);
            boid.acceleration.addInPlace(alignmentForce.scaleInPlace(params.wAlignment));
            boid.acceleration.addInPlace(separationForce.scaleInPlace(params.wSeparation));
            boid.acceleration.addInPlace(this._boundaryForce);

            const accelLength = boid.acceleration.length();
            if (accelLength > maxForce) {
                boid.acceleration.scaleInPlace(maxForce / accelLength);
            }

            boid.velocity.addInPlace(boid.acceleration.scaleInPlace(dt));

            const speed = boid.velocity.length();
            if (speed > maxSpeed) {
                boid.velocity.scaleInPlace(maxSpeed / speed);
            } else if (speed < 0.8) {
                boid.velocity.normalize().scaleInPlace(0.8);
            }

            boid.position.addInPlace(boid.velocity.scaleInPlace(dt));
        }
    }

    _calculateBoundaryForce(pos, forceOut) {
        forceOut.set(0, 0, 0);
        const margin = 5.0;
        const forceIntensity = 0.8;

        if (pos.x < this.minBounds.x + margin) {
            forceOut.x = ((this.minBounds.x + margin) - pos.x) * forceIntensity;
        } else if (pos.x > this.maxBounds.x - margin) {
            forceOut.x = -((pos.x - (this.maxBounds.x - margin))) * forceIntensity;
        }

        if (pos.y < this.minBounds.y + margin) {
            forceOut.y = ((this.minBounds.y + margin) - pos.y) * forceIntensity;
        } else if (pos.y > this.maxBounds.y - margin) {
            forceOut.y = -((pos.y - (this.maxBounds.y - margin))) * forceIntensity;
        }

        if (pos.z < this.minBounds.z + margin) {
            forceOut.z = ((this.minBounds.z + margin) - pos.z) * forceIntensity;
        } else if (pos.z > this.maxBounds.z - margin) {
            forceOut.z = -((pos.z - (this.maxBounds.z - margin))) * forceIntensity;
        }
    }

    _steerTowards(desiredDirection, currentVelocity, maxSpeed, maxForce, forceOut) {
        const desiredLength = desiredDirection.length();
        if (desiredLength > 0.0001) {
            desiredDirection.scaleToRef(maxSpeed / desiredLength, this._tempV1);
            this._tempV1.subtractToRef(currentVelocity, forceOut);

            const steerLength = forceOut.length();
            if (steerLength > maxForce) {
                forceOut.scaleInPlace(maxForce / steerLength);
            }
        } else {
            forceOut.set(0, 0, 0);
        }
    }

    getThinInstanceData(matricesArray, offsetsArray, speedsArray) {
        const tempMat = this._tempMatrix;

        for (let i = 0; i < this.count; i++) {
            const boid = this.boids[i];

            const speed = boid.velocity.length();
            if (speed > 0.0001) {
                boid.velocity.scaleToRef(1.0 / speed, this._tempV1);
            } else {
                this._tempV1.set(0, 0, 1);
            }

            BABYLON.Quaternion.FromLookDirectionLHToRef(this._tempV1, this._up, this._quat);
            BABYLON.Matrix.ComposeToRef(this._scaling, this._quat, boid.position, tempMat);

            const mOffset = i * 16;
            matricesArray[mOffset] = tempMat.m[0];
            matricesArray[mOffset + 1] = tempMat.m[1];
            matricesArray[mOffset + 2] = tempMat.m[2];
            matricesArray[mOffset + 3] = tempMat.m[3];

            matricesArray[mOffset + 4] = tempMat.m[4];
            matricesArray[mOffset + 5] = tempMat.m[5];
            matricesArray[mOffset + 6] = tempMat.m[6];
            matricesArray[mOffset + 7] = tempMat.m[7];

            matricesArray[mOffset + 8] = tempMat.m[8];
            matricesArray[mOffset + 9] = tempMat.m[9];
            matricesArray[mOffset + 10] = tempMat.m[10];
            matricesArray[mOffset + 11] = tempMat.m[11];

            matricesArray[mOffset + 12] = tempMat.m[12];
            matricesArray[mOffset + 13] = tempMat.m[13];
            matricesArray[mOffset + 14] = tempMat.m[14];
            matricesArray[mOffset + 15] = tempMat.m[15];

            offsetsArray[i] = boid.swimOffset;
            speedsArray[i] = boid.swimSpeedMultiplier;
        }
    }
}

// ============================================================================
// 3. GENERADOR DE GEOMETRÍA PROCEDURAL DEL PEZ
// ============================================================================
function createFishMesh(scene) {
    const mesh = new BABYLON.Mesh("proceduralFish", scene);

    const rings = [
        { z: 0.6, w: 0.001, h: 0.001 },
        { z: 0.5, w: 0.08, h: 0.12 },
        { z: 0.3, w: 0.15, h: 0.22 },
        { z: 0.0, w: 0.18, h: 0.28 },
        { z: -0.3, w: 0.12, h: 0.20 },
        { z: -0.55, w: 0.05, h: 0.09 },
        { z: -0.75, w: 0.01, h: 0.22 },
        { z: -0.95, w: 0.002, h: 0.35 }
    ];

    const radialSegments = 8;
    const positions = [];
    const indices = [];
    const uvs = [];

    // Generar cuerpo
    for (let r = 0; r < rings.length; r++) {
        const ring = rings[r];
        const z = ring.z;
        const w = ring.w;
        const h = ring.h;
        const v = (z - (-0.95)) / (0.6 - (-0.95));

        for (let s = 0; s < radialSegments; s++) {
            const angle = (s / radialSegments) * Math.PI * 2;
            const x = Math.cos(angle) * w;
            const y = Math.sin(angle) * h;

            positions.push(x, y, z);
            const u = s / radialSegments;
            uvs.push(u, v);
        }
    }

    for (let r = 0; r < rings.length - 1; r++) {
        for (let s = 0; s < radialSegments; s++) {
            const nextS = (s + 1) % radialSegments;
            const idx0 = r * radialSegments + s;
            const idx1 = r * radialSegments + nextS;
            const idx2 = (r + 1) * radialSegments + s;
            const idx3 = (r + 1) * radialSegments + nextS;

            indices.push(idx0, idx3, idx1);
            indices.push(idx0, idx2, idx3);
        }
    }

    const bodyVerticesCount = positions.length / 3;

    // Aleta Dorsal
    const idxTop3 = 3 * radialSegments + 2;
    const idxTop4 = 4 * radialSegments + 2;

    positions.push(0.0, 0.45, -0.15);
    positions.push(0.0, 0.35, -0.45);
    uvs.push(0.5, 0.5);
    uvs.push(0.5, 0.3);

    const d0 = bodyVerticesCount;
    const d1 = bodyVerticesCount + 1;

    indices.push(idxTop3, d0, idxTop4);
    indices.push(idxTop4, d0, d1);
    indices.push(idxTop3, idxTop4, d0);
    indices.push(idxTop4, d1, d0);

    // Aleta Pectoral Derecha
    const idxRight2 = 2 * radialSegments + 0;
    const idxRight3 = 3 * radialSegments + 0;
    positions.push(0.38, -0.05, 0.15);
    uvs.push(1.0, 0.7);
    const pR = bodyVerticesCount + 2;

    indices.push(idxRight2, pR, idxRight3);
    indices.push(idxRight2, idxRight3, pR);

    // Aleta Pectoral Izquierda
    const idxLeft2 = 2 * radialSegments + 4;
    const idxLeft3 = 3 * radialSegments + 4;
    positions.push(-0.38, -0.05, 0.15);
    uvs.push(0.0, 0.7);
    const pL = bodyVerticesCount + 3;

    indices.push(idxLeft2, idxLeft3, pL);
    indices.push(idxLeft2, pL, idxLeft3);

    const normals = [];
    BABYLON.VertexData.ComputeNormals(positions, indices, normals);

    const vertexData = new BABYLON.VertexData();
    vertexData.positions = positions;
    vertexData.indices = indices;
    vertexData.normals = normals;
    vertexData.uvs = uvs;
    vertexData.applyToMesh(mesh);

    mesh.scaling.set(1.5, 1.5, 1.5);

    return mesh;
}

// ============================================================================
// 4. SHADERS GLSL PERSONALIZADOS (T4 & T5)
// ============================================================================
const vertexShaderSource = `
precision highp float;

attribute vec3 position;
attribute vec3 normal;
attribute vec2 uv;

#ifdef INSTANCES
attribute vec4 world0;
attribute vec4 world1;
attribute vec4 world2;
attribute vec4 world3;
attribute float swimOffset;
attribute float swimSpeedMultiplier;
#else
uniform mat4 world;
#endif

uniform mat4 viewProjection;
uniform float time;
uniform float swimAmplitude;
uniform float swimFrequency;

varying vec3 vPositionW;
varying vec3 vNormalW;
varying vec2 vUV;
varying float vSwimPhase;

void main() {
    #ifdef INSTANCES
    mat4 finalWorld = mat4(world0, world1, world2, world3);
    float phaseOffset = swimOffset;
    float speedMult = swimSpeedMultiplier;
    #else
    mat4 finalWorld = world;
    float phaseOffset = 0.0;
    float speedMult = 1.0;
    #endif

    vec3 localPosition = position;
    
    // Wobble lateral (cabeza quieta, cola ondea)
    float tailFactor = smoothstep(0.3, -0.95, localPosition.z);
    float waveLength = 4.5;
    float phase = time * swimFrequency * speedMult + localPosition.z * waveLength + phaseOffset;
    float wobble = sin(phase) * swimAmplitude * tailFactor;
    localPosition.x += wobble;

    vec4 worldPos = finalWorld * vec4(localPosition, 1.0);
    vPositionW = worldPos.xyz;
    vNormalW = normalize(vec3(finalWorld * vec4(normal, 0.0)));
    vUV = uv;
    vSwimPhase = phase;

    gl_Position = viewProjection * worldPos;
}
`;

const fragmentShaderSource = `
precision highp float;

uniform vec3 cameraPosition;
uniform vec3 lightDirection;
uniform vec3 ambientColor;
uniform vec3 directionalLightColor;
uniform vec3 specularColor;
uniform float shininess;
uniform float shadingStyle;
uniform float causticsIntensity;
uniform float time;
uniform vec3 fogColor;
uniform float fogDensity;

varying vec3 vPositionW;
varying vec3 vNormalW;
varying vec2 vUV;
varying float vSwimPhase;

void main() {
    vec3 N = normalize(vNormalW);
    vec3 L = normalize(lightDirection);
    vec3 V = normalize(cameraPosition - vPositionW);
    vec3 H = normalize(L + V);

    // Textura procedural neón en base a UVs
    vec3 fishColor;
    if (vUV.y > 0.78) {
        fishColor = mix(vec3(1.0, 0.45, 0.05), vec3(1.0, 0.15, 0.0), (vUV.y - 0.78) / 0.22);
    } else {
        float distToSide1 = abs(vUV.x - 0.0);
        float distToSide2 = abs(vUV.x - 0.5);
        float distToSide3 = abs(vUV.x - 1.0);
        
        float lateralStripe = smoothstep(0.12, 0.01, distToSide1) + 
                              smoothstep(0.12, 0.01, distToSide2) + 
                              smoothstep(0.12, 0.01, distToSide3);
                              
        float verticalFactor = sin(vUV.x * 2.0 * 3.14159265);
        
        vec3 backColor = vec3(0.04, 0.10, 0.18);
        vec3 bellyColor = vec3(0.95, 0.25, 0.08);
        vec3 neonColor = vec3(0.0, 0.85, 1.0);
        
        vec3 bodyBase;
        if (verticalFactor > 0.0) {
            bodyBase = mix(vec3(0.08, 0.22, 0.28), backColor, verticalFactor);
        } else {
            bodyBase = mix(vec3(0.08, 0.22, 0.28), bellyColor, -verticalFactor);
        }
        
        fishColor = mix(bodyBase, neonColor, clamp(lateralStripe, 0.0, 1.0) * 0.9);

        if (verticalFactor > 0.3) {
            float stripes = sin(vUV.y * 30.0) * 0.5 + 0.5;
            stripes = step(0.7, stripes);
            fishColor = mix(fishColor, backColor * 0.5, stripes * (verticalFactor - 0.3) * 1.5);
        }
    }

    // Iluminación
    float diffSmooth = max(dot(N, L), 0.0);
    float specSmooth = pow(max(dot(N, H), 0.0), shininess);

    // Cel-Shading
    float diffToon;
    if (diffSmooth > 0.78) {
        diffToon = 1.0;
    } else if (diffSmooth > 0.45) {
        diffToon = 0.65;
    } else if (diffSmooth > 0.15) {
        diffToon = 0.30;
    } else {
        diffToon = 0.08;
    }
    float specToon = step(0.5, specSmooth);

    // Interpolación de sombreado
    float diffFactor;
    float specFactor;
    if (shadingStyle < 1.0) {
        diffFactor = mix(diffSmooth, diffToon, shadingStyle);
        specFactor = mix(specSmooth, specToon, shadingStyle);
    } else {
        diffFactor = mix(diffToon, diffSmooth * step(0.1, diffSmooth), shadingStyle - 1.0);
        specFactor = mix(specToon, specSmooth * step(0.5, specSmooth), shadingStyle - 1.0);
    }

    vec3 diffuseLight = ambientColor + diffFactor * directionalLightColor;
    vec3 specularLight = specularColor * specFactor;
    vec3 litColor = fishColor * diffuseLight + specularLight;

    // Cáusticas procedimentales 3D
    vec2 causticUV = vPositionW.xz * 0.18;
    float cTime = time * 1.4;
    float wave1 = sin(causticUV.x * 2.2 + cTime) + cos(causticUV.y * 1.8 - cTime * 0.6);
    float wave2 = sin(causticUV.y * 2.6 + cTime * 0.9) + cos(causticUV.x * 1.4 - cTime * 0.5);
    float causticWave = (wave1 + wave2) * 0.5;
    float causticPattern = pow(max(0.0, causticWave), 3.0);
    float depthAttenuation = clamp((vPositionW.y + 15.0) / 25.0, 0.0, 1.0);
    vec3 causticColor = vec3(0.5, 0.92, 1.0) * causticPattern * causticsIntensity * depthAttenuation;
    
    litColor += causticColor * fishColor;

    // Niebla
    float depth = gl_FragCoord.z / gl_FragCoord.w;
    float fogFactor = exp2(-fogDensity * fogDensity * depth * depth * 1.442695);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    
    vec3 finalColor = mix(fogColor, litColor, fogFactor);

    gl_FragColor = vec4(finalColor, 1.0);
}
`;

function createFishMaterial(scene) {
    // Registrar los shaders en el almacén global para máxima compatibilidad
    BABYLON.Effect.ShadersStore["fishShaderVertexShader"] = vertexShaderSource;
    BABYLON.Effect.ShadersStore["fishShaderFragmentShader"] = fragmentShaderSource;

    const material = new BABYLON.ShaderMaterial(
        "fishShaderMaterial",
        scene,
        {
            vertex: "fishShader",
            fragment: "fishShader",
        },
        {
            attributes: [
                "position", "normal", "uv",
                "world0", "world1", "world2", "world3",
                "swimOffset", "swimSpeedMultiplier"
            ],
            uniforms: [
                "world", "viewProjection", "time",
                "swimAmplitude", "swimFrequency", "cameraPosition",
                "lightDirection", "ambientColor", "directionalLightColor",
                "specularColor", "shininess", "shadingStyle",
                "causticsIntensity", "fogColor", "fogDensity"
            ],
            defines: ["INSTANCES"]
        }
    );

    // Iniciales
    material.setVector3("lightDirection", new BABYLON.Vector3(0.3, 1.0, 0.3).normalize());
    material.setVector3("ambientColor", new BABYLON.Vector3(0.08, 0.18, 0.28));
    material.setVector3("directionalLightColor", new BABYLON.Vector3(0.7, 0.9, 1.0));
    material.setVector3("specularColor", new BABYLON.Vector3(0.8, 0.95, 1.0));
    material.setFloat("shininess", 32.0);
    material.setFloat("shadingStyle", 0.0);
    material.setFloat("causticsIntensity", 0.5);
    material.setFloat("swimAmplitude", 0.2);
    material.setFloat("swimFrequency", 6.0);
    material.setFloat("time", 0.0);
    material.setVector3("fogColor", new BABYLON.Vector3(0.02, 0.08, 0.16));
    material.setFloat("fogDensity", 0.015);

    return material;
}

// ============================================================================
// 5. ORQUESTADOR Y BUCLE PRINCIPAL (RENDER & EVENT BINDINGS)
// ============================================================================
window.addEventListener('load', () => {
    const canvas = document.getElementById('renderCanvas');
    const engine = new BABYLON.Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true });
    const scene = new BABYLON.Scene(engine);

    // Océano y Niebla
    scene.clearColor = new BABYLON.Color4(0.01, 0.05, 0.12, 1.0);
    scene.fogMode = BABYLON.Scene.FOGMODE_EXP2;
    scene.fogColor = new BABYLON.Color3(0.02, 0.08, 0.16);
    scene.fogDensity = 0.015;

    // Cámara ArcRotate
    const camera = new BABYLON.ArcRotateCamera(
        "oceanCamera",
        -Math.PI / 2,
        Math.PI / 2.2,
        35.0,
        new BABYLON.Vector3(0, 0, 0),
        scene
    );
    camera.lowerRadiusLimit = 12.0;
    camera.upperRadiusLimit = 65.0;
    camera.panningSensibility = 500;
    camera.useBouncingBehavior = true;
    camera.attachControl(canvas, true);

    // Iluminación
    const ambientLight = new BABYLON.HemisphericLight("ambientWaterLight", new BABYLON.Vector3(0, 1, 0), scene);
    ambientLight.diffuse = new BABYLON.Color3(0.12, 0.28, 0.38);
    ambientLight.groundColor = new BABYLON.Color3(0.01, 0.04, 0.08);
    ambientLight.intensity = 0.8;

    const sunLight = new BABYLON.DirectionalLight("sunLight", new BABYLON.Vector3(-0.35, -1.0, -0.25), scene);
    sunLight.position = new BABYLON.Vector3(15, 30, 10);
    sunLight.intensity = 1.2;
    sunLight.diffuse = new BABYLON.Color3(0.75, 0.93, 1.0);
    sunLight.specular = new BABYLON.Color3(0.6, 0.85, 1.0);

    // Sombras
    const shadowGenerator = new BABYLON.ShadowGenerator(1024, sunLight);
    shadowGenerator.useBlurExponentialShadowMap = true;
    shadowGenerator.blurKernel = 16;
    shadowGenerator.depthScale = 45.0;

    // Fondo Marino
    const ground = BABYLON.MeshBuilder.CreateGround("seabed", { width: 120, height: 120, subdivisions: 32 }, scene);
    ground.position.y = -15.0;

    const groundMaterial = new BABYLON.StandardMaterial("seabedMat", scene);
    groundMaterial.diffuseColor = new BABYLON.Color3(0.06, 0.15, 0.22);
    groundMaterial.specularColor = new BABYLON.Color3(0.05, 0.1, 0.1);
    groundMaterial.roughness = 0.95;
    ground.material = groundMaterial;
    ground.receiveShadows = true;

    // Dinámico: burbujas procedurales
    const bubbleTexture = new BABYLON.DynamicTexture("bubbleTexture", 64, scene);
    const ctx = bubbleTexture.getContext();
    ctx.fillStyle = "rgba(0, 0, 0, 0)";
    ctx.fillRect(0, 0, 64, 64);
    ctx.beginPath();
    ctx.arc(32, 32, 28, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(255, 255, 255, 0.1)";
    ctx.fill();
    ctx.lineWidth = 3;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.65)";
    ctx.stroke();
    bubbleTexture.update();

    const bubbleParticles = new BABYLON.ParticleSystem("bubbles", 150, scene);
    bubbleParticles.particleTexture = bubbleTexture;
    bubbleParticles.emitter = new BABYLON.Vector3(0, -14.5, 0);
    bubbleParticles.minEmitBox = new BABYLON.Vector3(-30, 0, -30);
    bubbleParticles.maxEmitBox = new BABYLON.Vector3(30, 0, 30);
    bubbleParticles.color1 = new BABYLON.Color4(0.8, 0.95, 1.0, 0.7);
    bubbleParticles.color2 = new BABYLON.Color4(0.5, 0.8, 1.0, 0.35);
    bubbleParticles.colorDead = new BABYLON.Color4(0.02, 0.08, 0.16, 0.0);
    bubbleParticles.minSize = 0.08;
    bubbleParticles.maxSize = 0.28;
    bubbleParticles.minLifeTime = 5.0;
    bubbleParticles.maxLifeTime = 9.0;
    bubbleParticles.emitRate = 25;
    bubbleParticles.gravity = new BABYLON.Vector3(0, 1.2, 0);
    bubbleParticles.start();

    // Límites de Boids
    const minSimBounds = new BABYLON.Vector3(-25, -12, -25);
    const maxSimBounds = new BABYLON.Vector3(25, 12, 25);

    // Parámetros por defecto
    const params = {
        wCohesion: 0.6,
        wAlignment: 1.0,
        wSeparation: 2.2,
        perceptionRadius: 5.0,
        maxSpeed: 4.0,
        maxForce: 1.2,
        fishCount: 300,
        swimAmplitude: 0.20,
        swimFrequency: 6.0,
        shadingStyle: 0.0,
        causticsIntensity: 0.5
    };

    const sim = new BoidsSimulation(params.fishCount, minSimBounds, maxSimBounds);

    // Crear Mallas y Shaders
    const fishMesh = createFishMesh(scene);
    const fishMaterial = createFishMaterial(scene);
    fishMesh.material = fishMaterial;

    shadowGenerator.addShadowCaster(fishMesh);

    // Registrar atributos de Thin Instance
    fishMesh.thinInstanceRegisterAttribute("swimOffset", 1);
    fishMesh.thinInstanceRegisterAttribute("swimSpeedMultiplier", 1);

    const MAX_PECES = 1000;
    const matricesData = new Float32Array(MAX_PECES * 16);
    const offsetsData = new Float32Array(MAX_PECES * 1);
    const speedsData = new Float32Array(MAX_PECES * 1);

    sim.getThinInstanceData(matricesData, offsetsData, speedsData);

    // Configurar buffers iniciales
    fishMesh.thinInstanceSetBuffer("matrix", matricesData, 16, false);
    fishMesh.thinInstanceSetBuffer("swimOffset", offsetsData, 1, false);
    fishMesh.thinInstanceSetBuffer("swimSpeedMultiplier", speedsData, 1, false);
    fishMesh.thinInstanceCount = params.fishCount;

    // Vincular Controles HTML
    const bindUI = () => {
        const sliderBindings = [
            { id: 'w-coh', param: 'wCohesion', viewId: 'v-coh', format: v => v.toFixed(1) },
            { id: 'w-ali', param: 'wAlignment', viewId: 'v-ali', format: v => v.toFixed(1) },
            { id: 'w-sep', param: 'wSeparation', viewId: 'v-sep', format: v => v.toFixed(1) },
            { id: 'perception-rad', param: 'perceptionRadius', viewId: 'v-perception', format: v => v.toFixed(1) },
            { id: 'max-speed', param: 'maxSpeed', viewId: 'v-speed', format: v => v.toFixed(1) },
            { id: 'swim-amplitude', param: 'swimAmplitude', viewId: 'v-amplitude', format: v => v.toFixed(2) },
            { id: 'swim-frequency', param: 'swimFrequency', viewId: 'v-frequency', format: v => v.toFixed(1) },
            { id: 'caustics-intensity', param: 'causticsIntensity', viewId: 'v-caustics', format: v => v.toFixed(1) }
        ];

        sliderBindings.forEach(binding => {
            const input = document.getElementById(binding.id);
            const display = document.getElementById(binding.viewId);

            input.addEventListener('input', (e) => {
                const val = parseFloat(e.target.value);
                params[binding.param] = val;
                display.textContent = binding.format(val);
            });
        });

        const countInput = document.getElementById('fish-count');
        const countDisplay = document.getElementById('v-count');
        countInput.addEventListener('input', (e) => {
            const val = parseInt(e.target.value);
            params.fishCount = val;
            countDisplay.textContent = val;
            sim.setCount(val);
        });

        const shadingInput = document.getElementById('shading-mode');
        const shadingDisplay = document.getElementById('v-shading');
        const shadingLabels = ["Realista (Blinn-Phong)", "Híbrido (Soft Toon)", "Cel-Shaded (Retro Toon)"];

        shadingInput.addEventListener('input', (e) => {
            const val = parseInt(e.target.value);
            params.shadingStyle = val;
            shadingDisplay.textContent = shadingLabels[val];
        });

        document.getElementById('reset-button').addEventListener('click', () => {
            sim.reset();
        });

        // Botón Ocultar/Mostrar Menú (UI Colapsable)
        const toggleBtn = document.getElementById('toggle-menu-btn');
        const uiContainer = document.getElementById('ui-container');
        toggleBtn.addEventListener('click', () => {
            uiContainer.classList.toggle('collapsed');
            if (uiContainer.classList.contains('collapsed')) {
                toggleBtn.textContent = '▶';
            } else {
                toggleBtn.textContent = '◀';
            }
        });
    };

    bindUI();

    // Contadores FPS
    const fpsCounter = document.getElementById('fps-counter');
    const agentCounter = document.getElementById('agent-counter');
    let frameTimes = 0;
    let globalTime = 0;

    // Render Loop
    engine.runRenderLoop(() => {
        const dt = engine.getDeltaTime() / 1000.0;
        globalTime += dt;

        sim.update(dt, params);
        sim.getThinInstanceData(matricesData, offsetsData, speedsData);

        fishMesh.thinInstanceSetBuffer("matrix", matricesData, 16, false);
        fishMesh.thinInstanceSetBuffer("swimOffset", offsetsData, 1, false);
        fishMesh.thinInstanceSetBuffer("swimSpeedMultiplier", speedsData, 1, false);
        fishMesh.thinInstanceCount = params.fishCount;

        // Uniforms
        fishMaterial.setFloat("time", globalTime);
        fishMaterial.setFloat("swimAmplitude", params.swimAmplitude);
        fishMaterial.setFloat("swimFrequency", params.swimFrequency);
        fishMaterial.setFloat("shadingStyle", params.shadingStyle);
        fishMaterial.setFloat("causticsIntensity", params.causticsIntensity);
        fishMaterial.setVector3("cameraPosition", camera.position);

        scene.render();

        frameTimes++;
        if (frameTimes >= 15) {
            fpsCounter.textContent = Math.round(engine.getFps()) + " FPS";
            agentCounter.textContent = sim.count;
            frameTimes = 0;
        }
    });

    window.addEventListener('resize', () => {
        engine.resize();
    });
});
