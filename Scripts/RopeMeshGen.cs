using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Rendering;

namespace ZombSoftBodies
{
    [System.Serializable]
    public class RopeMeshGen
    {
        private const float _highNumber = 69420.0f;
        private const MeshUpdateFlags _defaultUpdateFlags =
              MeshUpdateFlags.DontRecalculateBounds
            | MeshUpdateFlags.DontValidateIndices
            | MeshUpdateFlags.DontResetBoneBounds;

        [SerializeField] private RopeData ropeConfig = new(true);
        private bool jobIsActive = false;
        private JobHandle jobHandle;
        private ComputeRopeMesh crm_job;
        private GetRopePositions grp_job;
        private Mesh prevMesh = null;
        private TransformAccessArray bodiesNative;

        private bool needsBodyConfigUpdate = true;

        public void RequestBodyConfigUpdate()
        {
            needsBodyConfigUpdate = true;
        }

        int prevBodyCount = 0;

        public void Tick(Transform meshTrans, Mesh mesh, List<Transform> bodies, float lerpAmount = 0.4f)
        {
            EndCompute(mesh);
            if (mesh == null || bodies == null || bodies.Count < 2 || meshTrans == null)
            {
                mesh.Clear(true);
                return;
            }

            bool created = bodiesNative.isCreated;
            int newBodyCount = bodies.Count;
            int bodyCount = needsBodyConfigUpdate == true || created == false
                ? newBodyCount : prevBodyCount;
            prevBodyCount = bodyCount;

            //Ensure native containers are valid
            if (EnsureLenght(ref grp_job.ropeBodies, bodyCount) == true)
                crm_job.ropeBodies = grp_job.ropeBodies.AsReadOnly();

            if (needsBodyConfigUpdate == true || created == false)
            {
                Transform last = meshTrans;
                if (created == false) bodiesNative = new(bodies.Count);
                int nativeBodyCount = bodiesNative.length;

                for (int i = 0; i < newBodyCount; i++)
                {
                    if (bodies[i] != null) last = bodies[i];

                    if (i < nativeBodyCount)
                    {
                        if (bodiesNative[i] == last) continue;
                        bodiesNative[i] = last;
                        SetBody(i);
                        continue;
                    }

                    bodiesNative.Add(last);
                    SetBody(i);
                }

                for (int i = nativeBodyCount - 1; i >= newBodyCount; i--)
                {
                    bodiesNative.RemoveAtSwapBack(i);
                }

                void SetBody(int i)
                {
                    last.GetPositionAndRotation(out Vector3 pos, out Quaternion rot);
                    grp_job.ropeBodies[i] = new()
                    {
                        rot = rot,
                        pos = pos,
                        prevPos = pos,
                    };
                }
            }

            int pCount = (int)Mathf.Pow(2, ropeConfig.smoothning);
            int n0 = bodyCount - 1;
            int maxSmoothChunkCount = (n0 - 1) * pCount + 2;
            int maxRawChunkCount = bodyCount;
            EnsureLenght(ref crm_job.inputChunks, maxRawChunkCount);
            EnsureLenght(ref crm_job.outputChunks, maxSmoothChunkCount);
            EnsureLenght(ref crm_job.decimateMask, maxRawChunkCount);
            EnsureCapacity(ref crm_job.rangeToCompute, maxRawChunkCount);
            if (crm_job.ropeData.IsCreated == false) crm_job.ropeData = new(ropeConfig, Allocator.Persistent);
            else if (needsBodyConfigUpdate == true) crm_job.ropeData.Value = ropeConfig;

            needsBodyConfigUpdate = false;

            //Start job
            prevMesh = mesh;
            grp_job.lerpAmount = lerpAmount;
            crm_job.deltaTime = Time.deltaTime;
            crm_job.wToL = meshTrans.worldToLocalMatrix;
            crm_job.meshDataArray = Mesh.AllocateWritableMeshData(1);
            jobHandle = grp_job.Schedule(bodiesNative);
            jobHandle = crm_job.Schedule(jobHandle);
            jobIsActive = true;
        }

        private float ropeLenght = 0.0f;
        public float _ropeLenght => ropeLenght;
        private float ropeSpeedRatio = 0.0f;
        public float _ropeSpeedRatio => ropeSpeedRatio;
        private Vector3 ropeAvgPos = Vector3.zero;
        public Vector3 _ropeAvgPos => ropeAvgPos;

        private void EndCompute(Mesh mesh)
        {
            if (jobIsActive == false) return;
            jobIsActive = false;

            jobHandle.Complete();
            if (prevMesh != mesh || mesh == null || crm_job.meshDataArray.Length != 1)
                crm_job.meshDataArray.Dispose();
            else
            {
                Mesh.ApplyAndDisposeWritableMeshData(crm_job.meshDataArray, mesh, _defaultUpdateFlags);
                RopeData rd = crm_job.ropeData.Value;
                mesh.bounds = rd.bounds;
                ropeLenght = rd.ropeLenght;
                ropeSpeedRatio = rd.speedRatio;
                ropeAvgPos = rd.avgBodyPos;
            }
        }

        public void Dispose()
        {
            EndCompute(null);//Make sure job aint running

            if (bodiesNative.isCreated == true) bodiesNative.Dispose();
            if (grp_job.ropeBodies.IsCreated == true) grp_job.ropeBodies.Dispose();

            if (crm_job.ropeData.IsCreated == true) crm_job.ropeData.Dispose();
            if (crm_job.rangeToCompute.IsCreated == true) crm_job.rangeToCompute.Dispose();
            if (crm_job.decimateMask.IsCreated == true) crm_job.decimateMask.Dispose();
            if (crm_job.outputChunks.IsCreated == true) crm_job.outputChunks.Dispose();
            if (crm_job.inputChunks.IsCreated == true) crm_job.inputChunks.Dispose();
        }

        /// <summary>
        /// Returns true if resized
        /// </summary>
        private static bool EnsureLenght<T>(ref NativeArray<T> container, int lenght, Allocator allocator = Allocator.Persistent) where T : struct
        {
            if (container.IsCreated == false)
            {
                container = new NativeArray<T>(lenght, allocator);
                return true;
            }

            if (container.Length == lenght) return false;
            NativeArray<T> newContainer = new(lenght, allocator);

            if (container.Length > lenght)
                newContainer.CopyFrom(container.GetSubArray(0, lenght));
            else
                container.CopyTo(newContainer.GetSubArray(0, container.Length));

            container.Dispose();
            container = newContainer;
            return true;
        }

        /// <summary>
        /// Returns true if resized, does not keep old items if resized
        /// </summary>
        private static bool EnsureCapacity<T>(ref NativeList<T> container, int capacity, Allocator allocator = Allocator.Persistent) where T : unmanaged
        {
            if (container.IsCreated == false)
            {
                container = new NativeList<T>(capacity, allocator);
                return true;
            }

            if (container.Capacity >= capacity) return false;
            NativeList<T> newContainer = new(capacity, allocator);

            container.Dispose();
            container = newContainer;
            return true;
        }

        /// <summary>
        /// Returns true if resized, does not keep bits of resized
        /// </summary>
        private static bool EnsureLenght(ref NativeBitArray container, int lenght, Allocator allocator = Allocator.Persistent)
        {
            if (container.IsCreated == false)
            {
                container = new NativeBitArray(lenght, allocator);
                return true;
            }

            if (container.Length == lenght) return false;
            NativeBitArray newContainer = new(lenght, allocator);

            container.Dispose();
            container = newContainer;
            return true;
        }

        private struct RopeBody
        {
            public Vector3 prevPos;
            public Vector3 pos;
            public Quaternion rot;
        }

        [BurstCompile]
        private struct GetRopePositions : IJobParallelForTransform
        {
            public NativeArray<RopeBody> ropeBodies;
            public float lerpAmount;

            public void Execute(int index, TransformAccess transform)
            {
                RopeBody rb = ropeBodies[index];
                rb.rot = transform.rotation;
                rb.prevPos = rb.pos;
                rb.pos = Vector3.Lerp(rb.pos, transform.position, lerpAmount);
                ropeBodies[index] = rb;
            }
        }

        [System.Serializable]
        private struct RopeData
        {
            public RopeData(bool unused)
            {
                decimation = 0.0f;
                smoothning = 1;
                normalizeV = true;
                uvScale = new Vector2(1.0f, 5.0f);
                thickness = 0.22f;
                resolution = 4;

                bounds = new(Vector3.zero, Vector3.one * _highNumber);
                ropeLenght = 0.0f;
                speedRatio = 0.0f;
                avgBodyPos = Vector3.zero;
            }

            public float decimation;
            public uint smoothning;
            public Vector2 uvScale;
            public bool normalizeV;
            public float thickness;
            public int resolution;

            internal Bounds bounds;
            internal float ropeLenght;
            internal float speedRatio;
            internal Vector3 avgBodyPos;
        }

        [BurstCompile]
        private struct ComputeRopeMesh : IJob
        {
            public Mesh.MeshDataArray meshDataArray;
            public NativeArray<RopeBody>.ReadOnly ropeBodies;
            public NativeReference<RopeData> ropeData;
            public Matrix4x4 wToL;
            public float deltaTime;

            public NativeArray<RopeChunk> inputChunks;
            public NativeArray<RopeChunk> outputChunks;
            public NativeBitArray decimateMask;
            public NativeList<Vector2Int> rangeToCompute;

            public void Execute()
            {
                var _meshData = meshDataArray[0];
                var _wTol = wToL;
                var _deltaTime = deltaTime;
                var _ropeBodies = ropeBodies;
                var _inputChunks = inputChunks;
                var _outputChunks = outputChunks;
                var _rangeToCompute = rangeToCompute;
                var _decimateMask = decimateMask;
                Quaternion wToL_rot = wToL.rotation;
                int bodyCount = _ropeBodies.Length;
                int bodyCountM1 = bodyCount - 1;
                int inputCount = _inputChunks.Length;
                int outputCount;
                RopeData data = ropeData.Value;

                ComputeSpeed();
                PrepareChunks();
                Decimate(data.decimation);
                Smooth(data.smoothning);

                float ropeLenght = GetRopeLength(outputChunks);
                CreateMesh(data.uvScale, data.normalizeV, data.thickness, data.resolution);
                data.ropeLenght = ropeLenght;
                ropeData.Value = data;

                void ComputeSpeed()
                {
                    Vector3 totPos = Vector3.zero;
                    float minSpeed = float.MaxValue;
                    float maxSpeed = float.MinValue;

                    for (int i = 0; i < bodyCount; i++)
                    {
                        RopeBody rb = _ropeBodies[i];
                        float speedSqr = (rb.pos - rb.prevPos).sqrMagnitude;
                        minSpeed = Mathf.Min(minSpeed, speedSqr);
                        maxSpeed = Mathf.Max(maxSpeed, speedSqr);
                        totPos += rb.pos;
                    }

                    data.speedRatio = (Mathf.Sqrt(maxSpeed) - Mathf.Sqrt(minSpeed)) / _deltaTime;
                    data.avgBodyPos = totPos / bodyCount;
                }

                void PrepareChunks()
                {
                    RopeChunk chunk0 = new();
                    RopeChunk chunk1 = new();
                    RopeChunk chunk2;
                    ChunkFromBody(ref chunk1, 0, false);

                    for (int m = 1; m <= bodyCount; m++)
                    {
                        int index;
                        if (m >= bodyCountM1)
                            index = bodyCountM1;
                        else
                            index = m;

                        ChunkFromBody(ref chunk0, index, true);
                        chunk2 = chunk1;
                        chunk1 = chunk0;
                        _inputChunks[m - 1] = chunk2;
                    }

                    void ChunkFromBody(ref RopeChunk chunk, int chunkI, bool lerpRotation)
                    {
                        RopeBody rb = _ropeBodies[chunkI];
                        chunk.position = _wTol.MultiplyPoint3x4(rb.pos);
                        Quaternion current;
                        Quaternion previous;

                        if (chunkI == bodyCountM1)
                        {
                            previous = _ropeBodies[^2].rot;
                            current = previous;
                        }
                        else if (chunkI == 0)
                        {
                            previous = _ropeBodies[1].rot;
                            current = previous;
                        }
                        else
                        {
                            current = rb.rot;
                            previous = _ropeBodies[Mathf.Max(0, chunkI - 1)].rot;
                        }

                        Quaternion avg = wToL_rot * (lerpRotation ? Quaternion.SlerpUnclamped(current, previous, 0.001f) : current);
                        chunk.normal = avg * Vector3.up;
                        chunk.binormal = avg * Vector3.right;
                        chunk.tangent = avg * Vector3.forward;
                    }
                }

                float GetRopeLength(NativeArray<RopeChunk> chunk)
                {
                    float length = 0.0f;
                    for (int i = 1; i < chunk.Length; i++)
                    {
                        length += Vector3.Distance(chunk[i].position, chunk[i - 1].position);
                    }

                    return length;
                }

                void Decimate(float threshold)
                {
                    if (threshold < 0.00001f || _inputChunks.Length < 3) return;

                    threshold = threshold * threshold * 0.01f;
                    _rangeToCompute.Clear();
                    _rangeToCompute.Add(new Vector2Int(0, _inputChunks.Length - 1));
                    _decimateMask.SetBits(0, true, _decimateMask.Length);

                    while (_rangeToCompute.Length > 0)
                    {
                        Vector2Int range = _rangeToCompute[^1];
                        _rangeToCompute.RemoveAt(_rangeToCompute.Length - 1);

                        float dmax = 0;
                        int startI = range.x;

                        for (int i = startI + 1; i < range.y; i++)
                        {
                            if (_decimateMask.GetBits(i) != 0u)
                            {
                                float d = Vector3.SqrMagnitude(ClosestPointOnLine(_inputChunks[i].position, _inputChunks[range.x].position,
                                    _inputChunks[range.y].position) - _inputChunks[i].position);

                                if (d > dmax)
                                {
                                    startI = i;
                                    dmax = d;
                                }
                            }
                        }

                        if (dmax > threshold)
                        {
                            _rangeToCompute.Add(new Vector2Int(range.x, startI));
                            _rangeToCompute.Add(new Vector2Int(startI, range.y));
                        }
                        else
                            _decimateMask.SetBits(range.x + 1, false, (range.x + 1) - range.y);
                    }

                    //Write back result
                    int newRawCount = 0;
                    for (int i = 0; i < _inputChunks.Length; i++)
                    {
                        if (_decimateMask.GetBits(i) == 0u) continue;
                        if (newRawCount == i) newRawCount++;
                        else _inputChunks[newRawCount++] = _inputChunks[i];
                    }

                    inputCount = newRawCount;
                }

                void Smooth(uint levels)
                {
                    if (levels == 0 || inputCount < 3)
                    {
                        _inputChunks.CopyTo(_outputChunks.GetSubArray(0, inputCount));
                        outputCount = inputCount;
                        return;
                    }

                    int outCountPow2Lvl = (int)Mathf.Pow(2, levels);
                    int inCountM1 = inputCount - 1;
                    float pow2NegLvlP1 = Mathf.Pow(2, -(levels + 1));
                    float pow2NegLvl = Mathf.Pow(2, -levels);
                    float pow2Neg2Lvl = Mathf.Pow(2, -2 * levels);
                    float pow2Neg2LvlM1 = Mathf.Pow(2, -2 * levels - 1);

                    outputCount = (inCountM1 - 1) * outCountPow2Lvl + 2;
                    _outputChunks[0] = (0.5f + pow2NegLvlP1) * _inputChunks[0] + (0.5f - pow2NegLvlP1) * _inputChunks[1];
                    _outputChunks[outCountPow2Lvl * inCountM1 - outCountPow2Lvl + 1]
                        = (0.5f - pow2NegLvlP1) * _inputChunks[inCountM1 - 1] + (0.5f + pow2NegLvlP1) * _inputChunks[inCountM1];

                    for (int j = 1; j <= outCountPow2Lvl; ++j)
                    {
                        float F = 0.5f - pow2NegLvlP1 - (j - 1) * (pow2NegLvl - j * pow2Neg2LvlM1);
                        float G = 0.5f + pow2NegLvlP1 + (j - 1) * (pow2NegLvl - j * pow2Neg2Lvl);
                        float H = (j - 1) * j * pow2Neg2LvlM1;

                        for (int i = 1; i < inCountM1; ++i)
                        {
                            _outputChunks[(i - 1) * outCountPow2Lvl + j] = RopeChunk.WeightedSum(F, G, H,
                                _inputChunks[i - 1], _inputChunks[i], _inputChunks[i + 1]);
                        }
                    }

                    _outputChunks[0] = _inputChunks[0];
                    _outputChunks[outputCount - 1] = _inputChunks[inputCount - 1];
                }

                void CreateMesh(Vector2 uvScale, bool normalizeV, float thickness, int resolution)
                {
                    NativeArray<Vector2> ropeVers = GetCircleOffsets(resolution);
                    int sectionSegments = ropeVers.Length - 1;
                    int verticesPerSection = sectionSegments + 1;
                    int verCount = outputCount * verticesPerSection;
                    int indCount = (outputCount - 1) * sectionSegments * 6;

                    NativeArray<VertexAttributeDescriptor> layout = new(4, Allocator.Temp);
                    layout[0] = new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, 0);
                    layout[1] = new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3, 1);
                    layout[2] = new VertexAttributeDescriptor(VertexAttribute.Tangent, VertexAttributeFormat.Float32, 4, 2);
                    layout[3] = new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2, 3);

                    _meshData.SetVertexBufferParams(verCount, layout);
                    _meshData.SetIndexBufferParams(indCount, IndexFormat.UInt32);
                    _meshData.subMeshCount = 1;

                    var vertices = _meshData.GetVertexData<Vector3>(0);
                    var normals = _meshData.GetVertexData<Vector3>(1);
                    var tangents = _meshData.GetVertexData<Vector4>(2);
                    var uvs = _meshData.GetVertexData<Vector2>(3);
                    var indices = _meshData.GetIndexData<uint>();

                    float vCoord = -uvScale.y;
                    float actualToRestLengthRatio = 1f;
                    int sectionIndex = 0;
                    int verI = 0;
                    int indI = 0;

                    Vector3 bMin = Vector3.one * _highNumber;
                    Vector3 bMax = Vector3.one * -_highNumber;

                    for (int i = 0; i < outputCount; i++)
                    {
                        int prevIndex = Mathf.Max(i - 1, 0);
                        vCoord += uvScale.y * (Vector3.Distance(_outputChunks[i].position, _outputChunks[prevIndex].position) /
                                               (normalizeV ? ropeLenght : actualToRestLengthRatio));

                        int nextSectionIndex = sectionIndex + 1;
                        for (int j = 0; j <= sectionSegments; j++)
                        {
                            Vector2 sectionVertex = ropeVers[j];
                            Vector3 normal;
                            normal.x = (sectionVertex.x * _outputChunks[i].normal.x + sectionVertex.y * _outputChunks[i].binormal.x) * thickness;
                            normal.y = (sectionVertex.x * _outputChunks[i].normal.y + sectionVertex.y * _outputChunks[i].binormal.y) * thickness;
                            normal.z = (sectionVertex.x * _outputChunks[i].normal.z + sectionVertex.y * _outputChunks[i].binormal.z) * thickness;

                            Vector3 vertex;
                            vertex.x = _outputChunks[i].position.x + normal.x;
                            vertex.y = _outputChunks[i].position.y + normal.y;
                            vertex.z = _outputChunks[i].position.z + normal.z;
                            bMax = Vector3.Max(bMax, vertex);
                            bMin = Vector3.Min(bMin, vertex);

                            Vector4 tangent;
                            tangent.x = normal.y * _outputChunks[i].tangent.z - normal.z * _outputChunks[i].tangent.y;
                            tangent.y = normal.z * _outputChunks[i].tangent.x - normal.x * _outputChunks[i].tangent.z;
                            tangent.z = normal.x * _outputChunks[i].tangent.y - normal.y * _outputChunks[i].tangent.x;
                            tangent.w = -1;

                            Vector2 uv;
                            uv.x = (j / (float)sectionSegments) * uvScale.x;
                            uv.y = vCoord;

                            vertices[verI] = vertex;
                            normals[verI] = normal;
                            tangents[verI] = tangent;
                            uvs[verI++] = uv;

                            if (j < sectionSegments && i < outputCount - 1)
                            {
                                indices[indI++] = (uint)(sectionIndex * verticesPerSection + j);
                                indices[indI++] = (uint)(nextSectionIndex * verticesPerSection + j);
                                indices[indI++] = (uint)(sectionIndex * verticesPerSection + (j + 1));

                                indices[indI++] = (uint)(sectionIndex * verticesPerSection + (j + 1));
                                indices[indI++] = (uint)(nextSectionIndex * verticesPerSection + j);
                                indices[indI++] = (uint)(nextSectionIndex * verticesPerSection + (j + 1));
                            }
                        }

                        sectionIndex++;
                    }

                    var sub = new SubMeshDescriptor(indexStart: 0, indexCount: indI, MeshTopology.Triangles);
                    _meshData.SetSubMesh(0, sub, _defaultUpdateFlags);
                    data.bounds.SetMinMax(bMin, bMax);
                    ropeVers.Dispose();
                }
            }

            private static NativeArray<Vector2> GetCircleOffsets(int count, Allocator allocator = Allocator.Temp)
            {
                NativeArray<Vector2> ropeVers = new(count + 1, allocator);
                for (int j = 0; j <= count; j++)
                {
                    float angle = 2 * Mathf.PI / count * j;
                    ropeVers[j] = Mathf.Cos(angle) * Vector2.right + Mathf.Sin(angle) * Vector2.up;
                }

                return ropeVers;
            }

            private static Vector3 ClosestPointOnLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
            {
                Vector3 ap = point - lineStart;
                Vector3 ab = lineEnd - lineStart;
                float mu = Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab);
                mu = Mathf.Clamp01(mu);
                return lineStart + ab * mu;
            }
        }

        private struct RopeChunk
        {
            public Vector3 position;
            public Vector3 tangent;
            public Vector3 normal;
            public Vector3 binormal;

            public RopeChunk(Vector3 pos, Vector3 nor, Vector3 binor, Vector3 tan)
            {
                this.position = pos;
                this.normal = nor;
                this.tangent = tan;
                this.binormal = binor;
            }

            public static RopeChunk operator +(RopeChunk c1, RopeChunk c2)
            {
                return new RopeChunk(c1.position + c2.position, c1.normal + c2.normal, c1.binormal + c2.binormal, c1.tangent + c2.tangent);
            }

            public static RopeChunk operator *(float f, RopeChunk c)
            {
                return new RopeChunk(c.position * f, c.normal * f, c.binormal * f, c.tangent * f);
            }

            public static RopeChunk WeightedSum(float w1, float w2, float w3, RopeChunk c1, RopeChunk c2, RopeChunk c3)
            {
                return new()
                {
                    position = new(c1.position.x * w1 + c2.position.x * w2 + c3.position.x * w3,
                        c1.position.y * w1 + c2.position.y * w2 + c3.position.y * w3,
                        c1.position.z * w1 + c2.position.z * w2 + c3.position.z * w3),

                    normal = new(c1.normal.x * w1 + c2.normal.x * w2 + c3.normal.x * w3,
                        c1.normal.y * w1 + c2.normal.y * w2 + c3.normal.y * w3,
                        c1.normal.z * w1 + c2.normal.z * w2 + c3.normal.z * w3),

                    tangent = new(c1.tangent.x * w1 + c2.tangent.x * w2 + c3.tangent.x * w3,
                        c1.tangent.y * w1 + c2.tangent.y * w2 + c3.tangent.y * w3,
                        c1.tangent.z * w1 + c2.tangent.z * w2 + c3.tangent.z * w3),

                    binormal = new(c1.binormal.x * w1 + c2.binormal.x * w2 + c3.binormal.x * w3,
                        c1.binormal.y * w1 + c2.binormal.y * w2 + c3.binormal.y * w3,
                        c1.binormal.z * w1 + c2.binormal.z * w2 + c3.binormal.z * w3)
                };
            }
        }
    }
}
