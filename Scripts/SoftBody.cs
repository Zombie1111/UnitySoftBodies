using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using BoneData = SoftBodyGlobals.BoneData;
using SoftProps = SoftBodyGlobals.SoftProps;

namespace ZombSoftBodies
{
    public class SoftBody : MonoBehaviour
    {
        [SerializeField] private bool setColliders = true;
        [SerializeField] private bool twoWayConnections = false;
        [SerializeField] SoftProps properties = new();

        public void Setup()
        {
            //Verify
            if (TryGetComponent(out SkinnedMeshRenderer sMR) == false)
            {
                Debug.LogError("No SkinnedMeshRenderer attatch to " + transform.name);
                return;
            }

            Transform[] bones = sMR.bones;
            if (bones.Length == 0)
            {
                Debug.LogError(transform.name + " has no bones");
                return;
            }

            //Get mesh data
            Mesh mesh = sMR.sharedMesh;
            Mesh bMesh = new();
            sMR.BakeMesh(bMesh, true);
            Vector3[] wVers = SoftBodyHelpFuncs.TransformPositionsWithMatrix(bMesh.vertices, sMR.localToWorldMatrix);
            byte[] bPerV = mesh.GetBonesPerVertex().ToArray();//Reading native is slower, cost of ToArray is hopefully lower than native overhead
            BoneWeight1[] bws = mesh.GetAllBoneWeights().ToArray();//And dont wanna bother with burst for now atleast

            //Get bone data, what vertics each bone uses
            BoneData[] bds = new BoneData[bones.Length];

            for (int i = 0; i < bones.Length; i++)
            {
                bds[i] = new BoneData()
                {
                    trans = bones[i],
                    rb = bones[i].GetOrAddComponent<Rigidbody>(),
                    wVerts = new(4),
                    connectedBoneIs = new(8),
                };
            }

            int vertexCount = mesh.vertexCount;
            int bwI = 0;
            Dictionary<int, int[]> vIToBI = new(vertexCount);

            for (int vI = 0; vI < vertexCount; vI++)
            {
                byte boneCount = bPerV[vI];
                int[] bIs = new int[boneCount];

                for (byte i = 0; i < boneCount; i++)
                {
                    int bI = bws[bwI].boneIndex;
                    BoneData bd = bds[bI];

                    bIs[i] = bI;
                    bd.wVerts.Add(wVers[vI]);
                    bwI++;
                }

                vIToBI.Add(vI, bIs);
            }

            //Get center + destroy all old joints
            for (int i = 0; i < bones.Length; i++)
            {
                BoneData bd = bds[i];
                bd.wCenter = SoftBodyHelpFuncs.GetGeometricCenterOfPositions(bd.wVerts);
                foreach (Joint j in bd.trans.GetComponents<Joint>())
                {
#if UNITY_EDITOR
                    if (Application.isPlaying == false) DestroyImmediate(j);
                    else
#endif
                        Destroy(j);
                }
            }

            //Setup bones, set colliders and joints
            HashSet<int> bIToConnects = new(4);
            object lockO = new();

            for (int i = 0; i < bones.Length; i++)
            {
                BoneData bd = bds[i];

                //Set collider
                if (setColliders == true && bd.trans.TryGetComponent(out Collider col) == true)
                {
                    SoftBodyHelpFuncs.SetColliderFromFromPoints(col,
                        SoftBodyHelpFuncs.TransformPositionsWithMatrix(bd.wVerts, bd.trans.worldToLocalMatrix));
                }

                //Set joint
                //Get what to connect to
                bIToConnects.Clear();

                //foreach (Vector3 wV in bd.wVerts)
                Parallel.ForEach(bd.wVerts, wV =>
                {
                    List<int> vIAtPos = SoftBodyHelpFuncs.GetAllVertexIndexsAtPos(wV, wVers);

                    foreach (int vI in vIAtPos)
                    {
                        foreach (int bI in vIToBI[vI])
                        {
                            if (bI == i) continue;

                            lock (lockO)
                            {
                                bIToConnects.Add(bI);
                            }
                        }
                    }
                });

                //Connect joints
                foreach (int bIToC in bIToConnects)
                {
                    BoneData cbd = bds[bIToC];
                    if (twoWayConnections == false
                        && cbd.connectedBoneIs.Contains(i) == true) continue;

                    bd.connectedBoneIs.Add(bIToC);
                    Joint joint = SoftBodyHelpFuncs.CreateJointForSoft(bd, cbd, properties);
                }
            }

#if UNITY_EDITOR
            if (Application.isPlaying == false)
                Debug.Log("Done generating SoftBody for " + transform.name);
#endif
        }
    }
}
