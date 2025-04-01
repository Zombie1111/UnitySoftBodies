using System.Collections.Generic;
using UnityEngine;
using BoneData = SoftBodyGlobals.BoneData;
using SoftProps = SoftBodyGlobals.SoftProps;

namespace ZombSoftBodies
{
    public static class SoftBodyHelpFuncs
    {
        public static T GetOrAddComponent<T>(this Component component) where T : Component
        {
            if (component.TryGetComponent(out T comp) == true) return comp;
            return component.gameObject.AddComponent<T>();
        }

        public static Vector3[] TransformPositionsWithMatrix(Vector3[] poss, Matrix4x4 matrix)
        {
            for (int i = poss.Length - 1; i >= 0; i -= 1)
            {
                poss[i] = matrix.MultiplyPoint3x4(poss[i]);
            }

            return poss;
        }

        public static List<Vector3> TransformPositionsWithMatrix(List<Vector3> poss, Matrix4x4 matrix)
        {
            for (int i = poss.Count - 1; i >= 0; i -= 1)
            {
                poss[i] = matrix.MultiplyPoint3x4(poss[i]);
            }

            return poss;
        }
    
        public static Joint CreateJointForSoft(BoneData source, BoneData toConnect, SoftProps softProps)
        {
            ConfigurableJoint j = source.trans.gameObject.AddComponent<ConfigurableJoint>();
            j.connectedBody = toConnect.rb;
            j.xMotion = ConfigurableJointMotion.Locked;
            j.yMotion = ConfigurableJointMotion.Locked;
            j.zMotion = ConfigurableJointMotion.Locked;
            return j;
        }

        public static List<int> GetAllVertexIndexsAtPos(Vector3 pos, Vector3[] verts)
        {
            int vCount = verts.Length;
            List<int> result = new(4);

            for (int vI = 0; vI < vCount; vI++)
            {
                if ((verts[vI] - pos).sqrMagnitude > 0.0001f) continue;
                result.Add(vI);
            }

            return result;
        }

        /// <summary>
        /// Modifies to given collider mesh/size/radius based on the given positions as good as possible
        /// </summary>
        public static void SetColliderFromFromPoints(Collider col, List<Vector3> possLocal)
        {
            Transform colTrans = col.transform;
            Vector3 extents;

            if (col is MeshCollider mCol)
            {
                mCol.sharedMesh.SetVertices(possLocal, 0, possLocal.Count,
                      UnityEngine.Rendering.MeshUpdateFlags.DontValidateIndices
                | UnityEngine.Rendering.MeshUpdateFlags.DontResetBoneBounds
                | UnityEngine.Rendering.MeshUpdateFlags.DontNotifyMeshUsers
                | UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds);
                mCol.enabled = false;
                mCol.enabled = true;
                extents = col.bounds.extents;
            }
            else if (col is BoxCollider bCol)
            {
                bCol.center = GetGeometricCenterOfPositions(possLocal);
                possLocal = TransformPositionsWithMatrix(possLocal, colTrans.localToWorldMatrix);

                extents = Vector3.one * 0.001f;
                float cDis;
                Vector3 tPos = bCol.bounds.center;
                Vector3 tFor = colTrans.forward;
                Vector3 tSide = colTrans.right;
                Vector3 tUp = colTrans.up;

                foreach (Vector3 wPos in possLocal)
                {
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tFor), tPos);
                    if (cDis > extents.z) extents.z = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tSide), tPos);
                    if (cDis > extents.x) extents.x = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tUp), tPos);
                    if (cDis > extents.y) extents.y = cDis;
                }

                extents.Scale(colTrans.worldToLocalMatrix.lossyScale);
                bCol.size = extents * 2.0f;
            }
            else if (col is SphereCollider sCol)
            {
                sCol.center = GetGeometricCenterOfPositions(possLocal);
                possLocal = TransformPositionsWithMatrix(possLocal, colTrans.localToWorldMatrix);

                extents = Vector3.one * 0.001f;
                float cDis;
                Vector3 tPos = sCol.bounds.center;
                Vector3 tFor = colTrans.forward;
                Vector3 tSide = colTrans.right;
                Vector3 tUp = colTrans.up;

                foreach (Vector3 wPos in possLocal)
                {
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tFor), tPos);
                    if (cDis > extents.z) extents.z = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tSide), tPos);
                    if (cDis > extents.x) extents.x = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tUp), tPos);
                    if (cDis > extents.y) extents.y = cDis;
                }

                extents.Scale(colTrans.worldToLocalMatrix.lossyScale);
                sCol.radius = Mathf.Max(extents.x, extents.y, extents.z);
            }
            else if (col is CapsuleCollider cCol)
            {
                cCol.center = GetGeometricCenterOfPositions(possLocal);
                possLocal = TransformPositionsWithMatrix(possLocal, colTrans.localToWorldMatrix);

                extents = Vector3.one * 0.001f;
                float cDis;
                Vector3 tPos = cCol.bounds.center;
                Vector3 tFor = colTrans.forward;
                Vector3 tSide = colTrans.right;
                Vector3 tUp = colTrans.up;

                foreach (Vector3 wPos in possLocal)
                {
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tFor), tPos);
                    if (cDis > extents.z) extents.z = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tSide), tPos);
                    if (cDis > extents.x) extents.x = cDis;
                    cDis = Vector3.Distance(ClosestPointOnLineInfinit(wPos, tPos, tUp), tPos);
                    if (cDis > extents.y) extents.y = cDis;
                }

                extents.Scale(colTrans.worldToLocalMatrix.lossyScale);

                if (extents.x > extents.y && extents.x > extents.z)
                {
                    // X-axis is the longest
                    cCol.direction = 0;
                    cCol.height = extents.x * 2.0f;
                    cCol.radius = Mathf.Max(extents.y, extents.z);
                }
                else if (extents.y > extents.x && extents.y > extents.z)
                {
                    // Y-axis is the longest
                    cCol.direction = 1;
                    cCol.height = extents.y * 2.0f;
                    cCol.radius = Mathf.Max(extents.x, extents.z);
                }
                else
                {
                    // Z-axis is the longest (or all axes are equal)
                    cCol.direction = 2;
                    cCol.height = extents.z * 2.0f;
                    cCol.radius = Mathf.Max(extents.x, extents.y);
                }
            }
            else
            {
                Debug.LogError(col.GetType() + " colliders are currently not supported, please only use Mesh, Box, Sphere and Capsule colliders!");
                return;
            }

            //renable collider to fix bug??
            if (col.enabled == false) return;
            col.enabled = false;
            col.enabled = true;
        }

        /// <summary>
        /// Returns the geometric/(not average) center of given positions
        /// </summary>
        public static Vector3 GetGeometricCenterOfPositions(List<Vector3> positions)
        {
            Vector3 min = positions[0];
            Vector3 max = positions[0];

            //Find the minimum and maximum coordinates along each axis
            for (int i = 1; i < positions.Count; i++)
            {
                min = Vector3.Min(min, positions[i]);
                max = Vector3.Max(max, positions[i]);
            }

            return (min + max) * 0.5f;
        }

        /// <summary>
        /// Returns the closest position to point on the given line
        /// </summary>
        public static Vector3 ClosestPointOnLineInfinit(Vector3 point, Vector3 linePosition, Vector3 lineDirection)
        {
            return linePosition + (Vector3.Dot(point - linePosition, lineDirection) / lineDirection.sqrMagnitude) * lineDirection;
        }

#if UNITY_EDITOR
        /// <summary>
        /// Draw line between all vertics in the worldspace mesh. 
        /// </summary>
        public static void Debug_drawMesh(Mesh mesh, bool drawNormals = false, float durration = 0.1f)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            // Draw lines between triangle vertices
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];

                Debug.DrawLine(v0, v1, Color.red, durration);
                Debug.DrawLine(v1, v2, Color.green, durration);
                Debug.DrawLine(v2, v0, Color.blue, durration);
            }

            // Draw normals from each vertex
            if (drawNormals)
            {
                Vector3[] normals = mesh.normals;

                for (int i = 0; i < vertices.Length; i++)
                {
                    Vector3 vertex = vertices[i];
                    Vector3 normal = normals[i];

                    // Adjust the length of the normal line as needed
                    float normalLength = 1f;
                    Vector3 endPos = vertex + normal * normalLength;

                    Debug.DrawLine(vertex, endPos, Color.yellow, durration);
                }
            }
        }
#endif
    }
}
