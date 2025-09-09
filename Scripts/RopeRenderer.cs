using System.Collections.Generic;
using UnityEngine;

namespace ZombSoftBodies
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(MeshFilter))]
    public class RopeRenderer : MonoBehaviour
    {
        [SerializeField] private RopeMeshGen ropeMesh = new();
        public List<Transform> ropeBodies = new();
        private Mesh mesh = null;
        private MeshFilter meshF = null;

        private void LateUpdate()
        {
            if (meshF == null)
            {
                meshF = GetComponent<MeshFilter>();
                meshF.sharedMesh = new(); 
                mesh = meshF.sharedMesh;
            }

            ropeMesh.Tick(transform, mesh, ropeBodies);
        }

        private void OnDisable() 
        {
            ropeMesh.Dispose();
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            ropeMesh.RequestBodyConfigUpdate();
        }
#endif
    }
}

