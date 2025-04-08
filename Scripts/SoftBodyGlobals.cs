using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class SoftBodyGlobals
{
    [System.Serializable]
    public class SoftProps
    {
        public float softness = 0.3f;
        public float spring = 500.0f;
        public float damping = 30.0f;
        public float bounciness = 0.0f;
    }

    public class BoneData
    {
        public Transform trans;
        public Rigidbody rb;

        public Vector3 wCenter;
        public List<Vector3> wVerts;
        public HashSet<int> connectedBoneIs;
    }
}
