using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class SoftBodyGlobals
{
    public class SoftProps
    {

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
