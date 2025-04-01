#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

namespace ZombSoftBodies
{
    [CustomEditor(typeof(SoftBody))]
    public class SoftBodyEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            //Button
            SoftBody sB = (SoftBody)target;

            if (GUILayout.Button("Setup SoftBody"))
            {
                sB.Setup();
            }

            //Draw pops
            EditorGUILayout.Space();
            DrawPropertiesExcluding(serializedObject, "m_Script");

            //Apply changes
            if (serializedObject.hasModifiedProperties == true)
            {
                EditorUtility.SetDirty(target);
                if (Application.isPlaying == false) EditorSceneManager.MarkAllScenesDirty();
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
    
}
#endif
