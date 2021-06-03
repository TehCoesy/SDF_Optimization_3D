using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ClothSimulatorModified))]
public class ClothSimulatorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        ClothSimulatorModified script = (ClothSimulatorModified) target;
        // Default Inspector
        base.DrawDefaultInspector();
        // Custom Inspector
        GUILayout.Label("Simulation Controls", EditorStyles.boldLabel);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Play")) {
            script.playSimulation();
        }
        if (GUILayout.Button("Pause")) {
            script.pauseSimulation();
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Reset")) {
            script.resetSimulation();
        }
        GUILayout.EndHorizontal();
    }
}
