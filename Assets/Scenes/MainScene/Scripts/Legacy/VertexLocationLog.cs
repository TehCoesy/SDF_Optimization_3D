using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VertexLocationLog : MonoBehaviour
{
    // Mesh Objects
    MeshFilter m_MeshFilter;
    Mesh m_Mesh;
    Matrix4x4 localToWorld;

    Vector3 WP(Vector3 vertex) {
        return localToWorld.MultiplyPoint3x4(vertex);
    }
    
    // Start is called before the first frame update
    void Start()
    {
        m_MeshFilter = gameObject.GetComponent<MeshFilter>();
        m_Mesh = m_MeshFilter.mesh;
    }

    // Update is called once per frame
    void Update()
    {
        localToWorld = transform.localToWorldMatrix;

        Vector3[] m_Vertices = m_Mesh.vertices;
        int[] m_Triangles = m_Mesh.triangles;

        for (int i = 0; i < m_Vertices.Length; i += 3) {
            Debug.Log("1: " + WP(m_Vertices[0]) +
            "; 2: " + WP(m_Vertices[1]) +
            "; 3: " + WP(m_Vertices[2]) +
            "; 4: " + WP(m_Vertices[3]));
        }
    }
}
