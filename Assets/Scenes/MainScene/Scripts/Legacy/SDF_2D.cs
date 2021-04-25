using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class SDF_2D : MonoBehaviour
{
    bool m_Started = false;

    // Mesh Objects
    MeshFilter m_MeshFilter;
    Mesh m_Mesh;
    Matrix4x4 localToWorld;

    // SDF Variables
    float u, v = 0;
    float c, d = 0;
    Vector3 x;
    
    [SerializeField]
    GameObject Target;

    [SerializeField]
    bool useFrankWolfe = false, useGradientDescent = false, useGoldenSection = true;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize m_Mesh and fetch first Vertices, Triangles
        m_Started = true;
        m_MeshFilter = gameObject.GetComponent<MeshFilter>();
        m_Mesh = m_MeshFilter.mesh;

        Vector3[] m_Vertices = m_Mesh.vertices;
        int[] m_Triangles = m_Mesh.triangles;

        Debug.Log("Vertices: " + m_Vertices.Length + "; Indices: " + m_Triangles.Length);

        // Initial Guess
        u = 0.5f;
        v = 0.5f;
    }

    Vector3 WP(Vector3 vertex) {
        return localToWorld.MultiplyPoint3x4(vertex);
    }

    float SDFCircle(Vector3 a) {
        return Math.Abs(GetLength(a, Vector3.zero) - 1.0f);
    }

    float SDFCube(Vector3 a, Vector3 p, float length) {
        return 0.0f;
    }

    float GetLength(Vector3 a, Vector3 b) {
        return (float) Math.Sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.y - b.y)*(a.y - b.y));
    }

    float GetField(Vector3 a) {
        return SDFCircle(a);
    }

    Vector3 GetFieldGradient(Vector3 a) {
        Vector3 gradient = Vector3.zero;
        float eps = 0.01f;

        gradient.x = GetField(a + Vector3.right * eps) - GetField(a + Vector3.left * eps);
        gradient.y = GetField(a + Vector3.up * eps) - GetField(a + Vector3.down * eps);
        gradient.z = GetField(a + Vector3.forward * eps) - GetField(a + Vector3.back * eps);
        gradient /= 2.0f * eps;

        return gradient;
    }

    // Update is called once per frame
    void Update()
    {
        localToWorld = transform.localToWorldMatrix;

        Vector3[] m_Vertices = m_Mesh.vertices;
        int[] m_Triangles = m_Mesh.triangles;

        /*for (int i = 0; i < m_Vertices.Length; i += 3) {
            Debug.Log("1: " + WP(m_Vertices[0]) +
            "; 2: " + WP(m_Vertices[1]) +
            "; 3: " + WP(m_Vertices[2]) +
            "; 4: " + WP(m_Vertices[3]));
        }*/

        float gr = 0;

        Vector3 a = WP(m_Vertices[1]);
        Vector3 b = WP(m_Vertices[2]);

        //Golden Section search
        if (useGoldenSection) {
            gr = (float) ((Math.Sqrt(5.0) + 1.0) / 2.0);
            u = 0.0f;
            v = 1.0f;

            c = v - (v - u) / gr;
            d = u + (v - u) / gr;
        }

        // Search iterations
        for (int i = 0; i < 32; i++) {
            if (useGoldenSection) {
                float mid = (v + u) / 2.0f;

                x = mid * a + (1.0f - mid) * b;

                if (SDFCircle(a * c + b * (1.0f - c)) < SDFCircle(a * d + b * (1.0f - d))) {
                    v = d;
                } else {
                    u = c;
                }

                c = v - (v - u) / gr;
                d = u + (v - u) / gr;
            } else if (useFrankWolfe) {
                Vector3 gradient = GetFieldGradient(x);
                float da = Vector3.Dot(a, gradient);
                float db = Vector3.Dot(b, gradient);
                
                Vector3 s;

                if (da < db)
                    s = a;
                else
                    s = b;
                
                float gamma = 0.3f *2.0f /((float) i+2.0f);
                
                x = (1.0f - gamma) * x + gamma * s;
            } else if (useGradientDescent) {
                // projected gradient descent
	            Vector3 gradient = GetFieldGradient(x);               
                float du = Vector3.Dot(gradient, a - b);
            
                const float alpha = 0.05f;

                u -= alpha * du;

                // projection
                u = Math.Max(u, 0.0f);
                u = Math.Min(u, 1.0f);
                v = 1.0f - u;

                x = u * a + v * b;
            }
        }
    }

    private void OnDrawGizmos() {
        if (m_Started) {
            localToWorld = transform.localToWorldMatrix;

            m_MeshFilter = gameObject.GetComponent<MeshFilter>();
            m_Mesh = m_MeshFilter.mesh;

            Vector3[] m_Vertices = m_Mesh.vertices;
            int[] m_Triangles = m_Mesh.triangles;

            //
            Vector3 position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[1];
            position += transform.position;
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(position, Vector3.one);
            //
            position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[2];
            position += transform.position;
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(position, Vector3.one);
            //
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(x, 0.1f);
        }
    }
}