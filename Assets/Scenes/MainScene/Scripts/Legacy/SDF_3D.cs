using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class SDF_3D : MonoBehaviour
{
    // Controls
    bool m_Started = false;

    // gameObject Variables
    MeshFilter m_MeshFilter;
    Mesh m_Mesh;
    Matrix4x4 localToWorld;

    // SDF Variables
    Vector3 x;

    // Disable showing result if target is too far away
    float boundDistance = 0;
    float threshold = 0.2f;
    
    // Target Variables
    [SerializeField]
    public GameObject target;

    private Vector3 targetBounds;

    // Debug tools
    [SerializeField]
    bool debugVertices = false;

    void Start()
    {
        // Establish gameObject's mesh
        m_Started = true;
        m_MeshFilter = gameObject.GetComponent<MeshFilter>();
        m_Mesh = m_MeshFilter.mesh;

        // Establish target's mesh
        if (target) {
            targetBounds = target.GetComponent<Renderer>().bounds.size;
        }

        Debug.Log("X: " + targetBounds.x + " Y: "
        + targetBounds.y + " Z: " + targetBounds.z);
    }

    // Transform position of a vertex to world space
    Vector3 WP(Vector3 vertex) {
        return localToWorld.MultiplyPoint3x4(vertex);
    }

    // Distance to a sphere's surface
    float SDFCircle(Vector3 a, bool neg) {
        // [TODO]: No handling for different round shapes yet, assuming bounds.x = bounds.y = bounds.z
        float da = GetLength(a, target.transform.position) - targetBounds.x;

        if (da < 0.0f && neg) return Math.Abs(da);
        else return da;
    }

    // Distance to a box's surface
    // Can be used for a bounding box or a cube mesh
    float SDFBox(Vector3 a, bool neg) {
        Vector3 position = target.transform.position;
        
        float left = position.x - targetBounds.x / 2, right = position.x + targetBounds.x / 2;
        float bottom = position.y - targetBounds.y / 2, top = position.y + targetBounds.y / 2;
        float back = position.z - targetBounds.z / 2, forward = position.z + targetBounds.z / 2;

        float dx = DistanceAux(a.x, left, right);
        float dy = DistanceAux(a.y, bottom, top);
        float dz = DistanceAux(a.z, back, forward);

        if (a.x > left && a.x < right && a.y > bottom && a.y < top && a.z > back && a.z < forward) {
            if (dx < dy && dx < dz) {
                if (neg) return -dx;
                return dx;
            }
            else if (dy < dx && dy < dz) {
                if (neg) return -dy;
                return dy;
            }
            else {
                if (neg) return -dz;
                return dz;
            }
        }

        return (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    float DistanceAux(float x, float min, float max) {
        if (x < min) return min - x;
        else if (x > max) return x - max;

        return Math.Min(x - min, max - x);
    }

    float SDFLine(Vector3 x, Vector3 a, Vector3 b) {
        return 0.0f;
    }

    // Distance between 2 points
    float GetLength(Vector3 a, Vector3 b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private float GetField(Vector3 a) {
        if (target) {
            if (target.tag == "Sphere") {
                return SDFCircle(a, true);
            } else if (target.tag == "Cube") {
                return SDFBox(a, true);
            }
        }
        return 0.0f;
    }

    private Vector3 GetFieldGradient(Vector3 a) {
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
        if (target)
        localToWorld = transform.localToWorldMatrix;

        Vector3[] m_Vertices = m_Mesh.vertices;
        int[] m_Triangles = m_Mesh.triangles;

        Vector3 a = WP(m_Vertices[0]);
        Vector3 b = WP(m_Vertices[1]);
        Vector3 c = WP(m_Vertices[2]);
        Vector3 d = WP(m_Vertices[3]);

        // Search iterations
        for (int i = 0; i < 32; i++) {
            // Frank - wolfe
            Vector3 gradient = GetFieldGradient(x);
            float da = Vector3.Dot(a, gradient);
            float db = Vector3.Dot(b, gradient);
            float dc = Vector3.Dot(c, gradient);
            float dd = Vector3.Dot(d, gradient);
                
            Vector3 s;

            if (da < db && da < dc && da < dd) 
                s = a;
            else if (db < da && db < dc && db < dd)
                s = b;
            else if (dc < da && dc < db && dc < dd)
                s = c;
            else s = d;
                
            float gamma = 0.3f *2.0f /((float) i + 2.0f);
                
            x = (1.0f - gamma) * x + gamma * s;
        }

        if (target.tag == "Sphere") {
            boundDistance = SDFCircle(x, false);
        } else {
            boundDistance = SDFCircle(x, true);
        }
    }

    private void OnDrawGizmos() {
        if (m_Started) {
            localToWorld = transform.localToWorldMatrix;

            m_MeshFilter = gameObject.GetComponent<MeshFilter>();
            m_Mesh = m_MeshFilter.mesh;

            Vector3[] m_Vertices = m_Mesh.vertices;
            int[] m_Triangles = m_Mesh.triangles;

            if (debugVertices) {
                //
                Vector3 position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[0];
                position += transform.position;
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(position, 0.05f * Vector3.one);
                //
                position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[1];
                position += transform.position;
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(position, 0.05f * Vector3.one);
                //
                position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[2];
                position += transform.position;
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(position, 0.05f * Vector3.one);
                //
                position = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale) * m_Vertices[3];
                position += transform.position;
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(position, 0.05f * Vector3.one);
            }
            
            //
            if (boundDistance < threshold)  {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(x, 0.03f);
            }
        }
    }
}