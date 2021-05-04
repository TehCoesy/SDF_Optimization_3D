using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public enum Method {FrankWolfe, GradientDescent};

public class SDF_3D : MonoBehaviour
{
    // gameObject Variables
    MeshFilter m_MeshFilter;
    Mesh m_Mesh;

    // SDF Variables
    Vector3 x1, x2;
    float distance1, distance2;
    
    // Target Variables
    [SerializeField]
    public GameObject target = null;

    // Method
    [SerializeField]
    public int n = 16;
    public Method method = Method.FrankWolfe;


    private Vector3 targetBounds = Vector3.zero;
    private Collider targetCollider = null;

    // Others
    

    void Start()
    {
        // Establish gameObject's mesh
        m_MeshFilter = gameObject.GetComponent<MeshFilter>();
        m_Mesh = m_MeshFilter.mesh;

        // Establish target's mesh
        if (target) {
            targetBounds = target.GetComponent<Renderer>().bounds.size;
            targetCollider = target.GetComponent<Collider>();
        }
    }

    // Distance to a sphere's surface
    float SDFCircle(Vector3 a) {
        float da = (a - target.transform.position).magnitude - targetBounds.x;
        return Mathf.Abs(da);   
        //return Mathf.Clamp(da, -1.0f, 1.0f);
    }

    // Distance to a box's surface
    // Can be used for a bounding box or a cube mesh
    float SDFBox(Vector3 a) {
        Vector3 p = a - target.transform.position;
        Vector3 targetBounds = target.GetComponent<Renderer>().bounds.size;
        
        float left = -targetBounds.x / 2, right = targetBounds.x / 2;
        float bottom = -targetBounds.y / 2, top = targetBounds.y / 2;
        float back = -targetBounds.z / 2, forward = targetBounds.z / 2;

        float dx = DistanceAux(p.x, - targetBounds.x / 2, targetBounds.x / 2);
        float dy = DistanceAux(p.y, - targetBounds.y / 2, targetBounds.y / 2);
        float dz = DistanceAux(p.z, - targetBounds.z / 2, targetBounds.z / 2);

        if (a.x > left && a.x < right && a.y > bottom && a.y < top && a.z > back && a.z < forward) {
            return Mathf.Min(dx, dy, dz);
        }
        return Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    float DistanceAux(float x, float min, float max) {
        if (x < min) return min - x;
        else if (x > max) return x - max;
        else return 0.0f;
    }

    float SDFLine(Vector3 x, Vector3 a, Vector3 b) {
        return 0.0f;
    }

    private float GetField(Vector3 a) {
        if (target && targetCollider) {
            if (targetCollider.GetType() == typeof(SphereCollider)) {
                return SDFCircle(a);
            } else if (targetCollider.GetType() == typeof(BoxCollider)) {
                return SDFBox(a);
            }
        }
        return 1.0f;
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
        Vector3[] m_Vertices = m_Mesh.vertices;
        int[] m_Triangles = m_Mesh.triangles;

        Vector3 a = transform.TransformPoint(m_Vertices[0]);
        Vector3 b = transform.TransformPoint(m_Vertices[1]);
        Vector3 c = transform.TransformPoint(m_Vertices[2]);
        Vector3 d = transform.TransformPoint(m_Vertices[3]);

        x1 = (a + b + c) / 3;
        x2 = (b + c + d) / 3;

        //float u = 1.0f / 3;
        //float v = 1.0f / 3;
        //float w = 1.0f / 3;

        // Search iterations for x1
        for (int i = 0; i < n; i++) {
            if (method == Method.FrankWolfe) {
                Vector3 gradient = GetFieldGradient(x1);
                float da = Vector3.Dot(a, gradient);
                float db = Vector3.Dot(b, gradient);
                float dc = Vector3.Dot(c, gradient);
                    
                Vector3 s;

                if (da < db && da < dc) 
                    s = a;
                else if (db < da && db < dc)
                    s = b;
                else s = c;
                    
                float gamma = 2.0f /((float)i + 2.0f);
                    
                x1 = x1 + gamma * (s - x1);
            } else {
                
                Vector3 gradient = GetFieldGradient(x1);
                float alpha = 0.05f;
                
                Vector3 p = x1 - alpha*gradient;

                x1 = projectTri(a, b, c, p);
            }
        }

        // Search iterations for x2
        for (int i = 0; i < n; i++) {
            if (method == Method.FrankWolfe) {
                Vector3 gradient = GetFieldGradient(x2);
                float db = Vector3.Dot(b, gradient);
                float dc = Vector3.Dot(c, gradient);
                float dd = Vector3.Dot(d, gradient);
                    
                Vector3 s;

                if (db < dc && db < dd) 
                    s = b;
                else if (dc < db && dc < dd)
                    s = c;
                else s = d;
                    
                float gamma = 2.0f /((float) i + 2.0f);
                    
                x2 = (1.0f - gamma) * x2 + gamma * s;
            } else {

            }
        }
    }

    private void OnDrawGizmos() {
        if (Application.isPlaying) {
            Vector3[] m_Vertices = m_Mesh.vertices;
            int[] m_Triangles = m_Mesh.triangles;

            Vector3 a = transform.TransformPoint(m_Vertices[0]);
            Vector3 b = transform.TransformPoint(m_Vertices[1]);
            Vector3 c = transform.TransformPoint(m_Vertices[2]);
            Vector3 d = transform.TransformPoint(m_Vertices[3]);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(a, 0.3f * Vector3.one);
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(b, 0.3f * Vector3.one);
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(c, 0.3f * Vector3.one);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(x1, 0.3f);
            Gizmos.color = Color.red;
            //Gizmos.DrawSphere(x2, 0.3f);
        }
    }

    private Vector3 projectTri(Vector3 a, Vector3 b, Vector3 c, Vector3 p) {
        
        float snom = Vector3.Dot(p - a, b - a), sdenom = Vector3.Dot(p - b, a - b);
        float tnom = Vector3.Dot(p - a, c - a), tdenom = Vector3.Dot(p - c, a - c);

        if (snom <= 0.0f && tnom <= 0.0f) return a;

        float unom = Vector3.Dot(p - b, c - b), udenom = Vector3.Dot(p - c, b - c);

        if (sdenom <= 0.0f && unom <= 0.0f)	return b;
	    if (tdenom <= 0.0f && udenom <= 0.0f)	return c;

        Vector3 n = Vector3.Cross(b - a, c - a);

        float vc = Vector3.Dot(n, Vector3.Cross(a - p, b - p));
        if (vc <= 0.0f && snom >= 0.0f && sdenom >= 0.0f) return a + snom / (snom + sdenom) * (b - a);

        float va = Vector3.Dot(n, Vector3.Cross(b - p, c - p));
        if (va <= 0.0f && unom >= 0.0f && udenom >= 0.0f) return b + unom / (unom + udenom) * (c - b);

        float vb = Vector3.Dot(n, Vector3.Cross(c - p, a - p));
        if (vb <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f) return a + tnom / (tnom + tdenom) * (c - a);

        float u = va / (va + vb + vc);
        float v = vb / (va + vb + vc);
        float w = 1.0f - u - v;
        return u*a + v*b + w*c;
    }
}