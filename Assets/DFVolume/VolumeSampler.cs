﻿// DFVolume - Distance field volume generator for Unity
// https://github.com/keijiro/DFVolume

using UnityEngine;

namespace DFVolume
{
    public class VolumeSampler : MonoBehaviour
    {
        #region Exposed attributes

        [SerializeField] int _resolution = 50;

        public int resolution {
            get { return _resolution; }
        }

        [SerializeField] float _extent = 0.5f;

        public float extent {
            get { return _extent; }
        }

        [SerializeField] GameObject _target = null;

        public GameObject target {
            get { return _target; }
        }

        #endregion

        #if UNITY_EDITOR

        #region Editor functions

        void OnDrawGizmos()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one * _extent * 2);
        }

        public Color[] GenerateBitmap()
        {   
            // Generate a distance field.
            MeshCollider collider = target.GetComponent<MeshCollider>();
            var df = new float[_resolution * _resolution * _resolution];

            for (var xi = 0; xi < _resolution; xi++)
            {
                var x = 2.0f * xi / (_resolution - 1) - 1;
                for (var yi = 0; yi < _resolution; yi++)
                {
                    var y = 2.0f * yi / (_resolution - 1) - 1;
                    for (var zi = 0; zi < _resolution; zi++)
                    {
                        var z = 2.0f * zi / (_resolution - 1) - 1;

                        var pt = new Vector3(x, y, z) * _extent;
                        pt = transform.TransformPoint(pt);

                        float dist = 0.0f;

                        if (collider) {
                            if (IsInCollider(collider, pt)) {
                                dist = -(SearchDistance(pt));
                            } else {
                                dist = SearchDistance(pt);
                            }
                        }
                        
                        df[GetIndex(xi, yi, zi)] = dist;
                    }
                }
            }

            // Compute gradients and pack them into a bitmap.
            var bmp = new Color[df.Length];
            var dds2 = (_resolution - 1) / 2.0f;

            for (var xi = 0; xi < _resolution; xi++)
            {
                for (var yi = 0; yi < _resolution; yi++)
                {
                    for (var zi = 0; zi < _resolution; zi++)
                    {
                        var d = df[GetIndex(xi, yi, zi)];
                        var dx0 = df[GetIndex(xi - 1, yi, zi)];
                        var dx1 = df[GetIndex(xi + 1, yi, zi)];
                        var dy0 = df[GetIndex(xi, yi - 1, zi)];
                        var dy1 = df[GetIndex(xi, yi + 1, zi)];
                        var dz0 = df[GetIndex(xi, yi, zi - 1)];
                        var dz1 = df[GetIndex(xi, yi, zi + 1)];

                        bmp[GetIndex(xi, yi, zi)] = new Color(
                            (dx1 - dx0) * dds2,
                            (dy1 - dy0) * dds2,
                            (dz1 - dz0) * dds2,
                            d
                        );
                    }
                }
            }

            return bmp;
        }

        #endregion

        #region Private functions

        int GetIndex(int xi, int yi, int zi)
        {
            xi = Mathf.Clamp(xi, 0, _resolution - 1);
            yi = Mathf.Clamp(yi, 0, _resolution - 1);
            zi = Mathf.Clamp(zi, 0, _resolution - 1);
            return xi + _resolution * (yi + _resolution * zi);
        }

        float SearchDistance(Vector3 pt)
        {
            var r = _extent;
            var s = _extent * 0.5f;

            for (var i = 0; i < 10; i++)
            {
                r += (Physics.CheckSphere(pt, r) ? -1 : 1) * s;
                s *= 0.5f;
            }

            return r;
        }

        public  bool IsInCollider(MeshCollider other, Vector3 point) {
            Vector3 from = (Vector3.up * 5000f);
            Vector3 dir = (point - from).normalized;
            float dist = Vector3.Distance(from, point);        
            //fwd      
            int hit_count = Cast_Till(from, point, other);
            //back
            dir = (from - point).normalized;
            hit_count += Cast_Till(point, point + (dir * dist), other);
    
            if (hit_count % 2 == 1) {
                return (true);
            }
            return (false);
        }
    
        int Cast_Till(Vector3 from, Vector3 to, MeshCollider other) {
            int counter = 0;
            Vector3 dir = (to - from).normalized;
            float dist = Vector3.Distance(from, to);
            bool Break = false;
            while (!Break) {
                Break = true;
                RaycastHit[] hit = Physics.RaycastAll(from, dir, dist);
                for (int tt = 0; tt < hit.Length; tt++) {
                    if (hit[tt].collider == other) {
                        counter++;
                        from = hit[tt].point+dir.normalized*.001f;
                        dist = Vector3.Distance(from, to);
                        Break = false;
                        break;                    
                    }
                }
            }
            return (counter);
        }

        #endregion

        #endif
    }
}
