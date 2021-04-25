using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DFVolume;

public class TextureVisualize : MonoBehaviour
{
    [SerializeField] VolumeData SDF_Data = null;
    int size = 0;
    Color[] data;

    // Start is called before the first frame update
    void Start()
    {
        if (SDF_Data) {
            Texture3D tex = SDF_Data.texture;
            int resolution = SDF_Data.resolution;
            float extent = SDF_Data.extent;

            size = tex.depth;

            data = new Color[size * size * size];

            for (int x = 0; x < size; x++) {
                for (int y = 0; y < size; y++) {
                    for (int z = 0; z < size; z++) {
                        data[GetIndex(x, y, z)] = tex.GetPixel(x, y, z);
                    }
                }
            }

            Debug.Log(resolution + " " + extent);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos() {
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                for (int z = size / 2; z < size / 2 + 1; z++) {
                    Vector3 position = new Vector3(x, y, z);
                    //Gizmos.color = Color.red;
                    Color c = data[GetIndex(x, y, z)];
                    Gizmos.color = c;
                    Gizmos.DrawSphere(transform.position +  0.1f * position, 0.03f);
                }
            }
        }
    }

    private int GetIndex(int xi, int yi, int zi)
    {
        xi = Mathf.Clamp(xi, 0, size - 1);
        yi = Mathf.Clamp(yi, 0, size - 1);
        zi = Mathf.Clamp(zi, 0, size - 1);
        return xi + size * (yi + size * zi);
    }
}
