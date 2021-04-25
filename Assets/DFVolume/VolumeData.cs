// DFVolume - Distance field volume generator for Unity
// https://github.com/keijiro/DFVolume

using UnityEngine;

namespace DFVolume
{
    public class VolumeData : ScriptableObject
    {
        #region Exposed attributes

        [SerializeField] Texture3D _texture;

        [SerializeField] int _resolution;

        [SerializeField] float _extent;

        public Texture3D texture {
            get { return _texture; }
        }

        public int resolution {
            get { return _resolution; }
        }

        public float extent {
            get { return _extent; }
        }

        #endregion

        #if UNITY_EDITOR

        #region Editor functions

        public void Initialize(VolumeSampler sampler)
        {
            _resolution = sampler.resolution;
            _texture = new Texture3D(_resolution, _resolution, _resolution, TextureFormat.RGBAHalf, true);

            _texture.name = "Distance Field Texture";
            _texture.filterMode = FilterMode.Bilinear;
            _texture.wrapMode = TextureWrapMode.Clamp;
            _texture.SetPixels(sampler.GenerateBitmap());
            _texture.Apply();

            _extent = sampler.extent;
        }

        #endregion

        #endif
    }
}
