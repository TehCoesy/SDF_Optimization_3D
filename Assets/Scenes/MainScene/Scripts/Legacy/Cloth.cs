using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cloth : MonoBehaviour
{   
    [SerializeField]
    GameObject meshPrefab = null, target = null;

    int sizeX = 10, sizeY = 10;
    float prefabX, prefabY;

    GameObject[] meshObjects;

    // Start is called before the first frame update
    void Start()
    {
        if (!meshPrefab) {
            return;
        }

        meshObjects = new GameObject[sizeX * sizeY];

        MeshFilter meshFilter = meshPrefab.GetComponent<MeshFilter>();
        Mesh mesh = meshFilter.sharedMesh;

        prefabX = mesh.bounds.size.x * meshPrefab.transform.localScale.x;
        prefabY = mesh.bounds.size.z * meshPrefab.transform.localScale.z;

        for (int i = 0; i < sizeY; i++) {
            for (int k = 0; k < sizeX; k++) {
                // Spawn cloth parallel to ground
                Vector3 offset = i * prefabX * Vector3.right +  k * prefabY * Vector3.forward;
                GameObject inst = Instantiate(meshPrefab, transform.position + offset, transform.rotation);
                
                // A mesh may have up to 6 neighbours
                // LU UU UR
                // LL MS RR
                // LD DD DR

                if (k != 0) {
                    // Connect left neighbour
                    SpringJoint joint1 = inst.AddComponent<SpringJoint>() as SpringJoint;
                    joint1.connectedBody = meshObjects[i * sizeY + k - 1].GetComponent<Rigidbody>();
                    joint1.spring = 10000.0f;
                    joint1.anchor = new Vector3(-1.0f, 0.0f, -1.0f);
                    joint1.axis = Vector3.one;

                    SpringJoint joint2 = inst.AddComponent<SpringJoint>() as SpringJoint;
                    joint2.connectedBody = meshObjects[i * sizeY + k - 1].GetComponent<Rigidbody>();
                    joint2.spring = 10000.0f;
                    joint2.anchor = new Vector3(1.0f, 0.0f, -1.0f);
                    joint2.axis = Vector3.one;
                }

                if (k != sizeX  - 1) {
                    // Connect right neighbour
                }

                if (i != 0) {
                    // Connect upper neighbour
                    SpringJoint joint3 = inst.AddComponent<SpringJoint>() as SpringJoint;
                    joint3.connectedBody = meshObjects[(i - 1) * sizeY + k].GetComponent<Rigidbody>();
                    joint3.spring = 10000.0f;
                    joint3.anchor = new Vector3(-1.0f, 0.0f, -1.0f);
                    joint3.axis = Vector3.one;


                    SpringJoint joint4 = inst.AddComponent<SpringJoint>() as SpringJoint;
                    joint4.connectedBody = meshObjects[(i - 1) * sizeY + k].GetComponent<Rigidbody>();
                    joint4.spring = 10000.0f;
                    joint4.anchor = new Vector3(-1.0f, 0.0f, 1.0f);
                    joint4.axis = Vector3.one;
                }

                if (k != sizeX - 1) {
                    // Connect lower neighbour
                }

                if (i == 0 && k == 0) {
                    ConfigurableJoint joint1 = inst.AddComponent<ConfigurableJoint>() as ConfigurableJoint;
                    //joint1.connectedBody = meshObjects[i * sizeY + k - 1].GetComponent<Rigidbody>();
                    joint1.xMotion = ConfigurableJointMotion.Locked;
                    joint1.yMotion = ConfigurableJointMotion.Locked;
                    joint1.zMotion = ConfigurableJointMotion.Locked;
                    joint1.anchor = new Vector3(-1.0f, 0.0f, -1.0f);
                    joint1.axis = Vector3.one;
                } else if (i == 0 && k == sizeX - 1) {
                    ConfigurableJoint joint1 = inst.AddComponent<ConfigurableJoint>() as ConfigurableJoint;
                    //joint1.connectedBody = meshObjects[i * sizeY + k - 1].GetComponent<Rigidbody>();
                    joint1.xMotion = ConfigurableJointMotion.Locked;
                    joint1.yMotion = ConfigurableJointMotion.Locked;
                    joint1.zMotion = ConfigurableJointMotion.Locked;
                    joint1.anchor = new Vector3(-1.0f, 0.0f, 1.0f);
                    joint1.axis = Vector3.one;
                }
                
                if (target) {
                    inst.GetComponent<SDF_3D>().target = target;
                }

                meshObjects[i * sizeY + k] = inst;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
