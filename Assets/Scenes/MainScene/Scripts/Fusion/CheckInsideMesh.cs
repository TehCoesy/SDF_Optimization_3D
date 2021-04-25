using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CheckInsideMesh : MonoBehaviour
{   
    [SerializeField] GameObject target = null;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (target) {
            MeshCollider collider = target.GetComponent<MeshCollider>();
            if (IsInCollider(collider, transform.position)) {
                Debug.Log("Hit!");
            } else {
                Debug.Log("NoHit");
            }
        }
    }

    private bool IsInCollider(MeshCollider other, Vector3 point) {
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
 
    private int Cast_Till(Vector3 from, Vector3 to, MeshCollider other) {
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
}
