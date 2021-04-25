using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RaycastCheck : MonoBehaviour
{   
    [SerializeField] GameObject target = null;

    private Vector3 x = Vector3.zero;

    // Start is called before the first frame update
    void Start()
    {
        x = transform.position;   
    }

    // Update is called once per frame
    void Update()
    {   
        x = transform.position;
        if (target) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, (target.transform.position - transform.position).normalized, out hit)) {
                Debug.DrawLine(transform.position, target.transform.position, Color.yellow);
                Debug.DrawRay(hit.point, hit.normal, Color.green);
                x = hit.point;
            }
        }
    }

    private void OnDrawGizmos() {
        if (Application.isPlaying) {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(x, 0.1f);
        }
    }
}
