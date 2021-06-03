using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmoothCameraTransition : MonoBehaviour
{   
    [SerializeField]
    private Vector3 GoPosition;

    private bool isMoving = false;

    public void moveTo(Vector3 newPosition) {
        GoPosition = newPosition;
    }

    // Start is called before the first frame update
    void Start()
    {
        GoPosition = transform.position + new Vector3(3, 0, 0);    
    }

    // Update is called once per frame
    void Update()
    {
        if (GoPosition != transform.position) {
            isMoving = true;
        } else {
            isMoving = false;
        }

        if (isMoving) {
            Vector3 newPosition = Vector3.Lerp(transform.position, GoPosition, Time.deltaTime * 2.0f);
            transform.position = newPosition;
        }
    }
}
