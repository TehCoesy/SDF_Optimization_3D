using UnityEngine;
using System.Collections;

public class SimpleMovement2D : MonoBehaviour {
    private float normalSpeed = 2.0f;
    private float normalRotation = 1.0f;
    
    void Start() {
        Cursor.lockState = CursorLockMode.Locked;
    }
    
    void Update() {
        // Keyboard Inputs
        if (Input.GetKey(KeyCode.A)) {
            transform.position = transform.position + (Vector3.left * normalSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D)) {
            transform.position = transform.position + (Vector3.right * normalSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.W)) {
            transform.position = transform.position + (Vector3.up * normalSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S)) {
            transform.position = transform.position + (Vector3.down * normalSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.Q)) {
            transform.Rotate(Vector3.forward * normalRotation);
        }
        if (Input.GetKey(KeyCode.E)) {
            transform.Rotate(Vector3.back * normalRotation);
        }
    }
}