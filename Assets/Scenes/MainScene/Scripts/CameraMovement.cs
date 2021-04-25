using UnityEngine;
using System.Collections;

public class CameraMovement : MonoBehaviour {
    private float normalSpeed = 2.0f;
    private float sprintSpeed = 10.0f;
    private float speedMultiplier = 0.0f;
    private float camSensitivity = 1.0f;
    
    void Start() {
        Cursor.lockState = CursorLockMode.Locked;
    }

    void Update() {
        // Keyboard Inputs
        if (Input.GetKey(KeyCode.LeftShift)) {
            speedMultiplier = normalSpeed;
        } else {
            speedMultiplier = sprintSpeed;
        }

        //Vector3 m = GetBaseInput();

        if (Input.GetKey(KeyCode.A)) {
            transform.position = transform.position + (-transform.right * speedMultiplier * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D)) {
            transform.position = transform.position + (transform.right * speedMultiplier * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.W)) {
            transform.position = transform.position + (transform.forward * speedMultiplier * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S)) {
            transform.position = transform.position + (-transform.forward * speedMultiplier * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.Space)) {
            transform.position = transform.position + (Vector3.up * speedMultiplier * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.LeftControl)) {
            transform.position = transform.position + (Vector3.down * speedMultiplier * Time.deltaTime);
        }

        // Mouse Inputs
        float newRotationX = transform.localEulerAngles.y + Input.GetAxis("Mouse X") * camSensitivity;
        float newRotationY = transform.localEulerAngles.x - Input.GetAxis("Mouse Y") * camSensitivity;
        transform.localEulerAngles = new Vector3(newRotationY, newRotationX, 0f);
    }
}