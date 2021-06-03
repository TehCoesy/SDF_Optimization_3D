using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DemoController : MonoBehaviour
{
    [SerializeField]
    public Camera mainCamera;
    public List<GameObject> clothing;
    public Vector3 cameraOffset;

    private int current = 1;
    private Vector3 goPosition;
    private bool moving = false;

    // Start is called before the first frame update
    void Start()
    {
        goPosition = mainCamera.transform.position;
        play();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown("q") && !moving) {
            if (current != 0) {
                pause();
                current--;
                goPosition += cameraOffset;
                moving = true;
            }
        }
        if (Input.GetKeyDown("e") && !moving) {
            if (current != clothing.Count - 1) {
                pause();
                current++;
                goPosition -= cameraOffset;
                moving = true;
            }
        }
        if (Input.GetKeyDown("r") && !moving) {
            reset();
        }
        if (Input.GetKeyDown("space") && !moving) {
            togglePause();
        }

        // Handle camera movement
        if (moving) {
            mainCamera.transform.position = Vector3.Slerp(mainCamera.transform.position, goPosition, Time.deltaTime * 1.5f);
            if ((mainCamera.transform.position - goPosition).magnitude < 0.1f) {
                play();
                mainCamera.transform.position = goPosition;
                moving = false;
            }
        }
    }

    private void togglePause() {
        GameObject piece = clothing[current];
        ClothSimulatorModified script = piece.GetComponent<ClothSimulatorModified>();
        if (script.isPlaying) {
            script.pauseSimulation();
        }
        else {
            script.playSimulation();
        }
    }

    private void pause() {
        GameObject piece = clothing[current];
        ClothSimulatorModified script = piece.GetComponent<ClothSimulatorModified>();
        script.pauseSimulation();
    }

    private void play() {
        GameObject piece = clothing[current];
        ClothSimulatorModified script = piece.GetComponent<ClothSimulatorModified>();
        script.playSimulation();
    }

    private void reset() {
        GameObject piece = clothing[current];
        ClothSimulatorModified script = piece.GetComponent<ClothSimulatorModified>();
        script.resetSimulation();
    }
}
