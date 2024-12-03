using UnityEngine;

public class JoystickInputDebugger : MonoBehaviour
{
    void Update()
    {
        // Log the raw values of configured inputs
        float steeringInput = Input.GetAxis("Steering");
        float acceleratorInput = Input.GetAxis("Accelerator");
        float brakeInput = Input.GetAxis("Brake");

        Debug.Log($"Steering (Joystick 1, X Axis): {steeringInput}");
        Debug.Log($"Accelerator (Joystick 3, Y Axis): {acceleratorInput}");
        Debug.Log($"Brake (Joystick 2, Y Axis): {brakeInput}");
    }
}
