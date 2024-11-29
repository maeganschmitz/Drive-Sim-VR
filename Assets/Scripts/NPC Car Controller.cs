// NPCCarController.cs
// Script written by Maegan L. Schmitz in 2024
// Updated to fix waypoint detection issue and ensure NPC turns towards waypoints
// Modified to respect the enableCutoff setting

using System.Collections;
using UnityEngine;

public class NPCCarController : MonoBehaviour
{
    //----------------------------------------------------------------------------------------------------------------
    // VARIABLES -----------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Brake light inputs
    [Header("Brake Light Inputs")]
    [Tooltip("Assign the left brake light point light here.")]
    public Light brakeLightLeft;
    [Tooltip("Assign the right brake light point light here.")]
    public Light brakeLightRight;

    // Waypoint settings provided from the spawner
    private float npcTargetSpeed;          // NPC speed in kph
    private float npcAccelerationRate;     // NPC acceleration rate (in g)
    private float npcTurnSpeed;            // Speed at which the NPC car turns
    private float npcWaypointThreshold;    // Waypoint threshold

    // Cutoff settings provided from the spawner
    private float cutoffRange;
    private Vector3 cutoffDirection;
    private float cutoffLateralDistance;   // Total lateral distance to move during cutoff
    private float cutoffDecelerationRate;  // Deceleration rate after cutoff

    private Waypoints npcWaypoints;        // Reference to the NPC's waypoint system
    private Transform currentWaypoint;     // Current waypoint the NPC is targeting
    private float currentSpeed;            // The current speed of the NPC in meters per second (m/s)
    private GameObject egoVehicle;         // Reference to the Ego Vehicle

    // NPC vehicle states
    private bool isCuttingOff = false;     // Indicates the NPC is performing the cutoff maneuver
    private bool isStopping = false;       // Indicates the NPC is slowing down to stop
    private bool isStopped = false;        // Indicates the NPC has stopped moving

    // **New Variable**
    private bool enableCutoff = false;     // Indicates whether cutoff is enabled

    // Public property to provide the NPC's current mode
    public string CurrentMode
    {
        get
        {
            if (isCuttingOff)
                return "Cutoff";
            else if (isStopping)
                return "Stopping";
            else if (isStopped)
                return "Stopped";
            else
                return "Normal";
        }
    }

    // Rigidbody reference
    private Rigidbody rb;

    //----------------------------------------------------------------------------------------------------------------
    // START ---------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    void Start()
    {
        // Initialize the current speed to zero
        currentSpeed = 0f;

        // Get the Rigidbody component
        rb = GetComponent<Rigidbody>();

        // Check if NPC Waypoints have been assigned
        if (npcWaypoints == null)
        {
            Debug.LogError("No waypoint system assigned to the NPC car!");
            return;
        }

        // Initialize the NPC car's waypoint by finding the closest one
        currentWaypoint = FindClosestNPCWaypoint();

        // Initialize brake lights to be off
        if (brakeLightLeft != null)
            brakeLightLeft.enabled = false;

        if (brakeLightRight != null)
            brakeLightRight.enabled = false;

    }

    //----------------------------------------------------------------------------------------------------------------
    // FIXED UPDATE --------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    void FixedUpdate()
    {
        if (isStopped)
        {

            // Update brake lights
            UpdateBrakeLights();

            // NPC has stopped; do nothing
            return;
        }

        if (isCuttingOff)
        {
            // Movement is handled in the CutoffBehavior coroutine during cutoff
        }
        else if (isStopping)
        {

            // Slow down to a stop
            SlowDownToStop();

            // Update brake lights
            UpdateBrakeLights();

        }
        else
        {
            // Follow normal waypoints
            MoveTowardsNPCWaypoint();
            // Check if NPC should cut off the ego vehicle
            CheckCutoffCondition();
        }
    }

    //----------------------------------------------------------------------------------------------------------------
    // METHODS -------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Method to set NPC settings from the NPC Spawner (speed, turn speed, waypoint threshold, and acceleration rate)
    public void SetNPCSettings(float speed, float npcTurnSpeed, float npcWaypointThreshold, float npcAccelerationRate)
    {
        this.npcTargetSpeed = speed;
        this.npcTurnSpeed = npcTurnSpeed;
        this.npcWaypointThreshold = npcWaypointThreshold;
        this.npcAccelerationRate = npcAccelerationRate;
    }

    // Method to set the waypoints manually from another script
    public void SetWaypoints(Waypoints waypoints)
    {
        npcWaypoints = waypoints;
    }

    // Method to dynamically assign the ego vehicle
    public void SetEgoVehicle(GameObject egoVehicleReference)
    {
        egoVehicle = egoVehicleReference;
    }

    // **Modified Method**
    // Method to set the cutoff behavior settings from the NPC Spawner
    public void SetCutoffSettings(
        float range,
        Vector3 direction,
        float lateralDistance,
        float cutoffDecelerationRate
    )
    {
        enableCutoff = true; // Enable cutoff behavior

        cutoffRange = range;
        cutoffDirection = direction.normalized; // Ensure direction is normalized
        cutoffLateralDistance = lateralDistance;
        this.cutoffDecelerationRate = cutoffDecelerationRate; // Set the deceleration rate
    }

    // Method to find the closest waypoint ahead of the car's current position
    private Transform FindClosestNPCWaypoint()
    {
        Transform closestWaypoint = null;
        float closestDistance = Mathf.Infinity;

        // Loop through all waypoints and find the closest one ahead
        for (int i = 0; i < npcWaypoints.transform.childCount; i++)
        {
            Transform waypoint = npcWaypoints.transform.GetChild(i);
            float distanceToWaypoint = Vector3.Distance(transform.position, waypoint.position);

            // Calculate direction to the waypoint
            Vector3 directionToWaypoint = (waypoint.position - transform.position).normalized;

            // Check if the waypoint is ahead of the car (using the dot product to compare direction)
            if (Vector3.Dot(transform.forward, directionToWaypoint) > 0.5f && distanceToWaypoint < closestDistance)
            {
                closestDistance = distanceToWaypoint;
                closestWaypoint = waypoint;
            }
        }

        return closestWaypoint; // Return the closest valid waypoint ahead of the car
    }

    // Method to move towards the current waypoint
    void MoveTowardsNPCWaypoint()
    {
        if (currentWaypoint == null)
        {
            Debug.LogWarning("Current waypoint is null.");
            return;
        }

        float npcTargetSpeedInMetersPerSecond = npcTargetSpeed * 0.27778f; // Convert kph to m/s
        float distanceToWaypoint = Vector3.Distance(transform.position, currentWaypoint.position);
        Vector3 directionToWaypoint = (currentWaypoint.position - transform.position).normalized;

        if (distanceToWaypoint < npcWaypointThreshold + 0.1f)
        {
            currentWaypoint = npcWaypoints.GetNextWaypoint(currentWaypoint);
            if (currentWaypoint == null)
            {
                // No more waypoints; stop moving
                currentSpeed = 0f;
                rb.velocity = Vector3.zero;
                isStopped = true;
                return;
            }
        }

        // Convert acceleration from g to m/s² (1g = 9.81 m/s²)
        float accelerationInMetersPerSecondSquared = npcAccelerationRate * 9.81f;

        // Gradually accelerate or decelerate the car towards the waypoint's speed
        currentSpeed = Mathf.MoveTowards(currentSpeed, npcTargetSpeedInMetersPerSecond, accelerationInMetersPerSecondSquared * Time.deltaTime);

        // Move the NPC towards the current waypoint at the target speed
        Vector3 movement = directionToWaypoint * currentSpeed * Time.deltaTime;
        rb.MovePosition(rb.position + movement);

        // Update Rigidbody velocity
        rb.velocity = directionToWaypoint * currentSpeed;

        // Smoothly rotate the NPC towards the waypoint
        Quaternion targetRotation = Quaternion.LookRotation(directionToWaypoint);
        Quaternion newRotation = Quaternion.Slerp(rb.rotation, targetRotation, npcTurnSpeed * Time.deltaTime);
        rb.MoveRotation(newRotation);

        // Debug logs for troubleshooting
        Debug.Log($"NPC '{gameObject.name}' moving towards waypoint '{currentWaypoint.name}'.");
        Debug.Log($"Distance to waypoint: {distanceToWaypoint}");
        Debug.Log($"Current speed: {currentSpeed} m/s");
        Debug.Log($"Current rotation: {rb.rotation.eulerAngles}");
        Debug.Log($"Target rotation: {targetRotation.eulerAngles}");
    }

    // **Modified Method**
    // Check if the NPC should initiate a cutoff maneuver
    private void CheckCutoffCondition()
    {
        if (enableCutoff && egoVehicle != null && !isCuttingOff)
        {
            // Calculate the distance along the Z-axis
            float distanceZ = transform.position.z - egoVehicle.transform.position.z;

            // Check if the NPC is ahead of the ego vehicle by cutoffRange units
            if (distanceZ >= cutoffRange)
            {
                // Start the cutoff behavior
                StartCoroutine(CutoffBehavior());
                Debug.Log("NPC is initiating cutoff maneuver.");
            }
        }
    }

    // Coroutine for cutoff behavior
    private IEnumerator CutoffBehavior()
    {
        isCuttingOff = true;

        // Maintain current forward speed during cutoff
        float forwardSpeedInMetersPerSecond = currentSpeed;

        // Calculate the lateral speed needed to cover cutoffLateralDistance
        float totalCutoffTime = cutoffLateralDistance / forwardSpeedInMetersPerSecond;
        float lateralSpeed = cutoffLateralDistance / totalCutoffTime;

        // Define the initial rotation and target rotation toward the cutoff direction
        Quaternion initialRotation = rb.rotation;
        Quaternion targetRotation = Quaternion.LookRotation(cutoffDirection);

        // Clamp the rotation to a maximum of 15 degrees
        targetRotation = Quaternion.RotateTowards(initialRotation, targetRotation, 15f);

        float totalLateralDistanceMoved = 0f;

        while (totalLateralDistanceMoved < cutoffLateralDistance)
        {
            float deltaTime = Time.deltaTime;

            // Calculate the forward movement for this frame (constant speed)
            Vector3 forwardMovement = transform.forward * forwardSpeedInMetersPerSecond * deltaTime;

            // Calculate the lateral movement for this frame, ensuring we don't exceed the cutoff distance
            float frameLateralMovement = Mathf.Min(lateralSpeed * deltaTime, cutoffLateralDistance - totalLateralDistanceMoved);
            Vector3 lateralMovement = cutoffDirection * frameLateralMovement;

            // Accumulate the total lateral distance moved
            totalLateralDistanceMoved += frameLateralMovement;

            // Combine the forward and lateral movements into a single movement vector
            Vector3 combinedMovement = forwardMovement + lateralMovement;

            // Update the NPC's position using Rigidbody
            rb.MovePosition(rb.position + combinedMovement);

            // Ensure the forward velocity remains constant while adjusting the lateral velocity
            rb.velocity = forwardMovement / deltaTime;

            // Rotate towards the target rotation during the first half of the cutoff distance
            if (totalLateralDistanceMoved < cutoffLateralDistance * 0.5f)
            {
                float rotationFactor = Mathf.Clamp01(totalLateralDistanceMoved / (cutoffLateralDistance * 0.5f));
                Quaternion newRotation = Quaternion.Slerp(initialRotation, targetRotation, rotationFactor);
                rb.MoveRotation(newRotation);
            }
            // Rotate back towards the initial rotation during the second half of the cutoff distance
            else
            {
                float rotationFactor = Mathf.Clamp01((totalLateralDistanceMoved - cutoffLateralDistance * 0.5f) / (cutoffLateralDistance * 0.5f));
                Quaternion newRotation = Quaternion.Slerp(targetRotation, initialRotation, rotationFactor);
                rb.MoveRotation(newRotation);
            }

            yield return null; // Wait for the next frame
        }

        // Start decelerating to a stop after completing the cutoff maneuver
        isCuttingOff = false;
        isStopping = true;
        Debug.Log("NPC is stopping after cutoff.");
    }

    // Method to gradually slow down the NPC to a stop
    private void SlowDownToStop()
    {
        if (currentSpeed > 0)
        {
            currentSpeed -= cutoffDecelerationRate * Time.deltaTime;
            if (currentSpeed < 0)
                currentSpeed = 0;
        }

        // Move forward at current speed
        float currentSpeedInMetersPerSecond = currentSpeed;
        Vector3 movement = transform.forward * currentSpeedInMetersPerSecond * Time.deltaTime;
        rb.MovePosition(rb.position + movement);

        // Update Rigidbody velocity
        rb.velocity = transform.forward * currentSpeedInMetersPerSecond;

        if (currentSpeed == 0)
        {
            // Set velocity to zero when stopped
            rb.velocity = Vector3.zero;

            isStopping = false;
            isStopped = true;
        }
    }

    void OnDrawGizmos()
    {
        // Visualize the cutoff range as a wireframe sphere around the Ego Vehicle
        if (enableCutoff && egoVehicle != null)
        {
            Gizmos.color = Color.red; // Color for cutoff range
            Gizmos.DrawWireSphere(egoVehicle.transform.position, cutoffRange); // Cutoff range around the ego vehicle
        }

        // Visualize the cutoff direction as an arrow
        if (enableCutoff)
        {
            Gizmos.color = Color.red; // Color for cutoff direction
            Vector3 cutoffEndPoint = transform.position + (cutoffDirection.normalized * cutoffLateralDistance);
            Gizmos.DrawLine(transform.position, cutoffEndPoint); // Draw a line for cutoff direction
            Gizmos.DrawSphere(cutoffEndPoint, 0.5f); // Draw a small sphere at the cutoff end point
        }
    }

    private void UpdateBrakeLights()
    {
        bool shouldBrakeLightsBeOn = isStopping || isStopped;

        if (brakeLightLeft != null)
            brakeLightLeft.enabled = shouldBrakeLightsBeOn;

        if (brakeLightRight != null)
            brakeLightRight.enabled = shouldBrakeLightsBeOn;
    }
}