// CarController.cs
// Script controlling ego car and automated driving system

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public enum Gear
{
    Park,
    Reverse,
    Neutral,
    Drive
}

public class CarController : MonoBehaviour
{
    //----------------------------------------------------------------------------------------------------------------
    // VARIABLES -----------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Variables to store user input and car status
    private float horizontalInput, verticalInput; // Input for steering and acceleration
    private float currentSteerAngle, currentBrakeForce; // Current steering angle and braking force
    private bool isBraking; // Whether the car is braking

    // Gear system variables
    [Header("Car Initialization Settings")]
    public Gear currentGear = Gear.Park; // Start in Park
    private bool isHandbrakeEngaged = true; // Handbrake is engaged in Park
    [SerializeField] private float steeringInputThreshold = 0.1f; // Threshold for joystick steering input

    // Variables to store car physics
    [Header("Ego Physics Settings")]
    [Tooltip("Motor Force in Newton-meters (Nm)")]
    [SerializeField] private float motorForce; // Editable in Inspector
    [Tooltip("Brake Force in Newton-meters (Nm)")]
    [SerializeField] private float brakeForce; // Editable in Inspector
    [Tooltip("Max Steering Angle in degrees (°)")]
    [SerializeField] private float maxSteerAngle; // Editable in Inspector

    // Waypoint System Settings
    [Header("Ego Waypoint Settings")]
    [Tooltip("Waypoint speed in kilometers per hour (kph). Determines the speed of the vehicle while moving between waypoints.")]
    [SerializeField] private float egoTargetSpeed = 50f; // Default waypoint speed in kph
    [Tooltip("Acceleration rate in g (1g = 9.81 m/s²). Determines how quickly the car accelerates to the target speed.")]
    [SerializeField] private float egoAccelerationRate = 0.2f; // Default acceleration in g
    [Tooltip("Turn speed factor (unitless). Higher values result in faster turning toward waypoints.")]
    [SerializeField] private float egoTurnSpeed;  // Speed at which the car turns
    [Tooltip("Distance threshold in meters (m) to determine if the car is close enough to the waypoint to proceed to the next.")]
    [SerializeField] private float egoWaypointThreshold; // Distance to determine if the car is close enough to the waypoint

    // Reference to the Waypoints script, which manages the waypoint system
    public Waypoints egoWaypoints;
    private Transform currentWaypoint; // This will store the current target waypoint that the car is moving towards
    private float currentSpeed = 0f; // The current speed of the car (starts at 0)

    // A boolean flag to indicate whether the car is moving automatically or not
    private bool isFollowingWaypoints = false;
    private bool isAutomaticMode = false; // Flag for automatic mode
    public bool IsAutomaticMode
    {
        get { return isAutomaticMode; }
    }

    // Wheel Colliders for simulating wheel physics
    [Header("Ego Input Settings")]
    [Tooltip("Drag objects over from prefabs or hierarchy.")]
    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    // Transforms for the visual representation of the wheels
    [SerializeField] private Transform frontLeftWheelTransform, frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform, rearRightWheelTransform;

    // Speed GUI
    [Header("Ego GUI Settings")]
    [Tooltip("Drag objects over from prefabs or hierarchy.")]
    [SerializeField] public TMP_Text speedText; // Existing speed display
    [SerializeField] public TMP_Text targetSpeedText; // Target speed display
    [SerializeField] public TMP_Text autopilotSpeedText; // Autopilot speed display
    private Vector3 previousPosition; // Store the previous position of the car

    // Gear indicator spheres and automatic mode indicator
    [Header("Ego Gear Indicator Settings")]
    [Tooltip("Assign the gear indicator spheres here.")]
    public GameObject parkIndicator;
    public GameObject reverseIndicator;
    public GameObject neutralIndicator;
    public GameObject driveIndicator;

    [Tooltip("Assign the automatic mode indicator here.")]
    public GameObject automaticModeIndicator; // Variable for automatic mode indicator

    // Audio settings
    [Header("Ego Audio Settings")]
    [Tooltip("Sound to play when automatic mode is activated.")]
    public AudioClip automaticModeOnSound;

    [Tooltip("Sound to play when automatic mode is deactivated.")]
    public AudioClip automaticModeOffSound;

    // Reference to the AudioSource component
    private AudioSource audioSource;

    // Rigidbody reference
    private Rigidbody rb;

    // Speed GUI smoothing variables
    private float speedUpdateInterval = 0.1f; // Update every 0.1 seconds
    private float timeSinceLastSpeedUpdate = 0f;
    private float displayedSpeed = 0f;
    private float previousActualSpeed = 0f;

    // Collision tracker variables
    private string collidedObjectName = "None"; // Variable to store the name of the collided object
    public string CollidedObjectName // Property to access the collided object's name
    {
        get { return collidedObjectName; }
    }
    private bool hasCollisionOccurred = false; // Variable to indicate if a collision occurred since the last check
    public bool HasCollisionOccurred // Property to check if a collision occurred
    {
        get { return hasCollisionOccurred; }
    }

    //----------------------------------------------------------------------------------------------------------------
    // START ---------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Start is called before the first frame update
    void Start()
    {
        // Initialize the currentWaypoint by finding the closest one
        currentWaypoint = FindClosestWaypoint();

        // Initialize the previous position
        previousPosition = transform.position;

        // Get the Rigidbody component
        rb = GetComponent<Rigidbody>();

        // Get the AudioSource component
        audioSource = GetComponent<AudioSource>();
        if (audioSource == null)
        {
            Debug.LogWarning("AudioSource component not found on the car GameObject. Please add one to play sounds.");
        }

        // Initialize gear indicators
        UpdateGearIndicators();

        // Initialize automatic mode indicator
        UpdateAutomaticModeIndicator();

        // Initialize ego target speed display
        UpdateEgoTargetSpeedDisplay();

        // Initialize autopilot speed display
        UpdateAutopilotSpeedDisplay();
    }

    //----------------------------------------------------------------------------------------------------------------
    // UPDATE --------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Update is called once per frame
    void Update()
    {
        // Handle gear selection input
        HandleGearInput();

        // Automated driving system - Checks to see if automated driving system should be active
        if (Input.GetKeyDown(KeyCode.A)) // Check if the 'A' key has been pressed to toggle automatic mode
        {
            if (currentGear == Gear.Drive)
            {
                isAutomaticMode = !isAutomaticMode; // Toggle automatic mode on and off
                isFollowingWaypoints = isAutomaticMode; // Set waypoint following based on automatic mode

                // Update the automatic mode indicator
                UpdateAutomaticModeIndicator();

                // Play sound notification
                if (audioSource != null)
                {
                    if (isAutomaticMode && automaticModeOnSound != null)
                    {
                        audioSource.PlayOneShot(automaticModeOnSound);
                    }
                    else if (!isAutomaticMode && automaticModeOffSound != null)
                    {
                        audioSource.PlayOneShot(automaticModeOffSound);
                    }
                }

                if (isAutomaticMode)
                {
                    // Update the closest valid waypoint when switching to automatic mode
                    currentWaypoint = FindClosestWaypoint();

                    // Set the current speed to the car's current velocity when switching to AD
                    if (rb != null)
                    {
                        currentSpeed = rb.velocity.magnitude; // Speed in meters per second
                    }
                }
            }
            else
            {
                // Provide feedback that automatic mode can only be activated when the car is in Drive
                Debug.Log("Automatic mode can only be activated when the car is in Drive.");
            }
        }

        // Check if joystick or keyboard inputs are detected to disable automatic mode
        bool manualInputDetected =
            Mathf.Abs(Input.GetAxis("Steering")) > steeringInputThreshold || // Joystick steering threshold
            Input.GetAxis("Accelerator") > -1f || // Joystick accelerator
            Input.GetAxis("Brake") > -1f || // Joystick brake
            Input.GetKey(KeyCode.UpArrow) || // Keyboard forward
            Input.GetKey(KeyCode.LeftArrow) || // Keyboard left
            Input.GetKey(KeyCode.DownArrow) || // Keyboard brake/reverse
            Input.GetKey(KeyCode.RightArrow) || // Keyboard right
            Input.GetKey(KeyCode.Space); // Keyboard brake

        if (manualInputDetected && isAutomaticMode)
        {
            // Switch from automatic mode to manual mode
            isAutomaticMode = false;
            isFollowingWaypoints = false;

            // Update the automatic mode indicator
            UpdateAutomaticModeIndicator();

            // Play sound notification for deactivating automatic mode
            if (audioSource != null && automaticModeOffSound != null)
            {
                audioSource.PlayOneShot(automaticModeOffSound);
            }

            // Update current speed based on actual Rigidbody velocity
            if (rb != null)
            {
                currentSpeed = rb.velocity.magnitude; // Speed in meters per second
            }

            Debug.Log("Manual input detected. Switching to manual mode.");
        }

        if (isFollowingWaypoints) // If automatic mode is active, move towards the waypoint
        {
            MoveTowardsWaypoint();
        }
        else if (!isAutomaticMode)
        {
            // Manual control logic
            GetInput();      // Get user input for steering, acceleration, and braking
            HandleMotor();   // Apply motor and braking forces based on input
            HandleSteering(); // Handle steering based on input
            //UpdateWheels();  // Update wheel visuals to match physics (if used)
        }

        // Update the speed display at fixed intervals
        timeSinceLastSpeedUpdate += Time.deltaTime;
        if (timeSinceLastSpeedUpdate >= speedUpdateInterval)
        {
            UpdateSpeedDisplay();
            timeSinceLastSpeedUpdate = 0f;
        }
    }

    //----------------------------------------------------------------------------------------------------------------
    // METHODS -------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Method to handle gear selection input
    private void HandleGearInput()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            SetGear(Gear.Park);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            SetGear(Gear.Reverse);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            SetGear(Gear.Neutral);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            SetGear(Gear.Drive);
        }
    }

    // Method to set the current gear
    private void SetGear(Gear newGear)
    {
        // If changing from Drive to another gear while in automatic mode, disable automatic mode
        if (currentGear == Gear.Drive && newGear != Gear.Drive && isAutomaticMode)
        {
            isAutomaticMode = false;
            isFollowingWaypoints = false;
            Debug.Log("Automatic mode disabled because gear changed out of Drive.");

            // Update the automatic mode indicator
            UpdateAutomaticModeIndicator();

            // Play sound notification for deactivating automatic mode
            if (audioSource != null && automaticModeOffSound != null)
            {
                audioSource.PlayOneShot(automaticModeOffSound);
            }
        }

        currentGear = newGear;

        // Update handbrake status
        isHandbrakeEngaged = (currentGear == Gear.Park);

        // Update gear indicators
        UpdateGearIndicators();

        // Optional: Log the gear change
        Debug.Log("Gear changed to: " + currentGear);
    }

    // Method to update gear indicators based on the current gear
    private void UpdateGearIndicators()
    {
        // Ensure all indicators are assigned
        if (parkIndicator == null || reverseIndicator == null || neutralIndicator == null || driveIndicator == null)
        {
            Debug.LogWarning("One or more gear indicator spheres are not assigned in the CarController.");
            return;
        }

        // Hide all indicators
        parkIndicator.SetActive(false);
        reverseIndicator.SetActive(false);
        neutralIndicator.SetActive(false);
        driveIndicator.SetActive(false);

        // Show the indicator corresponding to the current gear
        switch (currentGear)
        {
            case Gear.Park:
                parkIndicator.SetActive(true);
                break;
            case Gear.Reverse:
                reverseIndicator.SetActive(true);
                break;
            case Gear.Neutral:
                neutralIndicator.SetActive(true);
                break;
            case Gear.Drive:
                driveIndicator.SetActive(true);
                break;
        }
    }

    // Method to update the automatic mode indicator
    private void UpdateAutomaticModeIndicator()
    {
        if (automaticModeIndicator != null)
        {
            automaticModeIndicator.SetActive(isAutomaticMode);
        }
        else
        {
            Debug.LogWarning("Automatic mode indicator is not assigned in the CarController.");
        }
    }

    // Method to update the ego target speed display
    private void UpdateEgoTargetSpeedDisplay()
    {
        if (targetSpeedText != null)
        {
            float egoTargetSpeedInMPH = egoTargetSpeed * 0.621371f;
            int roundedEgoTargetSpeed = Mathf.RoundToInt(egoTargetSpeedInMPH);
            targetSpeedText.text = roundedEgoTargetSpeed.ToString();
        }
    }

    // Method to update the autopilot speed display
    private void UpdateAutopilotSpeedDisplay()
    {
        if (autopilotSpeedText != null)
        {
            float autopilotSpeedInMPH = egoTargetSpeed * 0.621371f;
            int roundedAutopilotSpeed = Mathf.RoundToInt(autopilotSpeedInMPH);
            autopilotSpeedText.text = roundedAutopilotSpeed.ToString();
        }
    }

    // Method to update the speed display
    private void UpdateSpeedDisplay()
    {
        float actualSpeedInMilesPerHour = 0f;

        // Use Rigidbody velocity for speed calculation
        if (rb != null)
        {
            float speedInMetersPerSecond = rb.velocity.magnitude; // Get speed in m/s
            actualSpeedInMilesPerHour = speedInMetersPerSecond * 2.23694f; // Convert to mph
        }

        // Determine if the vehicle is accelerating or decelerating
        bool isAccelerating = actualSpeedInMilesPerHour >= previousActualSpeed;

        // Adjust the smoothing factor based on acceleration or deceleration
        float smoothingFactor = isAccelerating ? 0.1f : 0.4f; // Use a higher factor when decelerating

        // Smooth the speed value
        displayedSpeed = Mathf.Lerp(displayedSpeed, actualSpeedInMilesPerHour, smoothingFactor);

        // Force the displayed speed to zero if actual speed is near zero
        if (actualSpeedInMilesPerHour < 0.1f)
        {
            displayedSpeed = 0f;
        }

        // Update the speed display with rounded speed
        if (speedText != null)
        {
            int roundedSpeed = Mathf.RoundToInt(displayedSpeed);
            speedText.text = roundedSpeed.ToString();
        }

        // Update previous actual speed
        previousActualSpeed = actualSpeedInMilesPerHour;
    }

    // Car movement system -------------------------------------------------------------------------------------------
    // Method for getting user input for controlling the car
    private void GetInput()
    {
        // Steering input: Joystick and keyboard separately
        float joystickSteering = Input.GetAxis("Steering"); // Joystick input
        float keyboardSteering = Input.GetAxis("Horizontal Keyboard"); // Keyboard input
        horizontalInput = Mathf.Clamp(joystickSteering + keyboardSteering, -1f, 1f);

        // Accelerator input: Joystick and keyboard separately
        float rawJoystickAccelerator = Input.GetAxis("Accelerator");
        float normalizedJoystickAccelerator = Mathf.Clamp((rawJoystickAccelerator + 1) / 2, 0f, 1f);
        float keyboardAccelerator = Mathf.Clamp(Input.GetAxis("Vertical Keyboard"), 0f, 1f); // Forward only
        verticalInput = Mathf.Clamp(normalizedJoystickAccelerator + keyboardAccelerator, 0f, 1f);

        // Brake input
        float rawJoystickBrake = Input.GetAxis("Brake");
        float normalizedJoystickBrake = Mathf.Clamp((rawJoystickBrake + 1) / 2, 0f, 1f);
        float keyboardBrake = Mathf.Clamp(-Input.GetAxis("Vertical Keyboard"), 0f, 1f); // Reverse/brake only
        float combinedBrakeInput = Mathf.Clamp(normalizedJoystickBrake + keyboardBrake, 0f, 1f);

        isBraking = combinedBrakeInput > 0.1f;
        currentBrakeForce = combinedBrakeInput * brakeForce;

        // Debug logs
        Debug.Log($"Steering: {horizontalInput}");
        Debug.Log($"Accelerator: {verticalInput}");
        Debug.Log($"Brake: {currentBrakeForce}");
    }

    // Method for handling car acceleration and braking
    private void HandleMotor()
    {
        if (isBraking)
        {
            // If braking, apply brake force and stop motor torque
            frontLeftWheelCollider.motorTorque = 0f;
            frontRightWheelCollider.motorTorque = 0f;
            ApplyBraking(); // Apply braking force
        }
        else
        {
            // Handle Drive and Reverse gears separately
            if (currentGear == Gear.Drive)
            {
                // Apply positive motor torque for forward motion
                frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = verticalInput * motorForce;
            }
            else if (currentGear == Gear.Reverse)
            {
                // Apply negative motor torque for reverse motion
                frontLeftWheelCollider.motorTorque = -verticalInput * motorForce;
                frontRightWheelCollider.motorTorque = -verticalInput * motorForce;
            }
            else
            {
                // No motor torque for other gears
                frontLeftWheelCollider.motorTorque = 0f;
                frontRightWheelCollider.motorTorque = 0f;
            }

            // Release brakes when not braking
            ApplyBraking(0f);
        }
    }

    // Method for applying the braking force to all four wheels
    private void ApplyBraking(float brakeForceOverride = -1f)
    {
        // If brakeForceOverride is not provided, use currentBrakeForce
        float appliedBrakeForce = brakeForceOverride >= 0 ? brakeForceOverride : currentBrakeForce;

        frontRightWheelCollider.brakeTorque = appliedBrakeForce;
        frontLeftWheelCollider.brakeTorque = appliedBrakeForce;
        rearLeftWheelCollider.brakeTorque = appliedBrakeForce;
        rearRightWheelCollider.brakeTorque = appliedBrakeForce;
    }

    // Method for handling the car's steering
    private void HandleSteering()
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    // Method to find the closest waypoint ahead of the car's current position
    private Transform FindClosestWaypoint()
    {
        Transform closestWaypoint = null;
        float closestDistance = Mathf.Infinity;

        // Loop through all waypoints (children of the Waypoints object) and find the closest one ahead
        for (int i = 0; i < egoWaypoints.transform.childCount; i++)
        {
            Transform waypoint = egoWaypoints.transform.GetChild(i);
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

    // Waypoint system -----------------------------------------------------------------------------------------------
    // Method to move towards the current waypoint
    void MoveTowardsWaypoint()
    {
        if (currentWaypoint == null) return;

        // Only move if in Drive
        if (currentGear != Gear.Drive)
        {
            // Stop the vehicle
            rb.velocity = Vector3.zero;
            return;
        }

        float egoTargetSpeedInMetersPerSecond = egoTargetSpeed * 0.27778f;  // Conversion from kph to m/s
        float distanceToWaypoint = Vector3.Distance(transform.position, currentWaypoint.position);
        Vector3 directionToWaypoint = (currentWaypoint.position - transform.position).normalized;

        if (Vector3.Dot(transform.forward, directionToWaypoint) > 0)
        {
            if (distanceToWaypoint < egoWaypointThreshold + 0.1f)
            {
                currentWaypoint = egoWaypoints.GetNextWaypoint(currentWaypoint);
            }

            // Convert acceleration from g to m/s² (1g = 9.81 m/s²)
            float accelerationInMetersPerSecondSquared = egoAccelerationRate * 9.81f;

            // Gradually accelerate or decelerate the car towards the waypoint's speed
            if (isAutomaticMode)
            {
                currentSpeed = Mathf.MoveTowards(currentSpeed, egoTargetSpeedInMetersPerSecond, accelerationInMetersPerSecondSquared * Time.deltaTime);
            }

            Vector3 movement = directionToWaypoint * currentSpeed;
            rb.velocity = movement;

            // Rotate towards the waypoint
            Quaternion targetRotation = Quaternion.LookRotation(directionToWaypoint);
            Quaternion newRotation = Quaternion.Slerp(transform.rotation, targetRotation, egoTurnSpeed * Time.deltaTime);
            rb.MoveRotation(newRotation);
        }
    }

    // Collision detection system -----------------------------------------------------------------------------------------------
    // Method to track collisions
    private void OnCollisionEnter(Collision collision)
    {
        collidedObjectName = collision.gameObject.name;
        hasCollisionOccurred = true;
    }

    // Method to reset collision data after logging
    public void ResetCollisionData()
    {
        collidedObjectName = "None";
        hasCollisionOccurred = false;
    }
}