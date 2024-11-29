// NPCSpawner.cs
// Script written by Maegan L. Schmitz in 2024
// Updated to notify DataLogger when NPCs are spawned and provide NPC settings

using UnityEngine;

public class NPCSpawner : MonoBehaviour
{
    [Header("NPC Input Settings")]
    [Tooltip("Drag objects over from prefabs or hierarchy.")]
    public GameObject npcCarPrefab;
    public Transform npcSpawnPoint;
    private bool npcHasSpawned = false;

    // Inspector variables for waypoint system
    [Header("NPC Waypoint Settings")]
    [Tooltip("Target speed in kilometers per hour (kph). Determines the speed of the vehicle while moving between waypoints.")]
    public float npcTargetSpeed = 40f;  // NPC speed in kph
    [Tooltip("Acceleration rate in g. Determines how quickly the NPC car accelerates.")]
    public float npcAccelerationRate = 0.2f;  // Default acceleration rate in g
    [Tooltip("Turn speed factor (unitless). Higher values result in faster turning toward waypoints.")]
    public float npcTurnSpeed = 1.5f;  // Default turn speed
    [Tooltip("Distance threshold in meters (m) to determine if the car is close enough to the waypoint to proceed to the next.")]
    public float npcWaypointThreshold = 5f;  // Waypoint threshold
    public Waypoints npcWaypoints;

    // Inspector variables for cutoff
    [Header("NPC Cutoff Settings")]
    [Tooltip("Enable or disable cutoff behavior for NPC vehicles.")]
    public bool enableCutoff = true;  // Toggle to enable or disable cutoff
    [Tooltip("Distance in front of the Ego Vehicle in meters (m) the NPC must be to begin cutoff.")]
    public float cutoffRange = 10f;
    [Tooltip("Direction of the cutoff.")]
    public Vector3 cutoffDirection = new Vector3(-1, 0, 0);
    [Tooltip("Maximum lateral distance in meters (m) the NPC Vehicle will move in a cutoff.")]
    public float cutoffLateralDistance = 8f;
    [Tooltip("Rate NPC Vehicle will decelerate in g (1g = 9.81 m/s²) after maximum lateral distance reached.")]
    public float cutoffDecelerationRate = 1.02f;  // Deceleration rate in g
    public GameObject egoVehicle;

    // Event to notify when an NPC is spawned
    public static event System.Action<GameObject> OnNPCSpawned;

    private void OnTriggerEnter(Collider other)
    {
        if (!npcHasSpawned && other.CompareTag("Ego Vehicle"))
        {
            // Spawn the NPC vehicle at the designated spawn point
            GameObject npcCar = Instantiate(npcCarPrefab, npcSpawnPoint.position, npcSpawnPoint.rotation);

            // Get the NPCCarController component
            NPCCarController npcController = npcCar.GetComponent<NPCCarController>();
            if (npcController != null)
            {
                npcController.SetWaypoints(npcWaypoints);
                npcController.SetEgoVehicle(egoVehicle);

                // Pass the speed, turn speed, waypoint threshold, and acceleration rate (converted to m/s²)
                npcController.SetNPCSettings(npcTargetSpeed, npcTurnSpeed, npcWaypointThreshold, npcAccelerationRate * 9.81f);

                // Pass the cutoff settings only if cutoff is enabled
                if (enableCutoff)
                {
                    // Convert deceleration rate from g to m/s² before passing it
                    float cutoffDecelerationRateInMs2 = cutoffDecelerationRate * 9.81f;
                    npcController.SetCutoffSettings(
                        cutoffRange,
                        cutoffDirection,
                        cutoffLateralDistance,
                        cutoffDecelerationRateInMs2
                    );
                }
            }

            // Notify the DataLogger that an NPC has been spawned
            OnNPCSpawned?.Invoke(npcCar);

            npcHasSpawned = true;
        }
    }
}