// DataLogger.cs
// Script written by Maegan L. Schmitz in 2024

using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class DataLogger : MonoBehaviour
{
    [Header("Vehicles to Track")]
    public GameObject egoVehicle; // The main vehicle being controlled.

    private List<GameObject> npcVehicles = new List<GameObject>(); // List of NPC vehicles
    private List<List<string>> csvData = new List<List<string>>(); // Stores rows of CSV data
    private List<string> csvHeader = new List<string>(); // Stores CSV header columns

    private float loggingInterval = 0.1f; // Time interval between data logs in seconds
    private float timeSinceLastLog = 0f; // Tracks time since the last log
    private bool isRecording = false; // Determines if data logging is currently active
    private string filePath; // File path for saving the CSV file

    private Dictionary<GameObject, Vector3> previousVelocities = new Dictionary<GameObject, Vector3>(); // Tracks previous velocities

    private Dictionary<KeyCode, float> keyPressStartTimes = new Dictionary<KeyCode, float>(); // Tracks key press start times
    private List<KeyCode> keysToCheck = new List<KeyCode> {
        KeyCode.W, KeyCode.A, KeyCode.S, KeyCode.D,
        KeyCode.UpArrow, KeyCode.LeftArrow, KeyCode.DownArrow, KeyCode.RightArrow, KeyCode.Space
    };

    private CarController egoController; // Reference to the ego vehicle's controller script

    void Start()
    {
        // Set the file path with timestamp to avoid overwriting previous logs
        filePath = Path.Combine(Application.dataPath, $"VehicleData_{System.DateTime.Now:yyyy-MM-dd_HH-mm-ss}.csv");

        // Initialize the velocity tracking for the ego vehicle
        if (egoVehicle != null)
        {
            previousVelocities[egoVehicle] = Vector3.zero;

            // Get the CarController component
            egoController = egoVehicle.GetComponent<CarController>();
            if (egoController == null)
            {
                Debug.LogWarning("CarController component not found on the ego vehicle.");
            }
        }
    }

    void OnEnable()
    {
        NPCSpawner.OnNPCSpawned += AddNPCVehicle; // Subscribe to NPC spawn event
    }

    void OnDisable()
    {
        NPCSpawner.OnNPCSpawned -= AddNPCVehicle; // Unsubscribe from NPC spawn event
    }

    void Update()
    {
        // Toggle recording on or off when the 'P' key is pressed
        if (Input.GetKeyDown(KeyCode.P))
        {
            isRecording = !isRecording;
            if (isRecording)
                StartRecording(); // Start logging if recording is enabled
            else
                StopRecording(); // Stop logging if recording is disabled
        }

        // If recording is active, log data at the specified interval
        if (isRecording)
        {
            timeSinceLastLog += Time.deltaTime; // Increment the time since the last log
            if (timeSinceLastLog >= loggingInterval)
            {
                LogData(); // Log vehicle data
                timeSinceLastLog = 0f; // Reset the timer
            }
        }

        TrackKeyPresses(); // Check for key presses
    }

    void TrackKeyPresses()
    {
        // Update key press start times and durations
        foreach (var key in keysToCheck)
        {
            if (Input.GetKeyDown(key))
            {
                if (!keyPressStartTimes.ContainsKey(key))
                    keyPressStartTimes[key] = Time.time;
            }

            if (Input.GetKeyUp(key))
            {
                if (keyPressStartTimes.ContainsKey(key))
                    keyPressStartTimes.Remove(key);
            }
        }
    }

    void StartRecording()
    {
        Debug.Log("Data recording started.");

        // Initialize the header list
        csvHeader = new List<string> { "Date", "Time", "TimeInSim" };

        // Add headers for key presses
        for (int i = 1; i <= 5; i++)
        {
            csvHeader.Add($"KeyPress{i}");
            csvHeader.Add($"KeyPressDuration{i}");
        }

        // Add headers for ego vehicle data
        if (egoVehicle != null)
        {
            csvHeader.AddRange(new string[] {
                "EgoPosX_m", "EgoPosY_m", "EgoPosZ_m",
                "EgoSpeed_kmh", "EgoAccelX_g", "EgoAccelY_g", "EgoAccelZ_g",
                "EgoHeading_deg",
                "EgoMode",            // Column for the vehicle's current mode
                "EgoCollision"        // New column for collision information
            });
        }

        // Initialize the csvData list with the header
        csvData.Clear();
        csvData.Add(new List<string>(csvHeader));
    }

    void StopRecording()
    {
        Debug.Log("Data recording stopped.");

        // Construct CSV lines from the data rows
        List<string> csvLines = new List<string>();

        // Join the header columns
        csvLines.Add(string.Join(",", csvHeader));

        // Join each data row
        for (int i = 1; i < csvData.Count; i++) // Start from 1 to skip the header row in csvData
        {
            var row = csvData[i];

            // Ensure the row has the same number of columns as the header
            while (row.Count < csvHeader.Count)
            {
                row.Add("0");
            }

            csvLines.Add(string.Join(",", row));
        }

        // Write to the CSV file
        File.WriteAllLines(filePath, csvLines.ToArray());
        Debug.Log("Data logged to " + filePath);
    }

    void LogData()
    {
        // Create a new data row
        List<string> dataRow = new List<string>
        {
            System.DateTime.Now.ToString("MM/dd/yyyy"),
            System.DateTime.Now.ToString("HH:mm:ss"),
            Time.time.ToString("F2")
        };

        // Add key press data
        int keyPressCount = 0;
        foreach (var keyTimePair in keyPressStartTimes)
        {
            if (keyPressCount >= 5) break; // Limit to 5 key presses
            KeyCode key = keyTimePair.Key;
            float duration = Time.time - keyTimePair.Value;
            dataRow.Add(key.ToString());
            dataRow.Add(duration.ToString("F2"));
            keyPressCount++;
        }
        // Fill remaining key press slots
        for (int i = keyPressCount; i < 5; i++)
        {
            dataRow.Add("None");
            dataRow.Add("0");
        }

        // Add ego vehicle data
        if (egoVehicle != null)
        {
            AppendVehicleData(dataRow, egoVehicle, true);

            // Add collision data
            if (egoController != null && egoController.HasCollisionOccurred)
            {
                dataRow.Add(egoController.CollidedObjectName);

                // Reset collision data after logging
                egoController.ResetCollisionData();
            }
            else
            {
                dataRow.Add("None"); // No collision occurred
            }
        }
        else
        {
            // Add placeholders for the ego vehicle data, including mode and collision
            dataRow.AddRange(new string[] { "0", "0", "0", "0", "0", "0", "0", "0", "Unknown", "None" });
        }

        // Add data for existing NPCs
        foreach (GameObject npc in npcVehicles)
        {
            if (npc != null)
            {
                AppendVehicleData(dataRow, npc, false);
            }
        }

        // Ensure data row matches header length
        while (dataRow.Count < csvHeader.Count)
        {
            dataRow.Add("0");
        }

        // Add the data row to the CSV data
        csvData.Add(dataRow);
    }

    void AppendVehicleData(List<string> dataRow, GameObject vehicle, bool isEgo)
    {
        // Get the vehicle's position in world space
        Vector3 position = vehicle.transform.position;

        // Get the vehicle's Rigidbody component to access velocity
        Rigidbody rb = vehicle.GetComponent<Rigidbody>();

        if (rb != null)
        {
            Vector3 currentVelocity = rb.velocity;
            float speed = currentVelocity.magnitude * 3.6f; // Speed in km/h

            // Calculate acceleration in m/s²
            Vector3 previousVelocity = previousVelocities.ContainsKey(vehicle) ? previousVelocities[vehicle] : Vector3.zero;
            Vector3 acceleration = (currentVelocity - previousVelocity) / loggingInterval;
            previousVelocities[vehicle] = currentVelocity;

            // Convert acceleration to g-force
            Vector3 accelerationInGs = acceleration / 9.80665f;

            // Calculate vehicle heading in degrees
            Vector3 forward = vehicle.transform.forward;
            forward.y = 0f; // Project onto horizontal plane
            if (forward.sqrMagnitude > 0f)
                forward.Normalize();
            else
                forward = Vector3.forward; // Default direction if forward vector is zero

            float headingRadians = Mathf.Atan2(forward.x, forward.z);
            float headingDegrees = headingRadians * Mathf.Rad2Deg;
            if (headingDegrees < 0f)
                headingDegrees += 360f;

            // Append vehicle data to the data row
            dataRow.AddRange(new string[] {
                position.x.ToString("F6"), position.y.ToString("F6"), position.z.ToString("F6"),
                speed.ToString("F6"),
                accelerationInGs.x.ToString("F6"), accelerationInGs.y.ToString("F6"), accelerationInGs.z.ToString("F6"),
                headingDegrees.ToString("F6")
            });

            if (isEgo)
            {
                // Get the current mode
                string egoMode = "Unknown";

                if (egoController != null)
                {
                    // Determine the mode based on 'IsAutomaticMode'
                    egoMode = egoController.IsAutomaticMode ? "Automatic" : "Manual";
                }
                else
                {
                    Debug.LogWarning("CarController is null. Cannot retrieve ego vehicle mode.");
                }

                dataRow.Add(egoMode); // Add ego vehicle mode
                // Collision data is handled in LogData()
            }
            else
            {
                // For NPC vehicles, get the NPC's current mode
                NPCCarController npcController = vehicle.GetComponent<NPCCarController>();
                if (npcController != null)
                {
                    string npcMode = npcController.CurrentMode;
                    dataRow.Add(npcMode); // Add NPC mode
                }
                else
                {
                    Debug.LogWarning("NPCCarController is null. Cannot retrieve NPC mode.");
                    dataRow.Add("Unknown"); // Default value if controller is missing
                }
            }
        }
        else
        {
            Debug.LogWarning("No Rigidbody attached to " + vehicle.name);
            dataRow.AddRange(new string[] { "0", "0", "0", "0", "0", "0", "0", "0" });

            if (isEgo)
            {
                dataRow.Add("Unknown"); // EgoMode
                // Collision data is handled in LogData()
            }
            else
            {
                dataRow.Add("Unknown"); // NPC Mode
            }
        }
    }

    public void AddNPCVehicle(GameObject npc)
    {
        // Add the NPC to the tracking list
        if (!npcVehicles.Contains(npc))
        {
            npcVehicles.Add(npc);
            previousVelocities[npc] = Vector3.zero;

            // Update the CSV header and data rows
            UpdateCSVHeaderForNewNPC(npc);
        }
    }

    void UpdateCSVHeaderForNewNPC(GameObject npc)
    {
        int npcIndex = npcVehicles.IndexOf(npc) + 1; // +1 to start from NPC1

        // Add new columns to the header with units
        csvHeader.AddRange(new string[] {
            $"NPC{npcIndex}_PosX_m", $"NPC{npcIndex}_PosY_m", $"NPC{npcIndex}_PosZ_m",
            $"NPC{npcIndex}_Speed_kmh", $"NPC{npcIndex}_AccelX_g", $"NPC{npcIndex}_AccelY_g", $"NPC{npcIndex}_AccelZ_g",
            $"NPC{npcIndex}_Heading_deg",
            $"NPC{npcIndex}_Mode" // Single column for NPC mode
        });

        // Add placeholders to existing data rows
        foreach (var row in csvData)
        {
            if (row == csvHeader) continue;
            row.AddRange(new string[] { "0", "0", "0", "0", "0", "0", "0", "0", "Normal" }); // Default mode is "Normal"
        }
    }
}