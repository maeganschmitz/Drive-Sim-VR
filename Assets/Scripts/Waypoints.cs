// Waypoints.cs
// Simplified script without loop and direction options
// Script written by Maegan L. Schmitz in 2024

using UnityEngine;

public class Waypoints : MonoBehaviour
{
    //----------------------------------------------------------------------------------------------------------------
    // VARIABLES -----------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    [Header("Gizmo Settings")] // Adds header for color customization
    [SerializeField] private Color waypointColor = Color.blue; // Color for waypoint spheres
    [SerializeField] private Color pathColor = Color.red;     // Color for path lines
    [Range(0f, 2f)]
    [SerializeField] private float waypointSize = 1f;

    //----------------------------------------------------------------------------------------------------------------
    // METHODS -------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------

    // Draw Waypoint spheres
    private void OnDrawGizmos()
    {
        Gizmos.color = waypointColor; // Use the color set in the Inspector
        foreach (Transform t in transform)
        {
            Gizmos.DrawWireSphere(t.position, waypointSize);
        }

        // Draw lines connecting waypoints
        Gizmos.color = pathColor; // Use the color set in the Inspector
        for (int i = 0; i < transform.childCount - 1; i++)
        {
            Gizmos.DrawLine(transform.GetChild(i).position, transform.GetChild(i + 1).position);
        }
    }

    // Get the next waypoint in sequence without looping or direction reversal
    public Transform GetNextWaypoint(Transform currentWaypoint)
    {
        if (currentWaypoint == null)
        {
            // Start at the first waypoint if none is set
            return transform.GetChild(0);
        }

        int currentIndex = currentWaypoint.GetSiblingIndex();
        int nextIndex = currentIndex + 1;

        if (nextIndex < transform.childCount)
        {
            // Return the next waypoint in the sequence
            return transform.GetChild(nextIndex);
        }
        else
        {
            // No more waypoints; return null
            return null;
        }
    }
}