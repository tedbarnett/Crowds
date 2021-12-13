using UnityEngine;

// Small script for drawing a gizmo on the waypoint in the Scene view.
public class npcWaypointScript : MonoBehaviour {

    public float gizmoRadius = 0.3F;

    void OnDrawGizmos()
    {
        Vector3 cubeDims = new Vector3(gizmoRadius, gizmoRadius, gizmoRadius);
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, gizmoRadius);
        Gizmos.color = Color.red;
        Gizmos.DrawCube(transform.position, cubeDims);
    }

}
