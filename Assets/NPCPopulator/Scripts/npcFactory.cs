using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[ExecuteInEditMode]
public class npcFactory : MonoBehaviour {

    public List<GameObject> npcModels = new List<GameObject>();
    public RuntimeAnimatorController animatorController;
    public GameObject waypointParent;
    public Bounds navMeshBounds;
    public float navMeshRange;
    public int navMeshMask = NavMesh.AllAreas;
    public Transform npcParent;               // Parent game object under which to put factory-created NPCs.
    public float usualSpeed = 1.0f;           // Default usual speed.
    public float usualSpeedVariation = 0.2f;  // Amount plus/minus that speed can vary from usual speed.
    public float agentStoppingDistance = 0.75f;
    public float npcRadius = 0.5f;
    public float stoppingDistance = 1.0f;
    public int minFramesToWait;               // Frames to wait at a waypoint.
    public int maxFramesToWait = 30;
    public float range = 1000f;               // Distance for finding waypoints.
    public bool limitRange = true;
    public int numberToCreate = 2;
    public bool disableLights = true;
    
    public void AddNpcModel()
    {
        // Add a new index position to the end of our list
        npcModels.Add(null);
    }

    public void RemoveNpcModel(int index)
    {
        //Remove an index position from our list at a point in our list array
        npcModels.RemoveAt(index);
    }

    /*
     * The bounds need to be refreshed when the NavMesh is rebuilt.
     * A button is in the Editor for that purpose.
     */
    public Bounds CalculateBoundsOfNavMesh()
    {
        /*
         * The value below is close to the largest value accepted by NavMesh.SamplePosition().  The function
         * seems to break with values around 1E10.  This implies that the value may internally be a 32 bit
         * unsigned int.
         * This value should be much larger than the dimensions of pretty much any Unity scene.
         * If performance becomes an issue, this value could be reduced as long as it is larger than the
         * dimensions of the scene.
         */
        float maxDistance     = 1E8f; 
        float maxDistanceHalf = maxDistance/2f;   
        navMeshBounds = new Bounds();    // Reset navMeshBounds in case there was an old version.
        NavMeshHit hitLeft, hitRight, hitForward, hitBack;
        Bounds tempBounds = new Bounds(Vector3.zero, Vector3.zero);

        // Get the extremes in all four directions.
        if (NavMesh.SamplePosition(Vector3.left * maxDistanceHalf, out hitLeft, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.back * maxDistanceHalf, out hitBack, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.right * maxDistanceHalf, out hitRight, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.forward * maxDistanceHalf, out hitForward, maxDistance, NavMesh.AllAreas))
        {
            /*
             * The way y is calculated below is not perfect, but the variation in y is expected to be small compared to the
             * variation in x and z.  The call to SamplePosition() in RandomPointOnNavMesh() will return a point that is on
             * the NavMesh, including the correct value of y.
             */ 
            Vector3 newMin = new Vector3(hitLeft.position.x, Mathf.Min(hitLeft.position.y,hitBack.position.y), hitBack.position.z);
            Vector3 newMax = new Vector3(hitRight.position.x, Mathf.Max(hitRight.position.y,hitForward.position.y), hitForward.position.z);
            tempBounds.SetMinMax(newMin, newMax);
        }
        else
        {
            Debug.LogError("Error getting NavMesh bounds.");
        }
        return tempBounds;
    }

    public Bounds ClearNavMeshBounds()
    {
        return new Bounds(Vector3.zero, Vector3.zero);
    }

    public void RefreshRanges()
    {
        // navMeshRange is the length of the diagonal of navMeshBounds in the xz plane.
        navMeshRange = 2f*Mathf.Sqrt(navMeshBounds.extents.x * navMeshBounds.extents.x + navMeshBounds.extents.z * navMeshBounds.extents.z);
        range = Mathf.Round(navMeshRange / 2f); // By default, set the range to half the range of the NavMesh.
    }

    private bool OccupiedPoint(Vector3 candidatePos)
    {
        bool isOccupied = false;
        
        // Iterate through all the NPCs generated so far to see if any of them occupy the same point.
        foreach (Transform child in npcParent)
        {
            // Compare the distance.  Make sure child is at least 2 * radius away from candidatePos.
            if (Vector3.Distance(child.position, candidatePos) < 2 * npcRadius)
            {
                isOccupied = true;
                break;
            }
        }
        return isOccupied;
    }

    private Vector3 RandomPointOnNavMesh()
    {
        Vector3 thePoint = Vector3.zero;
            
        /*
         * Ideally we could automatically make sure that navMeshBounds is up-to-date.  However, there may not be
         * an automatic way to recalculate navMeshBounds when the NavMesh is baked.
         * We can do a simple check to at least make sure that the bounds are not zero.  This does not cover the
         * case that the NavMesh has been re-baked and changed.
         */
        if (Mathf.Approximately(navMeshBounds.extents.magnitude, 0f))
        {
            Debug.LogError("ERROR:  Bounds on NavMesh is zero.");
        }
        else
        {
            // Calculate a random point with the extend of navMeshBounds in xz.
            Vector3 sourcePosition = new Vector3(Random.Range(navMeshBounds.min.x, navMeshBounds.max.x), 0f, Random.Range(navMeshBounds.min.z, navMeshBounds.max.z));
            NavMeshHit npcHit;
            if (NavMesh.SamplePosition(sourcePosition, out npcHit, navMeshRange, navMeshMask))
            {
                // Return the point.
                if (npcHit.hit)
                    thePoint = npcHit.position;
                else
                    Debug.LogError("RandomPointOnNavMesh() had an empty hit.");
            }
            else
            {
                // This should not fail, unless we (later) add a check for the location already being occupied.
                Debug.LogError("ERROR: RandomPointOnNavMesh() was not able to return a position.");
            }
        }
        return thePoint;
    }

    private Vector3 RandomUnoccupiedPointOnNavMesh()
    {
        int maxTries = 10;
        Vector3 thePoint = Vector3.positiveInfinity;

        for (int i = 0; i < maxTries; i++)
        {
            thePoint = RandomPointOnNavMesh();
            if (!OccupiedPoint(thePoint))
            {
                break;
            }
        }
        /*
         * thePoint will either be unoccupied, or the last one tried, so there is a small chance or returning
         * an occupied point.
         */ 
        return thePoint;
    }
    public GameObject CreateNpc()
    {
        GameObject newNpc;

        Vector3 pos = RandomUnoccupiedPointOnNavMesh();
        int randomModel = Random.Range(0, npcModels.Count);
        Quaternion rot = Quaternion.Euler(0f, 360f * Random.value, 0f);
        if (npcParent)
            newNpc = Instantiate(npcModels[randomModel], pos, rot, npcParent);
        else
            newNpc = Instantiate(npcModels[randomModel], pos, rot);

        // Add an npcIndividual script to the NPC.  That in turn pulls in the NavMeshAgent component.
        newNpc.AddComponent<npcIndividual>();
        npcIndividual indiv = newNpc.GetComponent<npcIndividual>();
        
        // Set animator controller.
        Animator indivAnim = indiv.GetComponent<Animator>();
        indivAnim.runtimeAnimatorController = animatorController;
        
        // Set the waypoint parent.
        indiv.waypointParent = waypointParent;
        
        // Set the frames to wait at a waypoint.
        indiv.minFramesToWait = minFramesToWait;
        indiv.maxFramesToWait = maxFramesToWait;
        
        // Set the waypoint range parameters.
        indiv.range = range;
        indiv.limitRange = limitRange;
        
        // Set the stopping distance.
        indiv.stoppingDistance = stoppingDistance;

        // Assign a random avoidance priority.
        NavMeshAgent nma = newNpc.GetComponent<NavMeshAgent>();
        if (nma == null)
            Debug.LogError("New NPC does not have a NavMeshAgent.");
        else
        {
            nma.avoidancePriority = Mathf.FloorToInt(Random.Range(0f, 99f));
            // Set stopping distance.
            nma.stoppingDistance = agentStoppingDistance;
            // Set the radius in case the user wants something different from the default value of 0.5;
            nma.radius = npcRadius;
        }
        
        // Set the usual speed in the states script, with a small variation.
        indiv.usualSpeed = usualSpeed + Random.Range(-usualSpeedVariation, usualSpeedVariation);
        
        // Some third-party characters come with built-in lights.  Disable these.
        if (disableLights)
        {
            Transform lightsTransform = newNpc.transform.Find("Lights");
            if (lightsTransform)
            {
                GameObject lightsObject = lightsTransform.gameObject;
                lightsObject.SetActive(false);
            }
        }

        return newNpc;
    }


}
