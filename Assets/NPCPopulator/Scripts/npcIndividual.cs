using System;  // For timers
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Animator))]
[RequireComponent(typeof(NavMeshAgent))]
public class npcIndividual : MonoBehaviour {
    
    // Waypoint variables
    public GameObject waypointParent; // Waypoints should be under a common parent in the Scene.
    public bool limitRange;           // Whether to prefer close waypoints.
    public float range = 1000f;       // Distance for finding waypoints.
    public int minFramesToWait = 5;
    public int maxFramesToWait = 30;

    // Configurable speed values.
    public float usualSpeed = 1f;

    // The other variables are mostly private and used for internal calculations or used by the factory.
    private Animator anim;
    private NavMeshAgent agent;

    // Parameters related to synchronization between navigation and animation.
    // These are not public so that the user does not get overwhelmed with parameters that usually don't need to be adjusted.
    private const float MoveThreshold = 0.05f; // For speeds above this, the NPC will move rather than idle.
    private const float SyncThreshold = 0.05f; // Multiplier for agent radius, offset greater than this between agent and animator will pull the animator to the agent.
    private const float SyncDamping = 0.1f;    // Parameter that controls the rate of pulling the animator to the agent.
    private const float SmoothTime = 0.1f;     // For calls to SmoothDamp() and SmoothDampAngle().
    private float currentVelocity;
    private float agentSpeed;

    // Variables related to the NavMeshAgent.
    private Vector3 currentPosition;
    private Vector3 nextPosition;
    private Vector3 deltaPosition;
    private Vector3 currentForward;
    private Vector3 nextForward;
    private float prevDeltaForward;
    private float currDeltaForward;
    private Vector2 agentVelocity;
    private float dxAgent, dyAgent;
    private float vxAgent, vyAgent;
    private Vector2 nextPosition2D;
    public float stoppingDistance = 0.75f; // Needs to be > 0.

    // Variables related to the Animator.
    private Vector3 currAnimPosition;
    private Vector2 currAnimPosition2D;

    // Variable related to the offset between the NavMeshAgent and the Animator.
    private Vector2 agentAnimOffset;
    private float agentAnimOffsetMagnitude;
    private float currentSpeed;
    private float targetSpeed;
    private float smoothDampVelocity;

    private readonly Vector3 invalidVector = new Vector3(Mathf.Infinity, Mathf.Infinity, Mathf.Infinity);

    // Variables for timers.
    private DateTime pathTimerStart;
    private bool pathTiming;
    private TimeSpan timeToCalcPathTS;
    
    private DateTime destTimerStart;
    private bool destTiming;
    private TimeSpan timeForNewDestTS;
    
    // If NPC_DEBUG is defined, expose the variables below in the Inspector.
#if NPC_DEBUG
    [SerializeField] private int numFrames;
    [SerializeField] private int numFramesToWait;
    [SerializeField] private string curState;
    [SerializeField] private Vector3 destination = Vector3.zero; // Initialized to an arbitrary value.
    [SerializeField] private bool hasPath;
    [SerializeField] private float distance = Mathf.Infinity;
    [SerializeField] private string timeToCalcPath;
    [SerializeField] private string timeForNewDest;
    [SerializeField] private string traceString;
#else
    private int numFrames;
    private int numFramesToWait;
    private string curState;
    private Vector3 destination = Vector3.zero;
    private bool hasPath;
    private float distance = Mathf.Infinity;
    private string timeToCalcPath;
    private string timeForNewDest;
    private string traceString;
#endif

    public delegate void EnterStateFunction();
    public delegate void UpdateStateFunction();
    private bool debugFSM = false;

    public class NpcState
    {
        public readonly string stateName;
        public EnterStateFunction enterStateFn;
        public UpdateStateFunction updateStateFn;
        public NpcState(string nm, EnterStateFunction enterfn, UpdateStateFunction updatefn)
        {
            stateName = nm;
            enterStateFn = enterfn;
            updateStateFn = updatefn;
        }
    }

    private readonly NpcState[] myStates;

    public npcIndividual()
    {
        /*
         * Create a table of the NPCs states, and the functions to be called when entering or updating each state.
         */
        myStates = new NpcState[]
        {
            new NpcState("Start",           EnterStartState,           UpdateStartState),
            new NpcState("Travel",          EnterTravelState,          UpdateTravelState),
            new NpcState("Paused",          EnterPausedState,          UpdatePausedState),
            new NpcState("NewDestination",  EnterNewDestinationState,  UpdateNewDestinationState)
        };
    }

    void Awake()
    {
        // Initialize component and object references.
        anim = GetComponent<Animator>();
        if (!anim) Debug.LogError(this.name + " could not get Animator.");
        
        agent = GetComponent<NavMeshAgent>();
        if (!agent) Debug.LogError(this.name + " could not get the NavMeshAgent.");

        if (waypointParent == null)
        {
            waypointParent = GameObject.Find("Waypoints");
            if (waypointParent == null)
                Debug.LogError("Need to set waypoint parent.");
        }
        traceString = "Awoken";
    }

    void Start()
    {
        // Initialize timers.
        destTiming = false;
        pathTiming = false;
        
        // Double check component references.
        if (!anim) Debug.LogError(this.name + " does not have Animator reference.");
        if (!agent) Debug.LogError(this.name + " does not have NavMeshAgent reference.");
        
        // Set the agent's stopping distance.
        agent.stoppingDistance = stoppingDistance;
        if (Mathf.Approximately(agent.stoppingDistance, 0.0f))
        {
            Debug.LogError(this.name + " needs to have a stopping distance greater than zero.");
        }
        
        // We update the agent's position in functions below to be coordinated with the animation clips.
        agent.updatePosition = false;
        
        // Initialize movement variables.
        agentVelocity = Vector2.zero;
        nextPosition = currentPosition = transform.position;
        nextForward = currentForward = transform.forward;
        currAnimPosition = transform.position;
        prevDeltaForward = currDeltaForward = 0f;
        
        traceString = "Awoken";

        // Start the Finite State Machine.
        EnterStartState();
    }
    
    void Update()
    {
        CoordinateAnimationAndNavigation();
        AdjustSpeed();
        FsmUpdate();
    }

    void CoordinateAnimationAndNavigation()
    {
        // Set some variables related to the next position that are calculated by the NavMeshAgent.
        currentPosition = nextPosition;
        nextPosition = agent.nextPosition;
        deltaPosition = nextPosition - currentPosition;
        dxAgent = Vector3.Dot(transform.right, deltaPosition);
        dyAgent = Vector3.Dot(transform.forward, deltaPosition);
        vxAgent = dxAgent / Time.deltaTime;
        vyAgent = dyAgent / Time.deltaTime;
        agentVelocity.x = vxAgent; 
        agentVelocity.y = vyAgent;
        agentSpeed = agentVelocity.magnitude;

        // Calculate calculate the difference in forward angle between frames.
        currentForward = nextForward;
        nextForward = transform.forward; // Transform of the NPC GameObject rather than the NavMeshAgent.
        prevDeltaForward = currDeltaForward;
        currDeltaForward = Vector3.Angle(nextForward, currentForward);
        // Determine whether the change in angle is negative.
        if (Vector3.Cross(nextForward, currentForward).y > 0f)
        {
            currDeltaForward = -currDeltaForward;
        }
        // Smooth out deltaAgentForward.
        currDeltaForward = Mathf.SmoothDampAngle(prevDeltaForward, currDeltaForward, ref currentVelocity, SmoothTime);

        if (agentSpeed > MoveThreshold)
        {
            anim.SetBool("move", true);
            anim.SetFloat("velx", currDeltaForward);
            anim.SetFloat("vely", vyAgent);
        }
        else
        {
            anim.SetBool("move", false);
        }
        
        // Determine whether the animator and agent have drifted too far apart.
        currAnimPosition = transform.position;
        nextPosition2D.x = nextPosition.x;
        nextPosition2D.y = nextPosition.z;
        currAnimPosition2D.x = currAnimPosition.x;
        currAnimPosition2D.y = currAnimPosition.z;
        agentAnimOffset = nextPosition2D - currAnimPosition2D;
        agentAnimOffsetMagnitude = agentAnimOffset.magnitude;
        if (agentAnimOffsetMagnitude > agent.radius * SyncThreshold)
        {
            // Pull the animator to the agent.
            Vector2 animTarget = currAnimPosition2D + agentAnimOffset;  // same as nextPosition2D
            Vector2 newAnimPosition2D = Vector2.Lerp(currAnimPosition2D, animTarget, SyncDamping);
            Vector3 newAnimPosition;
            newAnimPosition.x = newAnimPosition2D.x;
            newAnimPosition.y = transform.position.y;
            newAnimPosition.z = newAnimPosition2D.y;
            transform.position = newAnimPosition;
        }
    }

    void OnAnimatorMove()
    {
        // Match the y (vertical) coordinate of the animator and the agent.
        Vector3 position = anim.rootPosition;
        position.y = agent.nextPosition.y;
        transform.position = position;
    }

    private void FsmUpdate()
    {
        // Find the entry in myStates for curState, and call its update function.
        bool matchFound = false;
        for (int i = 0; i < myStates.Length; i++)
        {
            if (curState == myStates[i].stateName)
            {
                myStates[i].updateStateFn();
                matchFound = true;
                break;
            }
        }
        if (!matchFound)
        {
            Debug.LogError("Unrecognized state: " + curState);
        }
    }

    // Waypoint code.
    private Vector3 GetAWaypoint(Vector3 omit, float angMin = -90f, float angMax = 90f)
    {
        List<Vector3> validWPlist = new List<Vector3>();  // Randomly choose one of the valid waypoints.
        List<Vector3> tooFarWPlist = new List<Vector3>();  // If there are no valid waypoints, randomly choose one of the others.
        List<Vector3> wrongDirWPlist = new List<Vector3>();
        int cnt = waypointParent.transform.childCount;
        Vector3 selfPos = transform.position;
        Vector3 selfForward = transform.forward;
        Vector3 returnVector = invalidVector;
        // Examine all the waypoints.
        for (int i = 0; i < cnt; i++)
        {
            Vector3 currentWp = waypointParent.transform.GetChild(i).position;
            // Is the current waypoint the one that should be omitted?
            if (currentWp == omit) continue;
            // Is the current waypoint too far away?
            if (limitRange)
            {
                float dist = Vector3.Distance(selfPos, currentWp);
                if (dist > range)
                {
                    tooFarWPlist.Add(currentWp);
                    continue;
                }
            }
            // Is the current waypoint in the wrong direction?
            float angleToWaypoint = Vector3.Angle(selfForward, currentWp - selfPos);
            if (angleToWaypoint < angMin || angleToWaypoint > angMax)
            {
                wrongDirWPlist.Add(currentWp);
                continue;
            }
            // If we made it to here, the waypoint is valid.
            validWPlist.Add(currentWp);
        }
        // If there are any valid waypoints, choose one at random.
        if (validWPlist.Count > 0)
        {
            int index = (int)Mathf.Floor(UnityEngine.Random.value * validWPlist.Count);
            returnVector = validWPlist[index];
        }
        else if (wrongDirWPlist.Count > 0)
        {
            // Consider a point that is in the wrong direction.
            int index = (int)Mathf.Floor(UnityEngine.Random.value * wrongDirWPlist.Count);
            returnVector = wrongDirWPlist[index];
        }
        else if (tooFarWPlist.Count > 0)
        {
            // Consider a point that is too far.
            int index = (int)Mathf.Floor(UnityEngine.Random.value * tooFarWPlist.Count);
            returnVector = tooFarWPlist[index];
        }
        else
        {
            // Should never get here unless there is only one waypoint -- the one being omitted.
            Debug.LogError("Need to have more than one waypoint defined.");
        }
        return returnVector;
    }

    /*
     * Every state XXX needs functions enterXXXState(), updateXXXState, and a case in leaveState().
     * By convention, enterXXXState() should start with a call to leaveState().
     */

    private void EnterStartState()
    {
        //leaveState() is not called for the initial state.
        curState = "Start";
        if (debugFSM) Debug.Log(string.Format("{0}: entering", curState));
        traceString = "Start state";
    }

    private void UpdateStartState()
    {
        EnterNewDestinationState();
    }

    private void EnterPausedState()
    {
        LeaveState();
        curState = "Paused";
        SetSpeed(0f);
        numFrames = 0;
        numFramesToWait = (int) (UnityEngine.Random.value * (maxFramesToWait - minFramesToWait) + minFramesToWait);
        if (debugFSM) Debug.Log(string.Format("{0}: entering", curState));
        traceString = "Paused state";
    }

    private void UpdatePausedState()
    {
        if (numFrames > numFramesToWait)
        {
            EnterNewDestinationState();
            numFrames = 0;
        }
        else
        {
            numFrames++;
        }
    }

    private void EnterTravelState()
    {
        LeaveState();
        curState = "Travel";
        SetSpeed(usualSpeed);
        if (debugFSM) Debug.Log(string.Format("{0}: entering", curState));
        traceString = "Travel state";
    }

    private void UpdateTravelState()
    {
        if (GetDistanceToDestination() < agent.stoppingDistance)
        {
            traceString = "at destination";
            EnterPausedState();
        }
    }

    private void EnterNewDestinationState()
    {
        LeaveState();
        curState = "NewDestination";
        SetSpeed(0f);
        
        // Start timer.
        destTimerStart = DateTime.Now;
        destTiming = true;
        Vector3 newDestination = GetAWaypoint(destination);

        SetDestination(newDestination);
        if (debugFSM) Debug.Log(string.Format("{0}: entering", curState));
        traceString = "New Destination state";
    }

    private void UpdateNewDestinationState()
    {
        if (destTiming)
        {
            // The timer includes the time for control to transition here from EnterNewDestinationState().
            // When the timer is placed right after the function call call in that function, the result is always zero.
            timeForNewDestTS = DateTime.Now - destTimerStart;
            timeForNewDest = timeForNewDestTS.ToString(@"mm\:ss\.fff");
            destTiming = false;
        }
        hasPath = agent.hasPath;
        if (hasPath)
        {
            // complete timer.
            if (pathTiming)
            {
                timeToCalcPathTS = DateTime.Now - pathTimerStart;
                timeToCalcPath = timeToCalcPathTS.ToString(@"mm\:ss\.fff");
                pathTiming = false;
            }
            EnterTravelState();
        }
        else
        {
            traceString = "New Destination, hasPath is false";
        }
    }

    protected virtual void LeaveState()
    {
        switch (curState)
        {
            case "Start":
                {
                    break;
                }
            case "Paused":
                {
                    break;
                }
            case "Travel":
                {
                    break;
                }
            case "NewDestination":
                {
                    break;
                }
        }
        if (debugFSM) Debug.Log(string.Format("{0}: Leaving ", curState));
    }

    /*
     * Some utility functions related to movement.
     */
    private void SetSpeed(float newTargetSpeed, bool force = false)
    {
        targetSpeed = newTargetSpeed;
        if (force)
        {
            agent.speed = currentSpeed = targetSpeed;
        }
    }

    // Gradually change speed to targetSpeed.
    private void AdjustSpeed()
    {
        currentSpeed = Mathf.SmoothDamp(currentSpeed, targetSpeed, ref smoothDampVelocity, SmoothTime);
        if (!double.IsNaN(currentSpeed))  // This may happen on shutdown.
            agent.speed = currentSpeed;
    }

    private void SetDestination(Vector3 newDest)
    {
        if (agent.isActiveAndEnabled && agent.isOnNavMesh)
        {
            agent.destination = newDest;
            destination = newDest;
            // start timer.
            pathTimerStart = DateTime.Now;
            pathTiming = true;
        }
        else
        {
            Debug.LogWarning(name + " can't execute setDestination() for " + newDest);
        }
    }

    private float GetDistanceToDestination()
    {
        if (Application.isPlaying)  // Not while executing in editor.
        {
            distance = agent.remainingDistance;
            if (float.IsPositiveInfinity(distance))
            {
                // NavMeshAgent is not returning the distance yet.  Approximate with straight-line distance.
                traceString = "agent distance is infinity";
                distance = Vector3.Distance(transform.position, destination);
            }
            else
            {
                traceString = "agent distance is " + distance;
            }
        }
        else // application not playing
        {
            Debug.LogWarning("GetDistanceToDestination() called when the application is not playing.");
            distance = Vector3.Distance(transform.position, destination);
            traceString = "GetDistanceToDestination() with appln not playing.";
        }
        return distance;
    }

}
