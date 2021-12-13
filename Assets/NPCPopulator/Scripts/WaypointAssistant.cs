using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;


/*
 * This class suggests the placements of waypoints in Unity scenes.  It works best when the environment
 * has a "rectangular" layout, such as city streets.
 * The suggested placement of the waypoints is hoped to be reasonable, but is unlikely to be perfect.
 * The waypoints are simply GameObjects, and can be moved or deleted to tweak the suggested placement.
 */
[ExecuteAlways]
public class WaypointAssistant : MonoBehaviour
{
    /*
     * A "WaypointGrid" is created that overlays the NavMesh ground area of the environment but at a
     * resolution of a constant multiplier times the radius of a NavMeshAgent.
     */
    
    // Enumeration to represent values of the WaypointGrid.
    public enum WaypointGridType
    {
        None,
        Unknown,
        NotOnNavMesh,
        Hub,
        Straight,
        Terminus,
        Island,
        HubUsed,
        TerminusUsed
    }

    // It would be nice to get this automatically,
    // but on the other hand it could be different for different agents.
    public float agentRadius = 0.5f;

    // The bounds on the entire NavMesh.
    public Bounds navMeshBounds;
    
    // The grid of points to consider in seeking good locations for waypoints.
    private WaypointGridType[,] _waypointGrid;
    public int gridXSize = 0;
    public int gridYSize = 0;
    
    // Template to use for waypoints that are created.
    public GameObject waypoint;
    
    // Parent object for waypoints that are created.
    public Transform waypointParent;

    // Which NavMesh areas to consider for waypoints.
    public int navMeshMask = NavMesh.AllAreas;

    // Whether or not a terrain is used in the environment.
    public bool hasTerrain = false;

    // The level of the ground, if no terrain is used.
    // If there is a terrain, this should be set to the approximate or "bottom" ground level.
    public float groundLevel = 0f;

    // How much above the ground should a waypoint be?
    public float waypointElevation = 1f; 

    // How far down a corridor should we probe?
    public float corridorDistance = 10f;

    // How far should probe (small distance) to determine whether a point is on the NavMesh?
    public float navMeshProbe = 0.5f;

    // Whether to continually perform calculations on the WaypointGrid interactively
    // as parameters are adjusted.  This is helpful for setting the parameters, but
    // may consume processing power, so should be turned off when not being used.
    public bool enableInteractive = false;

    // The minimum size of a square in the grid that can generate a waypoint.
    public int minSquareSize = 5;

    // Whether to draw the gizmos on the WaypointGrid.  Having this on consumes processing 
    // power, so should be turned off when not being used.
    public bool drawGizmos = true;

    // The number of frames to skip when enableInteractive is true.
    // 1 - recalculate the WaypointGrid every frame.
    // 2 - recalculate every other frame.
    // etc.
    // Larger numbers require less processing power, but have some delay in the interactivity.
    public int skipFrames = 10;

    /*
     * Another parameter that can be adjusted to limit processing power.
     * 1 - use every point
     * 2 - use every other point
     * etc.
     * When the view is zoomed out, the gridResolution should be set to a value 2-4, since more
     * grid points are in view.
     */
    public int gridResolution = 1;
    
    // Whether to generate waypoints from terminus regions in addition to hub regions.
    // This is false by default.
    public bool waypointsFromTerminus = false;

    /*
     * These parameters are not exposed in the Editor by default.
     * They are for development or experimental.
     */

    // Whether waypoints are being generated across the entire NavMesh or just the visible subset.
    // This is false by default.  In normal operation, waypoints are generated across the entire NavMesh.
    public bool calculatingSubset = false;
    
    // Whether or not waypoints have been generated.  This affects the color of the grid point gizmos.
    public bool waypointsGenerated = false;

    // Some performance metrics.
    public int gridCount;
    public float gizmoDrawTime;
    public float gridResetTime;
    public float gridCalcTime;
    public float waypointGenTime;
    
    // The factor by which to multiply agentRadius for the size of the gizmos.
    // Probably does not need to be adjusted.
    public float gizmoSizeFactor = 0.5f;

    // Variable to hold the size of each grid cell gizmo within WaypointGrid.
    private Vector3 _gizmoSize = new Vector3(0f, 0f, 0f);

    /*
     * Variables related to the scene camera.
     * These are maintained to only draw gizmos in the region of the scene currently visible
     * in the scene view.
     */
    
// The SceneView type is not available in generated apps.      
#if UNITY_EDITOR
    public SceneView currentSceneView;
#endif
    public Camera sceneCamera;
#if UNITY_EDITOR && UNITY_2019_1_OR_NEWER    
    public SceneView.CameraSettings sceneCameraSettings;
#endif    
    public Transform sceneCameraTransform;
    //public float sceneCameraHeight;
    public float sceneFieldOfView;
    public float sceneAspectRatio;

    // The rays along the 4 diagonal edges of the scene view frustum.
    private Ray[] _frustumRays = new Ray[4];
    
    // A bounding rectangle on the ground that includes what is currently visible in the scene view.
    public Rect groundProjection;
    
    // Warning flags for performance of grid calculations and gizmo drawing.
    public bool gridCalcPerfWarning = false;
    public bool gizmoDrawPerfWarning = false;
    private const float FractionOfFrame = 0.25f; // How much of the frame time to be used for grid calculations before warning.

    // Used to initialize some data structures, subsequently adjusted.
    const float LargeNumber = 1E8f;
    
    /*
     * Variables for automatically adjusting the grid resolution.
     * This is commented out.  It was difficult (perhaps impossible) for automatic adjustments to work correctly in
     * all cases.  Instead, warning indicators were added to the Inspector to facilitate manual adjustments.
     * 
    // Thresholds for automatically adjusting the grid resolution.
    public bool automaticGridResolution = false;
    public int gridResolutionThreshPosDir = 1000;
    //public int gridResolutionThreshNegDir = 200;
    public float[] gridResolutionHeightThresh = new float[3];
    public bool[] gridResolutionHeightThreshSet = new bool[3];
    public float gridResolutionThreshCorrectionFactor = 0.95f;
     */

    // Whether to show developer part of the Inspector editor.
    public bool developerMode = false;

    /*
     * Event Handlers
     */
    

// This entire script is only valid in the Unity Editor, not in compiled builds.    
#if UNITY_EDITOR
    void Update()
    {
        if (enableInteractive && !Application.isPlaying)
        {
            if (Time.frameCount % skipFrames == 0)
            {
                /*
                 * Never automate calculations on the full NavMesh.
                 * Only calculate that when generating waypoints, not every frame.
                 */
                CalculateGroundProjection();
                
                // Commenting out automatic resolution adjustment.
                //if (automaticGridResolution) AdjustGridResolution();
            }
        }

        UpdatePerformanceWarnings();
    }
    void OnDrawGizmos()
    {
        // Just return if we are not drawing the gizmos.
        if (!drawGizmos)
        {
            gizmoDrawTime = 0;
            return;
        }
     
#if UNITY_EDITOR        
        if (!Application.isPlaying)
        {
            // Repaint the gizmos (and likewise the entire scene) when in Editor mode.
            EditorApplication.QueuePlayerLoopUpdate();
            SceneView.RepaintAll();
        }
#endif

        DateTime startTime = DateTime.Now;
        if (_waypointGrid == null) return;
        int gridMinX, gridMaxX, gridMinY, gridMaxY;
        gridMinX = WorldPointXToGridX(groundProjection.xMin);
        gridMaxX = WorldPointXToGridX(groundProjection.xMax);
        gridMinY = WorldPointZToGridY(groundProjection.yMin);
        gridMaxY = WorldPointZToGridY(groundProjection.yMax);
        
        /* If we were to draw gizmos over the entire NavMesh, we would use these lines of code.
         * This is not being used.  We are only drawing the gizmos on the region that is
         * visible in the Scene view.
            gridMinX = WorldPointXToGridX(navMeshBounds.min.x);
            gridMaxX = WorldPointXToGridX(navMeshBounds.max.x);
            gridMinY = WorldPointZToGridY(navMeshBounds.min.z);
            gridMaxY = WorldPointZToGridY(navMeshBounds.max.z);
         */
        
        _gizmoSize.x = _gizmoSize.y = _gizmoSize.z = agentRadius * gizmoSizeFactor;
        _gizmoSize.x *= gridResolution;
        _gizmoSize.z *= gridResolution;
        gridCount = 0;
        for (int xcoord = gridMinX; xcoord < gridMaxX; xcoord=xcoord+gridResolution)
        {
            for (int ycoord = gridMinY; ycoord < gridMaxY; ycoord=ycoord+gridResolution)
            {
                WaypointGridType currentGridType = _waypointGrid[xcoord, ycoord];
                if (currentGridType != WaypointGridType.Unknown &&
                    currentGridType != WaypointGridType.NotOnNavMesh &&
                    currentGridType != WaypointGridType.Island &&
                    currentGridType != WaypointGridType.Straight &&
                    currentGridType != WaypointGridType.None &&
                    (waypointsFromTerminus || 
                        (currentGridType != WaypointGridType.Terminus && 
                            currentGridType != WaypointGridType.TerminusUsed)))
                {
                    // Get the world coordinates.
                    Vector3 waypointPos = WaypointGridToWaypoint(xcoord, ycoord);
                    // Draw a gizmo, colored by grid type.
                    switch (currentGridType)
                    {
                        case WaypointGridType.Hub:
                            Gizmos.color = Color.green;
                            break;
                        case WaypointGridType.HubUsed:
                            Gizmos.color = Color.cyan;
                            break;
                        case WaypointGridType.Terminus:
                            Gizmos.color = Color.red;
                            break;
                        case WaypointGridType.TerminusUsed:
                            Gizmos.color = Color.magenta;
                            break;
                        /*
                        case WaypointGridType.Island:
                            Gizmos.color = Color.yellow;
                            break;
                        case WaypointGridType.Straight:
                            Gizmos.color = Color.blue;
                            break;
                        */
                    }
                    Gizmos.DrawCube(waypointPos, _gizmoSize);
                    gridCount++;
                }
            }
        }
        DateTime endTime = DateTime.Now;
        gizmoDrawTime = (endTime - startTime).Milliseconds / 1000f;
    }

    private void UpdatePerformanceWarnings()
    {
        gridCalcPerfWarning = enableInteractive && (gridCalcTime / skipFrames) > (Time.smoothDeltaTime * FractionOfFrame);
        gizmoDrawPerfWarning = drawGizmos && gizmoDrawTime > (Time.smoothDeltaTime * FractionOfFrame);
    }
    
    /*
     * Calculate the bounds that surround the scene's NavMesh and create the WaypointGrid.
     */
    public void CalculateBoundsOfNavMesh()
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

        // Get the extremes in all four directions.
        // (This approach could provide incorrect results if the ground area is a strange / concave shape.)
        if (NavMesh.SamplePosition(Vector3.left * maxDistanceHalf, out hitLeft, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.back * maxDistanceHalf, out hitBack, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.right * maxDistanceHalf, out hitRight, maxDistance, NavMesh.AllAreas) &&
            NavMesh.SamplePosition(Vector3.forward * maxDistanceHalf, out hitForward, maxDistance, NavMesh.AllAreas))
        {
            navMeshBounds.min = new Vector3(hitLeft.position.x, groundLevel, hitBack.position.z);
            navMeshBounds.max = new Vector3(hitRight.position.x, groundLevel, hitForward.position.z);
            CreateWaypointCalculationGrid();
        }
        else
        {
            Debug.LogError("Error getting NavMesh bounds.");
        }
    }
    
    private void CreateWaypointCalculationGrid()
    {
        /*
         * The grid is 2D.  The vertical dimension is not considered.  All waypoints are considered to be at the
         * same "ground" level.
         * The dimensions of the grid are 2 * bounds-extents / agent-radius.
         * Each grid point is an enum of unknown / hub / terminus / straight / redundant-hub / redundant-terminus.
         */
        gridXSize = (int) (2 * navMeshBounds.extents.x / agentRadius);
        gridYSize = (int) (2 * navMeshBounds.extents.z / agentRadius);

        _waypointGrid = new WaypointGridType[gridXSize,gridYSize];
    }

    /*
     * Functions to get the extents of the current rectangular tile being considered.
     * Generally the tile may be the entire NavMesh, or the ground projection of the view frustum.
     * In practice, only the ground projection will be used for performance.
     */
    
    // Functions to calculate the ground bounding box of the intersection with the scene camera's view frustum.
    public void CalculateGroundProjection()
    {
        UpdateSceneCamera();
        groundProjection = GroundProjectionOfViewFrustum(BoundsToRect(navMeshBounds), sceneCamera, sceneCameraTransform, sceneFieldOfView, sceneAspectRatio);
        CalculateWithinGroundProjection();
    }

    private void UpdateSceneCamera()
    {
        /*
         * The next two lines use properties that are not in the Unity 2018.4 manual,
         * but these lines do work in 2018.4!
         * The properties are documented starting in 2019.1.
         * It seems they did exist in 2018.4 but were not documented yet.
         */
        currentSceneView = SceneView.lastActiveSceneView;
        sceneCamera = currentSceneView.camera;
        
        sceneCameraTransform = sceneCamera.transform;
        sceneAspectRatio = sceneCamera.aspect;
        //sceneCameraHeight = sceneCameraTransform.position.y;
#if UNITY_2019_1_OR_NEWER        
        sceneCameraSettings = currentSceneView.cameraSettings;
        sceneFieldOfView = sceneCameraSettings.fieldOfView;
#else
        sceneFieldOfView = 45f;  // Just a guess at the scene camera's field of view if using Unity 2018.4.
#endif
    }

    /*
     * This function attempts to find a rectangle on the ground that contains the view frustum, to perform interactive
     * calculations only in the visible region.  By default, the groundRectangle is returned if a smaller rectangle
     * cannot be found.  The 4 "side" planes of the view frustum are used, not the near and far planes.  Working
     * with 4 planes rather than 6 significantly simplifies the calculations and is expected to handle most cases.
     * - groundRectangle is a rectangle in world coordinates to bound where there is a valid ground area.
     * - groundLevel will usually be zero, but is passed in as a parameter in case it is not zero.
     * If the ground is not all at a single altitude, it needs to be approximated as a plane.
     */
    Rect GroundProjectionOfViewFrustum(Rect groundRectangle, Camera thisCamera, Transform cameraTransform, float horizontalFieldOfView, float aspectRatio, float grndLevel=0f)
    {
        // The projection of the view frustum on the ground plane.
        Rect frustumProjection = new Rect(-LargeNumber, -LargeNumber, LargeNumber, LargeNumber);
        // The common area between the groundRectangle and the frustumProjection, to be calculated in this function.
        Rect environmentProjection = new Rect(-LargeNumber, -LargeNumber, LargeNumber, LargeNumber); 

#if UNITY_2019_1_OR_NEWER        
        float verticalFieldOfView = Camera.HorizontalToVerticalFieldOfView(horizontalFieldOfView, aspectRatio);
#else
        float verticalFieldOfView = 0.67f * horizontalFieldOfView;  // Estimate about a 2:3 aspect ratio.
#endif        
        Vector3 cameraPosition = cameraTransform.position;
        Plane groundPlane = new Plane(Vector3.up, new Vector3(0f, grndLevel, 0f));
        Ray rayThroughViewport = thisCamera.ViewportPointToRay(new Vector3(0.5F, 0.5F, 0));
        
        // Data structures for the 4 frustum planes:  0 = left, 1 = top, 2 = right, 3 = bottom
        Vector3[] frustumNormals = new Vector3[4];
        Plane[] frustumPlanes = new Plane[4];
        
        // Data structures for 4 frustum intersection rays: 0 = left/top, 1 = top/right, 2 = right/bottom, 3 = bottom/left
        Vector3[] rayDirections = new Vector3[4];
        Vector3[] rayPoints = new Vector3[4];
        _frustumRays = new Ray[4];
        bool[] rayIntersectionSucceeded = new bool[4];
        Vector3[] rayIntersection = new Vector3[4];
        
        // Setup the 4 frustum planes.
        Vector3 cameraUp = cameraTransform.up;
        Vector3 cameraRight = cameraTransform.right;
        // left
        frustumNormals[0] = Quaternion.AngleAxis(-horizontalFieldOfView, cameraUp) * cameraRight;
        frustumPlanes[0] = new Plane(frustumNormals[0], cameraPosition);
        // top
        frustumNormals[1] = Quaternion.AngleAxis(-verticalFieldOfView, cameraRight) * -cameraUp;
        frustumPlanes[1] = new Plane(frustumNormals[1], cameraPosition);
        // right
        frustumNormals[2] = Quaternion.AngleAxis(horizontalFieldOfView, cameraUp) * -cameraRight;
        frustumPlanes[2] = new Plane(frustumNormals[2], cameraPosition);
        // bottom
        frustumNormals[3] = Quaternion.AngleAxis(verticalFieldOfView, cameraRight) * cameraUp;
        frustumPlanes[3] = new Plane(frustumNormals[3], cameraPosition);

        // Intersect adjacent planes to get 4 rays.
        for (int j = 0; j < 4; j++)
        {
            int next = (j + 1) % 4;
            // The sequence of j and next below is important so that the resulting ray direction is not backwards.
            bool intersectionSucceeded = PlanePlaneIntersection(out rayPoints[j], out rayDirections[j], frustumPlanes[next].normal,
                cameraPosition, frustumPlanes[j].normal, cameraPosition);
            if (!intersectionSucceeded) return groundRectangle;  // This is not likely to happen.
            _frustumRays[j].origin = cameraPosition;
            _frustumRays[j].direction = rayDirections[j];
        }
        
        // Intersect 4 rays with ground plane to get 4 corners.
        for (int k = 0; k < 4; k++)
        {
            float enter;
            rayIntersectionSucceeded[k] = groundPlane.Raycast(_frustumRays[k], out enter);
            if (rayIntersectionSucceeded[k])
            {
                rayIntersection[k] = _frustumRays[k].GetPoint(enter);
            }
        }
        
        // Calculate the frustumProjection which is the bounding box around the projection of the view frustum on the ground plane.
        if (!rayIntersectionSucceeded[0] && !rayIntersectionSucceeded[1] && !rayIntersectionSucceeded[2] && !rayIntersectionSucceeded[3])
        {
            // The view frustum does not intersect the ground rectangle at all.  This can happen if the camera is pointed up.  Return an empty Rectangle.
            return Rect.zero;
        }
        // Find the first rayIntersection that succeeded.
        int firstIntersection = -1;
        for (int l = 0; l < 4; l++)
        {
            if (rayIntersectionSucceeded[l])
            {
                firstIntersection = l;
                break;
            }
        }
        if (firstIntersection == -1)
        {
            Debug.LogError("Should never get here.");
        }
        
        // Initialize frustumProjection to the first intersecting point, because this is known to be a point in frustumProjection.
        frustumProjection.xMin = rayIntersection[firstIntersection].x;
        frustumProjection.yMin = rayIntersection[firstIntersection].z;
        frustumProjection.width = 0f;
        frustumProjection.height = 0f;

        // Look at each of the rayIntersections.
        for (int m = 0; m < 4; m++)
        {
            if (rayIntersectionSucceeded[m])
            {
                if (rayIntersection[m].x < frustumProjection.xMin) frustumProjection.xMin = rayIntersection[m].x;
                if (rayIntersection[m].x > frustumProjection.xMax) frustumProjection.xMax = rayIntersection[m].x;
                if (rayIntersection[m].z < frustumProjection.yMin) frustumProjection.yMin = rayIntersection[m].z;
                if (rayIntersection[m].z > frustumProjection.yMax) frustumProjection.yMax = rayIntersection[m].z;
            }
            else // no intersection of frustum ray with ground plane
            {
                float viewPortRayDirX = rayThroughViewport.direction.x;
                float viewPortRayDirZ = rayThroughViewport.direction.z;
                // for the left frustum rays, if the viewport is aligned with world x direction,
                // or for right frustum rays if viewport goes in opposite direction to the world x direction... 
                if (((m==0 || m==3) && viewPortRayDirX >= 0f) || ((m==1 || m==2) && viewPortRayDirX < 0f))
                    frustumProjection.xMin = -LargeNumber;
                else    //if (((m==0 || m==3) && viewPortRayDirX < 0f) || ((m==1 || m==2) && viewPortRayDirX >= 0f))
                    frustumProjection.xMax = LargeNumber;
                // for the top frustum rays, if the viewport is aligned with world z direction,
                // or for bottom frustum rays if viewport goes in opposite direction to the world z direction... 
                if (((m==0 || m==1) && viewPortRayDirZ >= 0f) || ((m==2 || m==3) && viewPortRayDirZ < 0f))
                    frustumProjection.yMax = LargeNumber;
                else
                    frustumProjection.yMin = -LargeNumber;
            }
        }
        
        // Calcuate the environmentProjection as the intersection of the groundRectangle with the frustumProjection.
        // Any "infinity" factors should not survive this, because the groundRectangle is finite, xMin/yMin may be
        // NegativeInfinity in frustumProjection while xMax/yMax may be Infinity in frustumProjection.
        environmentProjection.xMin = Mathf.Max(groundRectangle.xMin, frustumProjection.xMin);
        environmentProjection.yMin = Mathf.Max(groundRectangle.yMin, frustumProjection.yMin);
        environmentProjection.xMax = Mathf.Min(groundRectangle.xMax, frustumProjection.xMax);
        environmentProjection.yMax = Mathf.Min(groundRectangle.yMax, frustumProjection.yMax);
        
        return environmentProjection;
    }
    
    // https://wiki.unity3d.com/index.php/3d_Math_functions
    //Find the line of intersection between two planes.	The planes are defined by a normal and a point on that plane.
    //The outputs are a point on the line and a vector which indicates it's direction. If the planes are not parallel, 
    //the function outputs true, otherwise false.
    private static bool PlanePlaneIntersection(out Vector3 linePoint, out Vector3 lineVec, Vector3 plane1Normal, Vector3 plane1Position, Vector3 plane2Normal, Vector3 plane2Position){
 
        linePoint = Vector3.zero;
        lineVec = Vector3.zero;
 
        //We can get the direction of the line of intersection of the two planes by calculating the 
        //cross product of the normals of the two planes. Note that this is just a direction and the line
        //is not fixed in space yet. We need a point for that to go with the line vector.
        lineVec = Vector3.Cross(plane1Normal, plane2Normal);
 
        //Next is to calculate a point on the line to fix it's position in space. This is done by finding a vector from
        //the plane2 location, moving parallel to it's plane, and intersecting plane1. To prevent rounding
        //errors, this vector also has to be perpendicular to lineDirection. To get this vector, calculate
        //the cross product of the normal of plane2 and the lineDirection.		
        Vector3 ldir = Vector3.Cross(plane2Normal, lineVec);		
 
        float denominator = Vector3.Dot(plane1Normal, ldir);
 
        //Prevent divide by zero and rounding errors by requiring about 5 degrees angle between the planes.
        if(Mathf.Abs(denominator) > 0.006f){
 
            Vector3 plane1ToPlane2 = plane1Position - plane2Position;
            float t = Vector3.Dot(plane1Normal, plane1ToPlane2) / denominator;
            linePoint = plane2Position + t * ldir;
 
            return true;
        }
 
        //output not valid
        else 
        {
            return false;
        }
    }

    // Calculate the waypoint grid values within the ground projection of the view frustum.
    public void CalculateWithinGroundProjection()
    {
        calculatingSubset = true;
        int gridMinX = WorldPointXToGridX(groundProjection.xMin);
        int gridMaxX = WorldPointXToGridX(groundProjection.xMax);
        int gridMinY = WorldPointZToGridY(groundProjection.yMin);
        int gridMaxY = WorldPointZToGridY(groundProjection.yMax);
        ResetWaypointGrid();
        // Calculate at gridResolution.
        CalculateWithinRectangle(gridMinX, gridMaxX, gridMinY, gridMaxY, gridResolution);
    }

    // Calculate the waypoint grid values across the entire NavMesh.
    public void CalculateEntireNavMesh()
    {
        calculatingSubset = false;
        int gridMinX = WorldPointXToGridX(navMeshBounds.min.x);
        int gridMaxX = WorldPointXToGridX(navMeshBounds.max.x);
        int gridMinY = WorldPointZToGridY(navMeshBounds.min.z);
        int gridMaxY = WorldPointZToGridY(navMeshBounds.max.z);
        ResetWaypointGrid();
        // Calculate full grid (resolution == 1).
        CalculateWithinRectangle(gridMinX, gridMaxX, gridMinY, gridMaxY, 1);
    }

    // Set all the cells within the waypoint grid as unknown.
    public void ResetWaypointGrid()
    {
        DateTime startTime = DateTime.Now;
        if (_waypointGrid == null)
        {
            // We don't expect this case to be hit.  Print a message if it happens.  It may be that the
            // waypoint grid was not yet properly initialized.
            Debug.LogWarning("Reallocating WaypointGrid.");
            _waypointGrid = new WaypointGridType[gridXSize,gridYSize];
        }
        for (int i=0; i<gridXSize; i++)
        for (int j = 0; j < gridYSize; j++)
            _waypointGrid[i, j] = WaypointGridType.Unknown;
        DateTime endTime = DateTime.Now;
        gridResetTime = (endTime - startTime).Milliseconds / 1000f;
    }

    // Calculate the values of cell of the waypoint grid within the provided bounds.
    private void CalculateWithinRectangle(int gridMinX, int gridMaxX, int gridMinY, int gridMaxY, int gridDelta)
    {
        DateTime startTime = DateTime.Now;
        for (int xcoord = gridMinX; xcoord < gridMaxX; xcoord=xcoord+gridDelta)
        {
            for (int ycoord = gridMinY; ycoord < gridMaxY; ycoord=ycoord+gridDelta)
            {
                CategorizeGridPoint(xcoord, ycoord, navMeshMask);
            }
        }
        DateTime endTime = DateTime.Now;
        gridCalcTime = (endTime - startTime).Milliseconds / 1000f;
        if (drawGizmos)
        {
            EditorWindow view = EditorWindow.GetWindow<SceneView>();
            view.Repaint();
        }
    }

    // Calculate and return the grid point value at one WaypointGrid point.
    private WaypointGridType CategorizeGridPoint(int gridX, int gridY, int areaMask)
    {
        Vector2Int gridPoint = Vector2Int.zero;
        gridPoint.x = gridX;
        gridPoint.y = gridY;
        Vector3 groundPoint = WaypointGridToGroundPoint(gridPoint);
        WaypointGridType calculatedGridType = GetCategoryOfGroundPoint(groundPoint, areaMask);
        _waypointGrid[gridPoint.x, gridPoint.y] = calculatedGridType;
        return calculatedGridType;
    }

    // Categorize the given point in world coordinates.
    private WaypointGridType GetCategoryOfGroundPoint(Vector3 groundPoint, int areaMask)
    {
        WaypointGridType gridType = WaypointGridType.Unknown;

        if (!GroundPointOnNavMesh(groundPoint, areaMask))
        {
            gridType = WaypointGridType.NotOnNavMesh;
        }
        else
        {
            /*
             * Look in 4 compass directions at threshold difference away.
             * Count how many of these are on the NavMesh and have an unobstructed raycast.
             */
            NavMeshHit hit;
            Vector3 groundpointN = groundPoint + Vector3.forward * corridorDistance;
            Vector3 groundpointW = groundPoint + Vector3.left * corridorDistance;
            Vector3 groundpointS = groundPoint + Vector3.back * corridorDistance;
            Vector3 groundpointE = groundPoint + Vector3.right * corridorDistance;
            if (hasTerrain)
            {
                groundpointN = MoveGroundPointToTerrain(groundpointN);
                groundpointW = MoveGroundPointToTerrain(groundpointW);
                groundpointS = MoveGroundPointToTerrain(groundpointS);
                groundpointE = MoveGroundPointToTerrain(groundpointE);
            }
            bool N_onNavMesh = GroundPointOnNavMesh(groundpointN, areaMask);
            bool W_onNavMesh = GroundPointOnNavMesh(groundpointW, areaMask);
            bool S_onNavMesh = GroundPointOnNavMesh(groundpointS, areaMask);
            bool E_onNavMesh = GroundPointOnNavMesh(groundpointE, areaMask);
            
            bool raycastN = true;  // Default value is that there is some obstruction on the raycast.
            if (N_onNavMesh) raycastN = NavMesh.Raycast(groundPoint, groundpointN, out hit, areaMask);
            bool N_hasCorridor = N_onNavMesh && !raycastN;
            
            bool raycastW = true;  // Default value is that there is some obstruction on the raycast.
            if (W_onNavMesh) raycastW = NavMesh.Raycast(groundPoint, groundpointW, out hit, areaMask);
            bool W_hasCorridor = W_onNavMesh && !raycastW;
            
            bool raycastS = true;  // Default value is that there is some obstruction on the raycast.
            if (S_onNavMesh) raycastS = NavMesh.Raycast(groundPoint, groundpointS, out hit, areaMask);
            bool S_hasCorridor = S_onNavMesh && !raycastS;
            
            bool raycastE = true;  // Default value is that there is some obstruction on the raycast.
            if (E_onNavMesh) raycastE = NavMesh.Raycast(groundPoint, groundpointE, out hit, areaMask);
            bool E_hasCorridor = E_onNavMesh && !raycastE;
            
            // Count the corridors.
            int numCorridors = 0;
            if (N_hasCorridor) numCorridors++;
            if (W_hasCorridor) numCorridors++;
            if (S_hasCorridor) numCorridors++;
            if (E_hasCorridor) numCorridors++;

            switch (numCorridors)
            {
                case 0:
                    gridType = WaypointGridType.Island;
                    break;
                case 1:
                    gridType = waypointsGenerated ? WaypointGridType.TerminusUsed : WaypointGridType.Terminus;
                    break;
                case 2:
                    if ((N_hasCorridor && S_hasCorridor) || (W_hasCorridor && E_hasCorridor))
                        gridType = WaypointGridType.Straight;
                    else
                        gridType = waypointsGenerated ? WaypointGridType.HubUsed : WaypointGridType.Hub;
                    break;
                case 3:
                    gridType = waypointsGenerated ? WaypointGridType.HubUsed : WaypointGridType.Hub;
                    break;
                case 4:
                    gridType = waypointsGenerated ? WaypointGridType.HubUsed : WaypointGridType.Hub;
                    break;
            }
            
        }
        return gridType;
    }

    // Determine whether the specified point in world coordinates is on the NavMesh.
    private bool GroundPointOnNavMesh(Vector3 groundPoint, int areaMask)
    {
        bool isOnNavMesh = false;
        NavMeshHit hit;
        isOnNavMesh = NavMesh.SamplePosition(groundPoint, out hit, navMeshProbe, areaMask);
        return isOnNavMesh;
    }
    
    /*
     * Automatically adjust the grid resolution based on the number of visible grid points.
     * This functionality is commented out / disabled.
     *
    public void AdjustGridResolution()
    {
        if (gridCount > gridResolutionThreshPosDir)
        {
            switch (gridResolution)
            {
                case 1:
                    gridResolution = 2;
                    if (!gridResolutionHeightThreshSet[0])
                    {
                        gridResolutionHeightThresh[0] = sceneCameraHeight;
                        gridResolutionHeightThreshSet[0] = true;
                    }
                    break;
                case 2:
                    gridResolution = 3;
                    if (!gridResolutionHeightThreshSet[1])
                    {
                        gridResolutionHeightThresh[1] = sceneCameraHeight;
                        gridResolutionHeightThreshSet[1] = true;
                    }
                    break;
                default:
                    gridResolution = 4;
                    if (!gridResolutionHeightThreshSet[2])
                    {
                        gridResolutionHeightThresh[2] = sceneCameraHeight;
                        gridResolutionHeightThreshSet[2] = true;
                    }
                    break;
            }
        }
        else // Automatically decrement based on scene height.
        {
            if (gridResolutionHeightThreshSet[0] && sceneCameraHeight < gridResolutionHeightThresh[0] * gridResolutionThreshCorrectionFactor)
            {
                gridResolution = 1;
                gridResolutionHeightThreshSet[0] = false;
                gridResolutionHeightThreshSet[1] = false;
                gridResolutionHeightThreshSet[2] = false;
            } 
            else if (gridResolutionHeightThreshSet[1] && sceneCameraHeight < gridResolutionHeightThresh[1] * gridResolutionThreshCorrectionFactor)
            {
                gridResolution = 2;
                gridResolutionHeightThreshSet[1] = false;
                gridResolutionHeightThreshSet[2] = false;
            } 
            else if (gridResolutionHeightThreshSet[2] && sceneCameraHeight < gridResolutionHeightThresh[2] * gridResolutionThreshCorrectionFactor)
            {
                gridResolution = 3;
                gridResolutionHeightThreshSet[2] = false;
            }
        }
    }
     * /

    /*
     * Functions to scan the waypoint grid and determine where to put waypoints.
     */
    
    public void ScanGridForWaypoints(WaypointGridType originalGridType, WaypointGridType replacementGridType,
                    WaypointGridType secondaryGridType, WaypointGridType secondaryReplacementGridType)
    {
        for (int x = TileMinX(); x < TileMaxX(); x++)
        {
            for (int y = TileMinY(); y < TileMaxY(); y++)
            {
                RectInt seedSquare = CreateGridSquare(x, y, minSquareSize);
                if (TestRectInGridOfType(seedSquare, originalGridType))
                {
                    // The seedSquare is all of Hub type in the WaypointGrid.  There will be a waypoint here.
                    // Determine whether the seedSquare can be expanded to cover more Hub grid points.
                    RectInt expandedSquare = ExpandSquare(seedSquare, originalGridType);
                    RectInt expandedRectangle = ExpandRectangle(expandedSquare, originalGridType);
                    // Get the waypoint position and markoff the region as HubUsed.
                    Vector2 waypointPosition = CreateWaypoint(expandedRectangle, replacementGridType);
                    // Create a waypoint.
                    Vector3 waypointWorldPosition = WaypointGridToWaypoint((int)waypointPosition.x, (int)waypointPosition.y);
                    if (waypointParent)
                        Instantiate(waypoint, waypointWorldPosition, Quaternion.identity, waypointParent);
                    else
                        Instantiate(waypoint, waypointWorldPosition, Quaternion.identity);
                    /*
                     * Expand the region, even if not rectangular, so that connected Hub points are no longer considered.
                     * Also mark off connected Terminus points.  Terminus points connected to Hub points should not generate waypoints.
                     */
                    MarkOffConnectedPoints(expandedRectangle, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
                }
            }
        }
    }
    
    // Create an initial grid square to determine whether it may be the seed for creating a waypoint.
    RectInt CreateGridSquare(int xcenter, int ycenter, int squareSize)
    {
        RectInt theSquare = new RectInt();
        int halfSize = squareSize / 2;  // If squareSize is odd, then halfSize is rounded down.
        theSquare.xMin = xcenter - halfSize;
        theSquare.yMin = ycenter - halfSize;
        theSquare.width = squareSize;
        theSquare.height = squareSize;
        return theSquare;
    }

    // Determine whether the test rectangle contains all points of gridType.
    bool TestRectInGridOfType(RectInt testRect, WaypointGridType gridType)
    {
        bool result = true;
        
        for (int x = testRect.xMin; x < testRect.xMax; x++)
        {
            if (x < TileMinX() || x > TileMaxX())
            {
                result = false;
            }
            else
            {
                for (int y = testRect.yMin; y < testRect.yMax; y++)
                {
                    if (y < TileMinY() || y > TileMaxY())
                    {
                        result = false;
                        break;
                    }
                    else if (_waypointGrid[x, y] != gridType)
                    {
                        result = false;
                        break;
                    }
                }
            }
        }
        return result;
    }

    // When this function is called, the inSquare is already known to be of gridType.  
    // This function tries to expand it while maintaining the square aspect ratio.
    RectInt ExpandSquare(RectInt inSquare, WaypointGridType gridType)
    {
        RectInt outSquare = inSquare;

        /*
         * Check on whether the square can be expanded in each direction.
         */
        bool xm = CanExpandXMinus(inSquare, gridType);
        bool ym = CanExpandYMinus(inSquare, gridType);
        bool xp = CanExpandXPlus(inSquare, gridType);
        bool yp = CanExpandYPlus(inSquare, gridType);
        bool xmym = CanExpandXMinusYMinus(inSquare, gridType);
        bool xmyp = CanExpandXMinusYPlus(inSquare, gridType);
        bool xpym = CanExpandXPlusYMinus(inSquare, gridType);
        bool xpyp = CanExpandXPlusYPlus(inSquare, gridType);

        /*
         * Can the square be expanded in all directions?
         */
        if (xm && xmym && ym && xpym && xp && xpyp && yp && xmyp)
        {
            outSquare.xMin--;
            outSquare.yMin--;
            outSquare.xMax++;
            outSquare.yMax++;
            // Try again recursively.
            outSquare = ExpandSquare(outSquare, gridType);
        }

        /*
         * Square cannot expand in all directions.
         * Can it expand in any two adjacent directions?  The will still maintain the square aspect ratio.
         */
        else if (xm && xmym && ym)
        {
            // Expand X minus / Y minus.
            outSquare.xMin--;
            outSquare.yMin--;
            // Try again recursively.
            outSquare = ExpandSquare(outSquare, gridType);
        }
        else if (xp && xpym && ym)
        {
            // Expand X plus / Y minus.
            outSquare.xMax++;
            outSquare.yMin--;
            // Try again recursively.
            outSquare = ExpandSquare(outSquare, gridType);
        }
        else if (xm && xmyp && yp)
        {
            // Expand X minus / Y plus.
            outSquare.xMin--;
            outSquare.yMax++;
            // Try again recursively.
            outSquare = ExpandSquare(outSquare, gridType);
        }
        else if (xp && xpyp && yp)
        {
            // Expand X plus / Y plus.
            outSquare.xMax++;
            outSquare.yMax++;
            // Try again recursively.
            outSquare = ExpandSquare(outSquare, gridType);
        }

        // Return the expanded square.
        return outSquare;
    }
    
    // When this function is called, the inRectangle is already known to be of gridType.  
    // This function tries to expand it to any rectangular shape.
    RectInt ExpandRectangle(RectInt inRectangle, WaypointGridType gridType)
    {
        RectInt outRectangle = inRectangle;

        /*
         * Can it expand in any direction?  These are not mutually exclusive, so each can be tested.
         */
        if (CanExpandXMinus(outRectangle, gridType))
        {
            // Expand X minus.
            outRectangle.xMin--;
            // Try again recursively.
            outRectangle = ExpandRectangle(outRectangle, gridType);
        }
        if (CanExpandXPlus(outRectangle, gridType))
        {
            // Expand X plus.
            outRectangle.xMax++;
            // Try again recursively.
            outRectangle = ExpandRectangle(outRectangle, gridType);
        }
        if (CanExpandYMinus(outRectangle, gridType))
        {
            // Expand Y minus.
            outRectangle.yMin--;
            // Try again recursively.
            outRectangle = ExpandRectangle(outRectangle, gridType);
        }
        if (CanExpandYPlus(outRectangle, gridType))
        {
            // Expand Y plus.
            outRectangle.yMax++;
            // Try again recursively.
            outRectangle = ExpandRectangle(outRectangle, gridType);
        }
        
        // Return the expanded rectangle.
        return outRectangle;
    }

    /*
     * Functions to determine whether a rectangle (including a square) can expand in one direction. 
     */
    
    bool CanExpandXMinus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMin <= TileMinX()) return false;
        int x = testRect.xMin - 1;
        for (int y = testRect.yMin; y <= testRect.yMax; y++)
        {
            if (_waypointGrid[x, y] != gridType) return false;
        }
        return true;
    }
    
    bool CanExpandXPlus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMax >= TileMaxX()) return false;
        int x = testRect.xMax + 1;
        for (int y = testRect.yMin; y <= testRect.yMax; y++)
        {
            if (_waypointGrid[x, y] != gridType) return false;
        }
        return true;
    }
    
    bool CanExpandYMinus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.yMin <= TileMinY()) return false;
        int y = testRect.yMin - 1;
        for (int x = testRect.xMin; x <= testRect.xMax; x++)
        {
            if (_waypointGrid[x, y] != gridType) return false;
        }
        return true;
    }

    bool CanExpandYPlus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.yMax >= TileMaxY()) return false;
        int y = testRect.yMax + 1;
        for (int x = testRect.xMin; x <= testRect.xMax; x++)
        {
            if (_waypointGrid[x, y] != gridType) return false;
        }

        return true;
    }

    /*
     * Functions to determine whether a rectangle (including a square) can expand to include a diagnonal point. 
     */

    bool CanExpandXMinusYMinus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMin <= TileMinX() || testRect.yMin <= TileMinY()) return false;
        return _waypointGrid[testRect.xMin - 1, testRect.yMin - 1] == gridType;
    }
    
    bool CanExpandXMinusYPlus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMin <= TileMinX() || testRect.yMax >= TileMaxY()) return false;
        return _waypointGrid[testRect.xMin - 1, testRect.yMax + 1] == gridType;
    }
    
    bool CanExpandXPlusYMinus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMax >= TileMaxX() || testRect.yMin <= TileMinY()) return false;
        return _waypointGrid[testRect.xMax + 1, testRect.yMin - 1] == gridType;
    }
    
    bool CanExpandXPlusYPlus(RectInt testRect, WaypointGridType gridType)
    {
        if (testRect.xMax >= TileMaxX() || testRect.yMax >= TileMaxY()) return false;
        return _waypointGrid[testRect.xMax + 1, testRect.yMax + 1] == gridType;
    }

    // Return the center of the specified rectangle as the position for a waypoint.  
    // Set each grid point of the rectangle to newGridType.

    Vector2 CreateWaypoint(RectInt rectangle, WaypointGridType newGridType)
    {
        // Return the center of the rectangle as the new waypoint position.
        for (int x = rectangle.xMin; x <= rectangle.xMax; x++)
        {
            for (int y = rectangle.yMin; y <= rectangle.yMax; y++)
            {
                // Change the type of each grid point, so that this rectangle is not further considered.
                _waypointGrid[x, y] = newGridType;
            }
        }

        return rectangle.center;
    }

    // Mark-off the grid points connected to a waypoint-generating region so that extra "spurious" waypoints are not generated.
    void MarkOffConnectedPoints(RectInt rectangle, WaypointGridType originalGridType, WaypointGridType replacementGridType,
        WaypointGridType secondaryGridType, WaypointGridType secondaryReplacementGridType)
    {
        // Grow along each edge of the rectangle.
        if (rectangle.xMin > TileMinX())
        {
            int x = rectangle.xMin - 1;
            for (int y=rectangle.yMin; y<=rectangle.yMax; y++)
                MarkOffPoint(x, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
        }
        if (rectangle.yMin > TileMinY())
        {
            int y = rectangle.yMin - 1;
            for (int x=rectangle.xMin; x<=rectangle.xMax; x++)
                MarkOffPoint(x, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
        }
        if (rectangle.xMax < TileMaxX())
        {
            int x = rectangle.xMax + 1;
            for (int y=rectangle.yMin; y<=rectangle.yMax; y++)
                MarkOffPoint(x, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
        }
        if (rectangle.yMax < TileMaxY())
        {
            int y = rectangle.yMax + 1;
            for (int x=rectangle.xMin; x<=rectangle.xMax; x++)
                MarkOffPoint(x, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
        }
    }

    /*
     * Starting with the seed point (x,y), mark off all connected points recursively.
     * WaypointGrid[x,y] == originalGridType (e.g. Hub) => Replace with replacementGridType (e.g. HubUsed), call recursively.
     * WaypointGrid[x,y] == secondaryGridType (e.g. Terminus) =>
     *                                          Replace with secondaryReplacementGridType (e.g. TerminusUsed),
     *                                          call recursively but only for the seconardaryGridType.
     */
    void MarkOffPoint(int x, int y, WaypointGridType originalGridType, WaypointGridType replacementGridType, 
                            WaypointGridType secondaryGridType, WaypointGridType secondaryReplacementGridType)
    {
        if (x < TileMinX() || x > TileMaxX() || y < TileMinY() || y > TileMaxY()) return;
        if (_waypointGrid[x, y] == originalGridType)
        {
            _waypointGrid[x, y] = replacementGridType;
            MarkOffPoint(x-1, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
            MarkOffPoint(x, y-1, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
            MarkOffPoint(x+1, y, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
            MarkOffPoint(x, y+1, originalGridType, replacementGridType, secondaryGridType, secondaryReplacementGridType);
        }
        else if (secondaryGridType != WaypointGridType.None && _waypointGrid[x, y] == secondaryGridType)
        {
            _waypointGrid[x, y] = secondaryReplacementGridType;
            MarkOffPoint(x-1, y, secondaryGridType, secondaryReplacementGridType, WaypointGridType.None, WaypointGridType.None);
            MarkOffPoint(x, y-1, secondaryGridType, secondaryReplacementGridType, WaypointGridType.None, WaypointGridType.None);
            MarkOffPoint(x+1, y, secondaryGridType, secondaryReplacementGridType, WaypointGridType.None, WaypointGridType.None);
            MarkOffPoint(x, y+1, secondaryGridType, secondaryReplacementGridType, WaypointGridType.None, WaypointGridType.None);
            
        }
    }
    
    
    /*
     * Functions for converting among coordinate systems.
     * 
     * Converstion from grid to world coordinates:
     *     x-world = x-grid * agent-radius - x-extent + x-bounds-center
     *     x-grid = (x-world + x-extent - x-bounds-center) / agent-radius
     * 
     * Rules for converting between 2D grid and 3D world:
     * - The XY of the 2D grid can be converted to the XZ (ground plane) of the world with simple conversions.
     * - The Y (altitude) of the ground plane is trickier, because sometimes we want this to be at ground level
     *   for example to determine whether the point is on the NavMesh, and sometimes we want this to be at the
     *   "waypoint level" (slightly above ground) for raycasting without intersecting the ground itself.
     * - The convention will be that converting from 2D grid to 3D world will provide the 3D point on the ground
     *   and there will be a function to convert from ground point to the "waypoint" point which will be higher.
     */
    
    private Vector3 WaypointGridToWaypoint(int xcoord, int ycoord)
    {
        Vector3 groundPoint = WaypointGridToGroundPoint(xcoord, ycoord);
        return GroundPointToWaypoint(groundPoint);
    }

    private Vector3 WaypointGridToGroundPoint(int xcoord, int ycoord)
    {
        Vector2Int gridPoint = Vector2Int.zero;
        gridPoint.x = xcoord;
        gridPoint.y = ycoord;
        return WaypointGridToGroundPoint(gridPoint);
    }
    
    private Vector3 WaypointGridToGroundPoint(Vector2Int gridPoint)
    {
        Vector3 groundPoint = Vector3.zero;
        groundPoint.x = gridPoint.x * agentRadius - navMeshBounds.extents.x + navMeshBounds.center.x;
        groundPoint.z = gridPoint.y * agentRadius - navMeshBounds.extents.z + navMeshBounds.center.z;
        // Get the height of the point.
        groundPoint.y = GroundLevelAtCoordinates(gridPoint.x, gridPoint.y);
        return groundPoint;
    }
    
    private Vector3 GroundPointToWaypoint(Vector3 groundPoint)
    {
        Vector3 waypointPos;
        waypointPos.x = groundPoint.x;
        waypointPos.y = groundPoint.y + waypointElevation;
        waypointPos.z = groundPoint.z;
        return waypointPos;
    }

    private Vector3 MoveGroundPointToTerrain(Vector3 groundPoint)
    {
        Vector3 terrainGroundPoint = groundPoint;
        terrainGroundPoint.y = GroundLevelAtCoordinates(groundPoint.x, groundPoint.z);
        return terrainGroundPoint;
    }

    // Return the ground / elevation level at a ground / world point.
    private float GroundLevelAtCoordinates(float x, float z)
    {
        if (hasTerrain)
        {
            Vector3 worldPoint;
            worldPoint.x = x;
            worldPoint.y = 0f; // Don't know what the height is yet.
            worldPoint.z = z;
            return Terrain.activeTerrain.SampleHeight(worldPoint);
        }
        else
        {
            // If not a terrain, assume the ground is a flat plane.  
            // Could expand this to do a raycast for more complex ground configurations.
            return groundLevel;
        }
    }
    
    // Convert point from world coordinates to waypoint grid coordinates.
    private int WorldPointXToGridX(float worldX)
    {
        return (int) ((worldX + navMeshBounds.extents.x - navMeshBounds.center.x) / agentRadius);
    }
    
    private int WorldPointZToGridY(float worldZ)
    {
        return (int)((worldZ + navMeshBounds.extents.z - navMeshBounds.center.z) / agentRadius);
    }
    
    // Get the min / max coordinates of the tile being analyzed, which may be the entire NavMesh.
    private int TileMinX()
    {
        return calculatingSubset ? WorldPointXToGridX(groundProjection.xMin) : 0;
    }
    private int TileMinY()
    {
        return calculatingSubset ? WorldPointZToGridY(groundProjection.yMin) : 0;
    }
    private int TileMaxX()
    {
        return calculatingSubset ? WorldPointXToGridX(groundProjection.xMax) : gridXSize-1;
    }
    private int TileMaxY()
    {
        return calculatingSubset ? WorldPointZToGridY(groundProjection.yMax) : gridYSize-1;
    }

    private Rect BoundsToRect(Bounds b)
    {
        // The y dimension of the bounds is not used.
        Rect theRect = new Rect(b.min.x, b.min.z, 2*b.extents.x, 2*b.extents.z);
        return theRect;
    }

    // Not used.
    /*
    public WaypointGridType WaypointGridTypeAtWorldPoint(Vector3 worldPoint)
    {
        Vector2Int gridCoords = WorldPointToWaypointGrid(worldPoint);
        return WaypointGrid[gridCoords.x, gridCoords.y];
    }

    public Vector2Int WorldPointToWaypointGrid(Vector3 worldPoint)
    {
        // This works whether worldPoint is a groundpoint or a waypoint, because worldPoint.y is not used.
        Vector2Int waypointGridPoint = Vector2Int.zero;
        waypointGridPoint.x = WorldPointXToGridX(worldPoint.x);
        waypointGridPoint.y = WorldPointZToGridY(worldPoint.z);
        return waypointGridPoint;
    }

    public Vector3 WaypointToGroundPoint(Vector3 waypoint)
    {
        Vector3 groundPoint;
        groundPoint.x = waypoint.x;
        groundPoint.y = waypoint.y - waypointElevation;
        groundPoint.z = waypoint.z;
        return groundPoint;
    }

    public float WaypointLevelAtCoordinates(float x, float z)
    {
        float gLevel = GroundLevelAtCoordinates(x, z);
        return gLevel + waypointElevation;
    }
    */

#endif

}
