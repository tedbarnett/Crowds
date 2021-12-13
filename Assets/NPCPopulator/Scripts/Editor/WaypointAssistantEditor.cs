using System;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(WaypointAssistant))]

public class AutomaticWaypointToolEditor : Editor
{
    private WaypointAssistant _target;
    private readonly GUIStyle _warningStyle= new GUIStyle();
    private readonly GUIStyle _sadStyle= new GUIStyle();
    private readonly GUIStyle _happyStyle = new GUIStyle();

    void OnEnable()
    {
        _target = (WaypointAssistant) target;
        
        _warningStyle.normal.textColor = Color.red;
        _warningStyle.fontStyle = FontStyle.BoldAndItalic;
        _warningStyle.alignment = TextAnchor.MiddleRight;
        
        _sadStyle.normal.textColor = Color.red;
        _sadStyle.fontStyle = FontStyle.BoldAndItalic;
        _sadStyle.alignment = TextAnchor.MiddleCenter;
        
        _happyStyle.normal.textColor = Color.green;
        _happyStyle.fontStyle = FontStyle.BoldAndItalic;
        _happyStyle.alignment = TextAnchor.MiddleCenter;
    }

    public override void OnInspectorGUI()
    {
        //DrawDefaultInspector();
        
        /********* Environment Parameters **********/

        EditorGUILayout.LabelField("NavMesh and Waypoint Grid", EditorStyles.boldLabel);
        
        // Agent Radius
        EditorGUI.BeginChangeCheck();
        float newAgentRadius = EditorGUILayout.FloatField("Agent Radius:", _target.agentRadius);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Agent Radius");
            _target.agentRadius = newAgentRadius;
        }

        // Refresh and show the bounds of the NavMesh
        EditorGUILayout.BeginHorizontal();
        Bounds b = _target.navMeshBounds;
        EditorGUILayout.LabelField("NavMesh: (" + b.min[0].ToString("0.") + "," + b.min[2].ToString("0.") +
                                   ") to (" + b.max[0].ToString("0.") + "," + b.max[2].ToString("0.") + ")");
        EditorGUILayout.LabelField("Grid Dimensions: (" + _target.gridXSize + "," + _target.gridYSize + ")");
        EditorGUILayout.EndHorizontal();

        if (GUILayout.Button("Create Waypoint Grid"))
        {
            _target.CalculateBoundsOfNavMesh();
        }
        
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Environment Parameters", EditorStyles.boldLabel);

        // Waypoint Object
        EditorGUI.BeginChangeCheck();
        GameObject newWaypoint = (GameObject) EditorGUILayout.ObjectField("Waypoint Object:", _target.waypoint, typeof(GameObject), true);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Waypoint");
            _target.waypoint = newWaypoint;
        }

        // Waypoint Parent
        EditorGUI.BeginChangeCheck();
        Transform newWaypointParent = (Transform) EditorGUILayout.ObjectField("Waypoint Parent:", _target.waypointParent, typeof(Transform), true);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Waypoint Parent");
            _target.waypointParent = newWaypointParent;
        }
        
        // NavMesh Mask
        EditorGUI.BeginChangeCheck();
        string[] maskNames = GameObjectUtility.GetNavMeshAreaNames();
        int newNavMeshMask = EditorGUILayout.MaskField("NavMesh Mask:", _target.navMeshMask, maskNames);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "NavMesh Mask");
            _target.navMeshMask = newNavMeshMask;
        }

        // Has Terrain
        EditorGUI.BeginChangeCheck();
        bool newHasTerrain = EditorGUILayout.Toggle("Has Terrain:", _target.hasTerrain);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Has Terrain");
            _target.hasTerrain = newHasTerrain;
        }

        // Ground Level
        EditorGUI.BeginChangeCheck();
        float newGroundLevel = EditorGUILayout.FloatField("Ground Level:", _target.groundLevel);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Ground Level");
            _target.groundLevel = newGroundLevel;
        }

        // Waypoint Level
        EditorGUI.BeginChangeCheck();
        float newWaypointElevation = EditorGUILayout.FloatField("Waypoint Elevation:", _target.waypointElevation);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Waypoint Elevation");
            _target.waypointElevation = newWaypointElevation;
        }
        
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Waypoint Parameters", EditorStyles.boldLabel);

        // Corridor Distance
        EditorGUI.BeginChangeCheck();
        float newCorridorDistance = EditorGUILayout.Slider("Corridor Distance:", _target.corridorDistance, 1f, 30f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Corridor Distance");
            _target.corridorDistance = newCorridorDistance;
            _target.CalculateGroundProjection();
        }
        
        // NavMesh Probe
        EditorGUI.BeginChangeCheck();
        float newNavmeshProbe = EditorGUILayout.Slider("NavMesh Probe:", _target.navMeshProbe, 0f, 1f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "NavMesh Probe");
            _target.navMeshProbe = newNavmeshProbe;
            _target.CalculateGroundProjection();
        }

        // Whether to generate terminus waypoints.
        EditorGUI.BeginChangeCheck();
        bool newWaypointsFromTerminus = EditorGUILayout.Toggle("Terminus Waypoints:", _target.waypointsFromTerminus);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Waypoints from Terminus");            
            _target.waypointsFromTerminus = newWaypointsFromTerminus;
        }

        /********* Waypoint Grid Parameters **********/

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Waypoint Grid Calculations", EditorStyles.boldLabel);

        // Enable Interactive
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Calculate in Scene View"))
        {
            _target.CalculateGroundProjection();
        }
        EditorGUI.BeginChangeCheck();
        bool newEnableInteractive = EditorGUILayout.ToggleLeft("Automate Scene View Change", _target.enableInteractive);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Automate Calculations");
            _target.enableInteractive = newEnableInteractive;
        }
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.LabelField("In Scene View:  (" + _target.groundProjection.xMin.ToString("0.") + "," +
                                   _target.groundProjection.yMin.ToString("0.") + ") to (" +
                                   _target.groundProjection.xMax.ToString("0.") + "," +
                                   _target.groundProjection.yMax.ToString("0.") + ")");

        // minSquareSize
        EditorGUI.BeginChangeCheck();
        int newMinSquareSize= EditorGUILayout.IntSlider("Min Square Size:", _target.minSquareSize, 1, 5);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Min Square Size");
            _target.minSquareSize = newMinSquareSize;
        }
        

        EditorGUILayout.Space();
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Generate Waypoints"))
        {
            DateTime startTime = DateTime.Now;
            _target.waypointsGenerated = false;
            _target.CalculateEntireNavMesh();
            _target.ScanGridForWaypoints(WaypointAssistant.WaypointGridType.Hub, WaypointAssistant.WaypointGridType.HubUsed, WaypointAssistant.WaypointGridType.Terminus, WaypointAssistant.WaypointGridType.TerminusUsed);
            if (_target.waypointsFromTerminus)
                _target.ScanGridForWaypoints(WaypointAssistant.WaypointGridType.Terminus, WaypointAssistant.WaypointGridType.TerminusUsed, WaypointAssistant.WaypointGridType.None, WaypointAssistant.WaypointGridType.None);
            _target.waypointsGenerated = true;
            DateTime endTime = DateTime.Now;
            _target.waypointGenTime = (endTime - startTime).Milliseconds / 1000f;
        }
        if (GUILayout.Button("Delete Waypoints"))
        {
            if (_target.waypointParent)
            {
                int numChildren = _target.waypointParent.childCount;
                for (int i = numChildren - 1; i >= 0; i--)
                {
                    // Delete that child.
                    GameObject aChild = _target.waypointParent.GetChild(i).gameObject;
                    DestroyImmediate(aChild);
                }
                _target.waypointsGenerated = false;
                _target.ResetWaypointGrid();
            }
            else
            {
                Debug.LogError("No waypoint parent set.");
            }
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.LabelField("Delete with Caution!", _warningStyle);

        /********* Tool Performance Parameters **********/

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Tool Performance Parameters", EditorStyles.boldLabel);
        
        EditorGUI.BeginChangeCheck();
        int newSkipFrames = EditorGUILayout.IntSlider("Frames to Skip:", _target.skipFrames, 1, 60);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Skip Frames");
            _target.skipFrames = newSkipFrames;
        }

        EditorGUI.BeginChangeCheck();
        int newGridResolution = EditorGUILayout.IntSlider("Grid Resolution:", _target.gridResolution, 1, 4);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "Grid Resolution");
            _target.gridResolution = newGridResolution;
        }

        GUIStyle gridCalcPerfStyle = _target.gridCalcPerfWarning ? _sadStyle : _happyStyle;
        EditorGUILayout.LabelField("Grid Calculation Performance", gridCalcPerfStyle);
        
        EditorGUI.BeginChangeCheck();
        bool newDrawGizmos = EditorGUILayout.Toggle("Draw Gizmos:", _target.drawGizmos);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(_target, "DrawGizmos");
            _target.drawGizmos = newDrawGizmos;
            if (newDrawGizmos)
            {
                // Force a repaint.
                EditorWindow view = EditorWindow.GetWindow<SceneView>();
                view.Repaint();
            }
        }

        GUIStyle gizmoDrawPerfStyle = _target.gizmoDrawPerfWarning ? _sadStyle : _happyStyle;
        EditorGUILayout.LabelField("Gizmo Draw Performance", gizmoDrawPerfStyle);

        /********* Developer Controls **********/

        /*
         // Don't provide the toggle for developer info.  This can be uncommented to show these additional fields.
        EditorGUILayout.Space();
        _target.developerMode = EditorGUILayout.ToggleLeft("Developer Controls and Metrics:", _target.developerMode);
        */

        if (_target.developerMode)
        {
            EditorGUILayout.Space();
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Toggle("Waypoints Generated:", _target.waypointsGenerated);
            EditorGUILayout.Toggle("Calculating Subset:", _target.calculatingSubset);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();
        
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Metrics", EditorStyles.boldLabel);

            EditorGUILayout.LabelField("Grid Count: ", _target.gridCount.ToString());
            EditorGUILayout.Slider("Gizmo Time: ", _target.gizmoDrawTime, 0f, 1f);
            EditorGUILayout.Slider("Grid Reset Time: ", _target.gridResetTime, 0f, 1f);
            EditorGUILayout.Slider("Grid Calc Time: ", _target.gridCalcTime, 0f, 1f);
            EditorGUILayout.Slider("WaypointGen Time: ", _target.waypointGenTime, 0f, 5f);
            
            /*
             * Automatic resolution adjustment is disabled / commented out.
             * 
            _target.automaticGridResolution = EditorGUILayout.Toggle("Auto-adjust grid resolution:", _target.automaticGridResolution);
            _target.gridResolutionThreshPosDir = EditorGUILayout.IntField("Grid Res Thresh + Dir: ", _target.gridResolutionThreshPosDir);
            _target.gridResolutionThreshCorrectionFactor = EditorGUILayout.FloatField("Threshold correction factor: ",
                _target.gridResolutionThreshCorrectionFactor);
            //_target.gridResolutionThreshNegDir = EditorGUILayout.IntField("Grid Res Thresh - Dir: ", _target.gridResolutionThreshNegDir);

            EditorGUILayout.LabelField("Height Thresholds:");
            EditorGUILayout.FloatField(_target.gridResolutionHeightThresh[0]);
            EditorGUILayout.FloatField(_target.gridResolutionHeightThresh[1]);
            EditorGUILayout.FloatField(_target.gridResolutionHeightThresh[2]);
            EditorGUILayout.LabelField("Height Thresholds Set:");
            EditorGUILayout.Toggle(_target.gridResolutionHeightThreshSet[0]);
            EditorGUILayout.Toggle(_target.gridResolutionHeightThreshSet[1]);
            EditorGUILayout.Toggle(_target.gridResolutionHeightThreshSet[2]);
            EditorGUILayout.FloatField("Scene Camera Height: ", _target.sceneCameraHeight);
             */
            
        }
        
    }

}
