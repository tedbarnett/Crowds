using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(npcFactory))]
public class npcFactoryEditor : Editor {

    private npcFactory myFactory;
    private readonly GUIStyle sadStyle= new GUIStyle();
    private readonly GUIStyle happyStyle = new GUIStyle();
    private bool canCreateNPCs = true;

    int numberToCreate;

    void OnEnable()
    {
        myFactory = (npcFactory)target;
        sadStyle.normal.textColor = Color.red;
        happyStyle.normal.textColor = Color.green;
    }

    public override void OnInspectorGUI()
    {
        //DrawDefaultInspector();
        
        if (myFactory.npcModels.Count == 0)
        {
            //Debug.LogError("You must specify at least one NPC Model.");
            canCreateNPCs = false;
        }

        // Create NPC
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Create NPCs", EditorStyles.boldLabel);

        EditorGUI.BeginChangeCheck();
        int newNumberToCreate = EditorGUILayout.IntSlider("Number To Create:", myFactory.numberToCreate, 1, 50);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Number to Create");
            myFactory.numberToCreate = newNumberToCreate;
        }

        if (canCreateNPCs)
        {
            if (GUILayout.Button("Create " + myFactory.numberToCreate + " NPCs"))
            {
                Undo.SetCurrentGroupName("Create NPCs");
                int undoGroup = Undo.GetCurrentGroup();
                for (int i = 0; i < myFactory.numberToCreate; i++)
                {
                    GameObject newNpc = myFactory.CreateNpc();
                    Undo.RegisterCreatedObjectUndo(newNpc, "Create NPC");
                }
                Undo.CollapseUndoOperations(undoGroup);
            }
        }
        else
        {
            EditorGUILayout.LabelField("You must fix missing data before creating NPCs.", sadStyle);
        }
        
        // Initialize the canCreateNPCs flag to true, but down below set it to false if any object references are missing.
        canCreateNPCs = true;
        
        EditorGUILayout.Space();
    
        // NPC Parent
        EditorGUI.BeginChangeCheck();
        Transform newNPCparent = (Transform) EditorGUILayout.ObjectField("NPC Parent:", myFactory.npcParent, typeof(Transform), true);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "NPC Parent");
            myFactory.npcParent = newNPCparent;
        }

        EditorGUILayout.Space();

        // NavMesh Bounds
        EditorGUILayout.BeginHorizontal();
        GUIStyle boundsStyle;
        if (myFactory.navMeshBounds.extents == Vector3.zero)
        {
            boundsStyle = sadStyle;
            canCreateNPCs = false;
        }
        else
        {
            boundsStyle = happyStyle;
        }
        EditorGUILayout.LabelField("NavMesh Bounds:", boundsStyle, GUILayout.MaxWidth(120));
        Bounds tempNavMeshBounds = new Bounds(Vector3.zero, Vector3.zero);
        EditorGUI.BeginChangeCheck();
        if (GUILayout.Button("Refresh"))
        {
            tempNavMeshBounds = myFactory.CalculateBoundsOfNavMesh();
        }
        if (GUILayout.Button("Clear"))
        {
            tempNavMeshBounds = myFactory.ClearNavMeshBounds();
        }
        EditorGUILayout.EndHorizontal();
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "NavMesh Bounds");
            myFactory.navMeshBounds = tempNavMeshBounds;
            myFactory.RefreshRanges();
        }

        EditorGUI.BeginChangeCheck();
        Bounds editedNavMeshBounds = EditorGUILayout.BoundsField(myFactory.navMeshBounds);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "NavMesh Bounds");
            myFactory.navMeshBounds = editedNavMeshBounds;
            myFactory.RefreshRanges();
        }
        //EditorGUILayout.LabelField("navMeshRange: ", myFactory.navMeshRange.ToString());
        //EditorGUILayout.LabelField("range: ", myFactory.range.ToString());

        // NavMesh Mask
        EditorGUI.BeginChangeCheck();
        string[] maskNames = GameObjectUtility.GetNavMeshAreaNames();
        int newNavMeshMask = EditorGUILayout.MaskField("NavMesh Mask:", myFactory.navMeshMask, maskNames);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "NavMesh Mask");
            myFactory.navMeshMask = newNavMeshMask;
        }

        EditorGUILayout.Space();

        // Animator Controller
        EditorGUILayout.BeginHorizontal();
        EditorGUI.BeginChangeCheck();
        GUIStyle controllerStyle;
        if (myFactory.animatorController)
        {
            controllerStyle = happyStyle;
        }
        else
        {
            controllerStyle = sadStyle;
            canCreateNPCs = false;
        }
        EditorGUILayout.LabelField("Controller", controllerStyle, GUILayout.MaxWidth(100));
        RuntimeAnimatorController newController = (RuntimeAnimatorController) EditorGUILayout.ObjectField(myFactory.animatorController, typeof(RuntimeAnimatorController), true);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Set Animator Controller");
            myFactory.animatorController = newController;
        }
        EditorGUI.BeginChangeCheck();
        EditorGUILayout.EndHorizontal();

        // Waypoint Parent
        EditorGUILayout.BeginHorizontal();
        EditorGUI.BeginChangeCheck();
        GUIStyle wpParentStyle;
        if (myFactory.waypointParent)
        {
            wpParentStyle = happyStyle;
        }
        else
        {
            wpParentStyle = sadStyle;
            canCreateNPCs = false;
        }
        EditorGUILayout.LabelField("Waypoint Parent", wpParentStyle, GUILayout.MaxWidth(100));
        GameObject newWaypointParent = (GameObject)EditorGUILayout.ObjectField(myFactory.waypointParent, typeof(GameObject), true);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Set Waypoint Parent");
            myFactory.waypointParent = newWaypointParent;
        }
        EditorGUI.BeginChangeCheck();
        EditorGUILayout.EndHorizontal();
        
        // NPC Radius
        EditorGUI.BeginChangeCheck();
        float newNpcRadius = EditorGUILayout.Slider("NPC Radius:", myFactory.npcRadius, 0.0f, 1.0f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "NPC Radius");
            myFactory.npcRadius = newNpcRadius;
        }

        // Stopping Distance
        EditorGUI.BeginChangeCheck();
        float newStoppingDistance = EditorGUILayout.Slider("Stopping Distance:", myFactory.stoppingDistance, 0.5f, 2.0f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Stopping Distance");
            myFactory.stoppingDistance = newStoppingDistance;
        }

        // Speed Variation
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Speed Variation", EditorStyles.boldLabel);

        EditorGUI.BeginChangeCheck();
        float newUsualSpeed = EditorGUILayout.Slider("Usual Speed:", myFactory.usualSpeed, 0.4f, 3.0f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Usual Speed");
            myFactory.usualSpeed = newUsualSpeed;
        }

        EditorGUI.BeginChangeCheck();
        float newSpeedVariation = EditorGUILayout.Slider("Speed Variation:", myFactory.usualSpeedVariation, 0.0f, 2.0f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Speed Variation");
            myFactory.usualSpeedVariation = newSpeedVariation;
        }
        
        EditorGUI.BeginChangeCheck();
        int newMinFramesToWait = EditorGUILayout.IntSlider("Min Frames to Wait:", myFactory.minFramesToWait, 0, 100);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Min Frames");
            myFactory.minFramesToWait = newMinFramesToWait;
        }
        
        EditorGUI.BeginChangeCheck();
        int newMaxFramesToWait = EditorGUILayout.IntSlider("Max Frames to Wait:", myFactory.maxFramesToWait, 0, 500);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Max Frames");
            myFactory.maxFramesToWait = newMaxFramesToWait;
        }
        
        // Waypoint Range
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Waypoint Range", EditorStyles.boldLabel);

        EditorGUI.BeginChangeCheck();
        bool newLimitRange = EditorGUILayout.Toggle("Limit Range:", myFactory.limitRange);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(myFactory, "Limit Range");
            myFactory.limitRange = newLimitRange;
        }

        if (myFactory.limitRange)
        {
            EditorGUI.BeginChangeCheck();
            float newRange = EditorGUILayout.Slider("Waypoint Range:", myFactory.range, Mathf.Round(myFactory.navMeshRange*0.1f), Mathf.Round(myFactory.navMeshRange*1.1f));
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(myFactory, "Waypoint Range");
                myFactory.range = newRange;
            }
        }

        // NPC Models
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("NPC Models", EditorStyles.boldLabel);
        if (GUILayout.Button("Add New NPC Model"))
        {
            Undo.RecordObject(myFactory, "Add Model");
            myFactory.AddNpcModel();
        }
        for (int i = 0; i < myFactory.npcModels.Count; i++)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUI.BeginChangeCheck();
            GUIStyle modelStyle;
            if (myFactory.npcModels[i])
            {
                modelStyle = happyStyle;
            }
            else
            {
                modelStyle = sadStyle;
                canCreateNPCs = false;
            }
            EditorGUILayout.LabelField("NPC Model:", modelStyle, GUILayout.MaxWidth(100));
            GameObject newModel = (GameObject) EditorGUILayout.ObjectField(myFactory.npcModels[i],
                typeof(GameObject), true);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(myFactory, "Set Model");
                myFactory.npcModels[i] = newModel;
            }
            if (GUILayout.Button("-", GUILayout.MaxWidth(30)))
            {
                Undo.RecordObject(myFactory, "Remove Model");
                myFactory.RemoveNpcModel(i);
            }
            EditorGUILayout.EndHorizontal();
        }

        
    }
}

