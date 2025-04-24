using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Net.Sockets;
using System.Text;
using System;
using System.Collections.Generic;

public class VRControllerPointer : MonoBehaviour
{
    // VR Controller Settings
    public OVRInput.Controller controllerType = OVRInput.Controller.RTouch;
    public LineRenderer pointerLine;
    public float maxPointerDistance = 100f;
    public Text coordinatesDisplay;

    // Server Settings
    private string ngrokHost = "6.tcp.ngrok.io";
    private int ngrokPort = 18168;
    private int request_count = 0;
    private float responseTimeout = 3f;
    private Vector3 lastSentPosition;

    // Articulation Bodies to Control
    public ArticulationBody[] articulationBodies = new ArticulationBody[7];

    // Zero position angles
    private float[] zeroPositionAngles = { 0, 20, 0, 20, 0, 0, 0 };

    // Internal variables
    private Vector3 lastHitPosition;
    private GameObject lastHitObject;
    private bool triggerPressed = false;
    private TcpClient client;
    private NetworkStream stream;
    private float responseStartTime;
    private bool waitingForResponse = false;
    private Vector3 currentTargetPosition;
    private bool hasCurrentPosition = false;

    [Header("Input Settings")]
    public float triggerCooldown = 0.3f;  // Minimum time between trigger presses
    private float lastTriggerPressTime = -1f;
    private bool joystickClickTriggered = false;
    private bool gripButtonTriggered = false;

    // Joystick control settings
    public float joystickDeadzone = 0.5f;
    public float joystickCooldown = 0.3f;
    private float lastJoystickMoveTime = -1f;
    private Vector2 lastJoystickInput = Vector2.zero;

    public GameObject debugSpherePrefab;
    private List<GameObject> debugSpheres = new List<GameObject>();
    private Color[] debugColors = { Color.red, Color.blue, Color.green, Color.yellow, Color.cyan, Color.magenta };
    private int colorIndex = 0;

    // Reference to the OVRCameraRig
    private Transform trackingSpace;

    void Start()
    {
        // Find the OVRCameraRig/TrackingSpace in the scene
        GameObject cameraRig = GameObject.Find("OVRCameraRig");
        if (cameraRig != null)
        {
            trackingSpace = cameraRig.transform.Find("TrackingSpace");
            if (trackingSpace == null)
            {
                trackingSpace = cameraRig.transform;
                Debug.Log("Using OVRCameraRig as tracking reference");
            }
            else
            {
                Debug.Log("Using TrackingSpace as tracking reference");
            }
        }
        else
        {
            Debug.LogWarning("OVRCameraRig not found in scene. Using this object's transform as reference.");
            trackingSpace = transform;
        }

        InitializePointer();
        ConnectToServer();
    }

    void Update()
    {
        UpdatePointer();
        HandleControllerInput();
        HandleJoystickInput();
    }

    private void HandleJoystickInput()
    {
        // Check for joystick click - now only does the joint rotations without return to home
        bool joystickClick = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, controllerType);
        if (joystickClick && !joystickClickTriggered)
        {
            joystickClickTriggered = true;

            // Get current angles
            var drive4 = articulationBodies[3].xDrive;
            float currentAngle4 = drive4.target * Mathf.Deg2Rad;

            var drive6 = articulationBodies[5].xDrive;
            float currentAngle6 = drive6.target * Mathf.Deg2Rad;

            // Calculate new angles
            float rotationAmount4 = -5.0f * Mathf.Deg2Rad;
            float rotationAmount6 = -5.0f * Mathf.Deg2Rad;

            float newAngle4 = currentAngle4 + rotationAmount4;
            float newAngle6 = currentAngle6 + rotationAmount6;

            // Apply rotations
            SetArticulationDrive(articulationBodies[3], newAngle4);
            SetArticulationDrive(articulationBodies[5], newAngle6);

            if (coordinatesDisplay != null)
            {
                coordinatesDisplay.text = $"Joints rotated by +3°";
            }
        }
        else if (!joystickClick && joystickClickTriggered)
        {
            joystickClickTriggered = false; // Reset trigger state but don't take any action
        }

        // Check for joystick movement (x-axis for joint 1, y-axis for joints 4 and 6)
        Vector2 joystickInput = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, controllerType);

        // Only process if outside deadzone and cooldown has elapsed
        if ((joystickInput.magnitude > joystickDeadzone) &&
            (Time.time - lastJoystickMoveTime > joystickCooldown))
        {
            // X-axis movement controls first articulation body
            if (Mathf.Abs(joystickInput.x) > joystickDeadzone &&
                Mathf.Abs(joystickInput.x) != Mathf.Abs(lastJoystickInput.x)&& Mathf.Abs(joystickInput.y)<0.2)
            {
                if (articulationBodies.Length > 0 && articulationBodies[0] != null)
                {
                    // Get current angle
                    var drive = articulationBodies[0].xDrive;
                    float currentAngle = drive.target * Mathf.Deg2Rad;

                    // Calculate new angle - use a value that depends on joystick magnitude
                    float rotationAmount = -3.0f * Mathf.Sign(joystickInput.x) * Mathf.Deg2Rad;
                    float newAngle = currentAngle + rotationAmount;

                    // Apply rotation
                    SetArticulationDrive(articulationBodies[0], newAngle);

                    if (coordinatesDisplay != null)
                    {
                        coordinatesDisplay.text = $"Joint 1 rotated by {rotationAmount * Mathf.Rad2Deg:F1}°";
                    }

                    lastJoystickMoveTime = Time.time;
                }
            }

            // Y-axis movement controls joints 4 and 6
            if (Mathf.Abs(joystickInput.y) > joystickDeadzone &&
                Mathf.Abs(joystickInput.y) != Mathf.Abs(lastJoystickInput.y) && Mathf.Abs(joystickInput.x) < 0.2)
            {
                if (articulationBodies.Length >= 5 && articulationBodies[3] != null &&
                    articulationBodies.Length >= 7 && articulationBodies[5] != null)
                {
                    // Get current angles
                    var drive4 = articulationBodies[3].xDrive;
                    float currentAngle4 = drive4.target * Mathf.Deg2Rad;

                    var drive6 = articulationBodies[5].xDrive;
                    float currentAngle6 = drive6.target * Mathf.Deg2Rad;

                    var drive2 = articulationBodies[1].xDrive;
                    float currentAngle2 = drive2.target * Mathf.Deg2Rad;

                    // Calculate new angles
                    float rotationAmount4 = 3.0f * Mathf.Sign(joystickInput.y) * Mathf.Deg2Rad;
                    float rotationAmount6 = 3.0f * Mathf.Sign(joystickInput.y) * Mathf.Deg2Rad;
                    float rotationAmount2 = 3.0f * Mathf.Sign(joystickInput.y) * Mathf.Deg2Rad;

                    float newAngle4 = currentAngle4 + rotationAmount4;
                    float newAngle6 = currentAngle6 + rotationAmount6;
                    float newAngle2 = currentAngle2 + rotationAmount2;

                    // Apply rotations
                    SetArticulationDrive(articulationBodies[3], newAngle4);
                    //SetArticulationDrive(articulationBodies[5], newAngle6);
                    SetArticulationDrive(articulationBodies[1], newAngle2);

                    if (coordinatesDisplay != null)
                    {
                        coordinatesDisplay.text = $"Joints rotated by {rotationAmount4 * Mathf.Rad2Deg:F1}°";
                    }

                    lastJoystickMoveTime = Time.time;
                }
            }
        }

        // Store last joystick state
        lastJoystickInput = joystickInput;
    }

    private void MoveToZeroPosition()
    {
        Debug.Log("Moving all joints to zero position");

        if (zeroPositionAngles != null && zeroPositionAngles.Length == articulationBodies.Length)
        {
            for (int i = 0; i < articulationBodies.Length; i++)
            {
                if (articulationBodies[i] != null)
                {
                    float angle = zeroPositionAngles[i];
                    SetArticulationDrive(articulationBodies[i], angle);
                }
            }

            if (coordinatesDisplay != null)
            {
                coordinatesDisplay.text = "All joints zeroed.";
            }

            // Reset current position tracking
            hasCurrentPosition = false;
        }
    }

    private void UpdatePointer()
    {
        // Get controller position and rotation using OVRInput
        Vector3 localPosition = OVRInput.GetLocalControllerPosition(controllerType);
        Quaternion localRotation = OVRInput.GetLocalControllerRotation(controllerType);

        // Convert to world space
        Vector3 position = trackingSpace.TransformPoint(localPosition);
        Quaternion rotation = trackingSpace.rotation * localRotation;

        Ray ray = new Ray(position, rotation * Vector3.forward);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, maxPointerDistance))
        {
            lastHitPosition = hit.point;
            lastHitObject = hit.collider.gameObject;

            if (pointerLine != null)
            {
                pointerLine.SetPosition(0, position);
                pointerLine.SetPosition(1, hit.point);
            }

            if (coordinatesDisplay != null && !hasCurrentPosition)
            {
                coordinatesDisplay.text = $"Object: {hit.collider.gameObject.name}\n" +
                                       $"Hit Point: {hit.point.x:F2}, {hit.point.y:F2}, {hit.point.z:F2}\n" +
                                       $"Object Position: {hit.collider.transform.position.x:F2}, {hit.collider.transform.position.y:F2}, {hit.collider.transform.position.z:F2}";
            }
        }
        else
        {
            Vector3 endPoint = position + (rotation * Vector3.forward) * maxPointerDistance;
            lastHitObject = null;

            if (pointerLine != null)
            {
                pointerLine.SetPosition(0, position);
                pointerLine.SetPosition(1, endPoint);
            }

            if (coordinatesDisplay != null && !hasCurrentPosition)
            {
                coordinatesDisplay.text = $"Position: {endPoint.x:F2}, {endPoint.y:F2}, {endPoint.z:F2}";
            }
        }
    }

    private void OnTriggerPressed()
    {
        // Get fresh controller data using OVRInput
        Vector3 localPosition = OVRInput.GetLocalControllerPosition(controllerType);
        Quaternion localRotation = OVRInput.GetLocalControllerRotation(controllerType);

        // Convert to world space
        Vector3 position = trackingSpace.TransformPoint(localPosition);
        Quaternion rotation = trackingSpace.rotation * localRotation;

        Ray ray = new Ray(position, rotation * Vector3.forward);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, maxPointerDistance))
        {
            lastHitPosition = hit.point;
            lastHitObject = hit.collider.gameObject;

            // Create debug sphere at hit point
            CreateDebugSphere(hit.point, "Hit Point");

            // Check if the hit object is a plane
            bool isPlane = IsPlane(lastHitObject);

            // Decide which position to send
            Vector3 positionToSend;

            if (isPlane)
            {
                // If it's a plane, use the exact hit point
                positionToSend = hit.point + new Vector3(0f, 0.3f, 0f);
                CreateDebugSphere(positionToSend, "Plane Hit Point");
            }
            else if (lastHitObject != null)
            {
                // For non-plane objects, use the previous behavior
                positionToSend = lastHitObject.transform.position + new Vector3(0f, 0.3f, 0f);
                CreateDebugSphere(positionToSend, "Object Position (y+0.3)");
            }
            else
            {
                positionToSend = lastHitPosition + new Vector3(0f, 0.2f, 0f);
            }

            // Store the current target position for joystick adjustments
            currentTargetPosition = positionToSend;
            hasCurrentPosition = true;

            if (coordinatesDisplay != null)
            {
                if (isPlane)
                {
                    coordinatesDisplay.text = $"Sending plane hit: {positionToSend.x:F3}, {positionToSend.y:F3}, {positionToSend.z:F3}";
                }
                else
                {
                    coordinatesDisplay.text = $"Sending: {positionToSend.x:F3}, {positionToSend.y:F3}, {positionToSend.z:F3}" +
                                            (lastHitObject != null ? " (object y+0.3)" : "");
                }
            }

            SendPositionToServer(positionToSend);
        }
        else
        {
            lastHitPosition = position + (rotation * Vector3.forward) * maxPointerDistance;
            CreateDebugSphere(lastHitPosition, "Max Distance Position");

            // Store the current target position for joystick adjustments
            currentTargetPosition = lastHitPosition;
            hasCurrentPosition = true;

            if (coordinatesDisplay != null)
            {
                coordinatesDisplay.text = $"Sending: {lastHitPosition.x:F3}, {lastHitPosition.y:F3}, {lastHitPosition.z:F3}";
            }

            SendPositionToServer(lastHitPosition);
        }
    }

    // Helper method to check if an object is a plane
    // You can customize this logic based on your scene structure
    private bool IsPlane(GameObject obj)
    {
        if (obj == null) return false;

        // Check by name (common naming conventions)
        if (obj.name.Contains("Plane") || obj.name.Contains("Floor") || obj.name.Contains("Ground"))
        {
            return true;
        }

        // Check by tag if you've tagged your planes
        if (obj.CompareTag("Plane"))
        {
            return true;
        }

        // Check by component (if your planes use specific components)
        if (obj.GetComponent<MeshRenderer>() != null)
        {
            // Check if the mesh is relatively flat
            Mesh mesh = obj.GetComponent<MeshFilter>()?.sharedMesh;
            if (mesh != null)
            {
                // Simple check: if the y-extent of the mesh is very small compared to x and z
                Vector3 size = mesh.bounds.size;
                if (size.y < 0.1f && (size.x > 1.0f && size.z > 1.0f))
                {
                    return true;
                }
            }
        }

        // Add any other checks that would identify your planes

        return false; // Default to false if none of the above checks passed
    }
    private void CreateDebugSphere(Vector3 position, string label)
    {
        // Create sphere object - either from prefab or primitive
        GameObject sphere;

        if (debugSpherePrefab != null)
        {
            sphere = Instantiate(debugSpherePrefab, position, Quaternion.identity);
        }
        else
        {
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = position;
            Destroy(sphere.GetComponent<Collider>()); // Remove collider to avoid interference
        }

        sphere.transform.localScale = Vector3.one * 0.05f; // 5cm diameter

        // Set color
        Color sphereColor = debugColors[colorIndex % debugColors.Length];
        sphere.GetComponent<Renderer>().material = new Material(Shader.Find("Standard"));
        sphere.GetComponent<Renderer>().material.color = sphereColor;
        colorIndex++;

        // Add label
        sphere.name = $"{label} {Time.time:F2}s ({sphereColor})";

        debugSpheres.Add(sphere);

        // Auto-cleanup old spheres after 10 seconds
        Destroy(sphere, 10f);
        if (debugSpheres.Count > 20)
        {
            Destroy(debugSpheres[0]);
            debugSpheres.RemoveAt(0);
        }

        Debug.Log($"Created debug sphere at {position} with color {sphereColor}");
    }

    private void SendPositionToServer(Vector3 position)
    {
        Debug.Log($"Attempting to send position: {position}");
        if (waitingForResponse) return; // Don't send if already waiting
        if (client == null || !client.Connected)
        {
            Debug.LogWarning("Not connected - attempting reconnect...");
            ConnectToServer();
            if (client == null || !client.Connected)
            {
                Debug.LogError("Reconnect failed");
                return;
            }
        }

        try
        {
            request_count++;
            string jsonData = $"{{\"position\":[{position.x:F4},{position.y:F4},{position.z:F4}], \"request_id\":{request_count}}}";
            Debug.Log("Sending JSON: " + jsonData);

            byte[] data = Encoding.ASCII.GetBytes(jsonData);
            stream.Write(data, 0, data.Length);

            // Store this as the last position sent to the server
            lastSentPosition = position;

            waitingForResponse = true;
            responseStartTime = Time.time;
            StartCoroutine(WaitForServerResponse());
        }
        catch (Exception e)
        {
            Debug.LogError($"Send error: {e.Message}\n{e.StackTrace}");
            HandleNetworkError("Send error: " + e.Message);
        }
    }

    private void InitializePointer()
    {
        if (pointerLine != null)
        {
            pointerLine.positionCount = 2;
            pointerLine.startWidth = 0.01f;
            pointerLine.endWidth = 0.01f;
            pointerLine.startColor = Color.green;
            pointerLine.endColor = Color.red; // Color changes along length
            pointerLine.material = new Material(Shader.Find("Sprites/Default"));
        }
    }

    private void ConnectToServer()
    {
        try
        {
            client = new TcpClient();
            client.ReceiveTimeout = (int)(responseTimeout * 1000);
            client.Connect(ngrokHost, ngrokPort);
            stream = client.GetStream();
            Debug.Log("Connected to server");
        }
        catch (Exception e)
        {
            Debug.LogError("Connection error: " + e.Message);
        }
    }

    private void CloseConnection()
    {
        try
        {
            if (stream != null)
            {
                stream.Close();
                stream = null;
            }
            if (client != null)
            {
                client.Close();
                client = null;
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error closing connection: " + e.Message);
        }
    }

    private void HandleNetworkError(string message)
    {
        Debug.LogError(message);
        waitingForResponse = false;
        CloseConnection();
        ConnectToServer();
    }

    private void HandleControllerInput()
    {
        // Check for trigger press using OVRInput
        bool triggerValue = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, controllerType);

        // Only register a new press if:
        // 1. Trigger is pressed (triggerValue = true)
        // 2. Not already pressing (triggerPressed = false)
        // 3. Not waiting for a server response (!waitingForResponse)
        // 4. Enough time has passed since the last press (cooldown)
        if (triggerValue && !triggerPressed && !waitingForResponse &&
            (Time.time - lastTriggerPressTime > triggerCooldown))
        {
            triggerPressed = true;
            lastTriggerPressTime = Time.time; // Record the press time
            OnTriggerPressed(); // Send position ONCE
        }
        // Reset the pressed state when the trigger is released
        else if (!triggerValue && triggerPressed)
        {
            triggerPressed = false;
        }

        // Check for grip button press to reset to zero position
        bool gripValue = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger, controllerType);

        if (gripValue && !gripButtonTriggered)
        {
            // Get current angles
            var drive4 = articulationBodies[3].xDrive;
            float currentAngle4 = drive4.target * Mathf.Deg2Rad;

            var drive6 = articulationBodies[5].xDrive;
            float currentAngle6 = drive6.target * Mathf.Deg2Rad;

            // Calculate new angles
            float rotationAmount4 = 5.0f * Mathf.Deg2Rad;
            float rotationAmount6 = 5.0f * Mathf.Deg2Rad;

            float newAngle4 = currentAngle4 + rotationAmount4;
            float newAngle6 = currentAngle6 + rotationAmount6;

            // Apply rotations
            SetArticulationDrive(articulationBodies[3], newAngle4);
            SetArticulationDrive(articulationBodies[5], newAngle6);
        }
        else if (!gripValue && gripButtonTriggered)
        {
            gripButtonTriggered = false;
        }
    }

    private IEnumerator WaitForServerResponse()
    {
        byte[] buffer = new byte[1024];
        StringBuilder responseBuilder = new StringBuilder();
        bool responseComplete = false;
        Exception caughtException = null;
        float startTime = Time.time;

        while (!responseComplete && waitingForResponse && client.Connected)
        {
            try
            {
                if (stream.DataAvailable)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    responseBuilder.Append(Encoding.ASCII.GetString(buffer, 0, bytesRead));

                    if (responseBuilder.ToString().Contains("}"))
                    {
                        responseComplete = true;
                    }
                }
            }
            catch (Exception e)
            {
                caughtException = e;
                break;
            }

            if (Time.time - startTime > responseTimeout)
            {
                caughtException = new TimeoutException("Response timeout");
                break;
            }

            yield return null;
        }

        if (caughtException != null)
        {
            HandleNetworkError("Receive error: " + caughtException.Message);
            yield break;
        }

        if (responseComplete)
        {
            waitingForResponse = false;
            ProcessServerResponse(responseBuilder.ToString());
        }
    }

    private void ProcessServerResponse(string response)
    {
        try
        {
            Debug.Log("Processing response: " + response);

            // Parse angles
            int anglesStart = response.IndexOf("\"angles\": [") + "\"angles\": [".Length;
            int anglesEnd = response.IndexOf("]", anglesStart);
            string anglesStr = response.Substring(anglesStart, anglesEnd - anglesStart);
            string[] angleValues = anglesStr.Split(',');

            if (angleValues.Length == articulationBodies.Length)
            {
                // Apply the angles
                for (int i = 0; i < articulationBodies.Length; i++)
                {
                    if (articulationBodies[i] != null)
                    {
                        float angle = float.Parse(angleValues[i].Trim());
                        SetArticulationDrive(articulationBodies[i], angle);
                    }
                }

                Debug.Log("Updated articulation bodies with angles");
            }
            else
            {
                Debug.LogError($"Received {angleValues.Length} angles but expected {articulationBodies.Length}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Response parsing error: " + e.Message);
        }
    }

    private void SetArticulationDrive(ArticulationBody body, float targetAngle)
    {
        if (body == null) return;

        var drive = body.xDrive;
        drive.target = targetAngle * Mathf.Rad2Deg;
        body.xDrive = drive;
    }

    void OnDestroy()
    {
        CloseConnection();
    }
}