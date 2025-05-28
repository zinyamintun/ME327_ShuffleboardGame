using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
    public Camera mainCamera;
    public float launchForceMultiplier = 1f;
    public float maxForce = 100f;
    public float mass = 0.345f;
    public float kineticFriction = 0.8f;
    public float stopSpeed = 0.03f;

    private Rigidbody rb;
    private bool isHeld = false;
    private bool hasFallen = false;
    private bool launched = false;
    public bool isActivePuck = false;
    private float timeSinceLaunch = 0f;

    private Queue<Vector3> mouseHistory = new Queue<Vector3>();
    private Queue<float> timeHistory = new Queue<float>();
    private const int historyLength = 5;

    private Vector3 currentMouseWorldPos;
    private Vector3 launchVelocity;

    private Plane groundPlane;

    public float rightLimit = 0.25f;
    public float leftLimit = -0.25f;
    public float frontLimit = -2.4f;
    public float backLimit = 2.4f;
    public float maxHoldX = 1.75f;

    public float rightOuter = 0.278f;
    public float leftOuter = -0.278f;
    public float frontOuter = -2.553f;
    public float backOuter = 2.553f;
    public float bottom = -0.002f;

    private Vector3 grabOffset;


    void Start()
    {
        rb = GetComponent<Rigidbody>();

        rb.mass = mass;

        if (mainCamera == null)
            mainCamera = Camera.main;

        groundPlane = new Plane(Vector3.up, 0.0201f);
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

        rb.useGravity = false;
        rb.isKinematic = true;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    void Update()
    {
        if (!isHeld && !hasFallen && IsOutOfBounds())
        {
            FallOffTable();
        }

        if (hasFallen)
        {
            float clampedX = Mathf.Clamp(transform.position.x, frontOuter, backOuter);
            float clampedZ = Mathf.Clamp(transform.position.z, leftOuter, rightOuter);
            transform.position = new Vector3(clampedX, transform.position.y, clampedZ);

            if (transform.position.y < bottom)
            {
                StopAtBottom();
            }
        }
    }

    void FixedUpdate()
    {
        float speed = rb.velocity.magnitude;
        if (speed > 1e-4f)
        {
            Vector3 friction = -rb.velocity.normalized * kineticFriction * mass * Physics.gravity.magnitude;
            rb.AddForce(friction, ForceMode.Force);
        }

        if ((launched || isActivePuck) && !hasFallen)
        {
            timeSinceLaunch += Time.fixedDeltaTime;

            if (speed < stopSpeed && timeSinceLaunch > 0.2f)
            {
                Debug.Log("Stopping...");
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation;
            }
        }
    }

    Vector3 GetMouseWorldPosition()
    {
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (groundPlane.Raycast(ray, out float enter))
        {
            return ray.GetPoint(enter);
        }
        return transform.position;
    }

    void OnMouseDown()
    {
        if (!isActivePuck || !gameObject.activeSelf) return;
        if (!rb.isKinematic) return; 

        isHeld = true;
        rb.isKinematic = true;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

        mouseHistory.Clear();
        timeHistory.Clear();

        Vector3 startPos = GetMouseWorldPosition();
        mouseHistory.Enqueue(startPos);
        timeHistory.Enqueue(Time.time);

        grabOffset = transform.position - startPos;
    }

    void OnMouseDrag()
    {
        if (!isActivePuck) return;
        if (isHeld)
        {
            currentMouseWorldPos = GetMouseWorldPosition();
            mouseHistory.Enqueue(currentMouseWorldPos);
            timeHistory.Enqueue(Time.time);

            if (mouseHistory.Count > historyLength)
            {
                mouseHistory.Dequeue();
                timeHistory.Dequeue();
            }

            Vector3 targetPosition = currentMouseWorldPos + grabOffset;

            float clampedX = Mathf.Clamp(targetPosition.x, maxHoldX, backLimit);
            float clampedZ = Mathf.Clamp(targetPosition.z, leftLimit, rightLimit);

            Vector3 newPosition = new Vector3(clampedX, transform.position.y, clampedZ);
            transform.position = newPosition;
        }
    }

    void OnMouseUp()
    {
        if (!isActivePuck) return;
        if (isHeld)
        {
            Debug.Log("Mouse released – attempting launch");
            isHeld = false;
            rb.isKinematic = false;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.constraints = RigidbodyConstraints.None;

            if (mouseHistory.Count >= 2)
            {
                Vector3 startPos = mouseHistory.Peek();
                float startTime = timeHistory.Peek();

                Vector3 endPos = currentMouseWorldPos;
                float endTime = Time.time;

                float dt = endTime - startTime;
                if (dt < 1e-4f) dt = Time.fixedDeltaTime;

                Vector3 velocity = (endPos - startPos) / dt;
                velocity.y = 0;

                launchVelocity = Vector3.ClampMagnitude(velocity * launchForceMultiplier, maxForce);
                Vector3 impulse = launchVelocity * mass;

                Debug.Log($"Sampled velocity = {velocity:F3}, Launch = {launchVelocity:F3}, Time = {dt:F3}");

                rb.AddForce(impulse, ForceMode.Impulse);
                Debug.Log("Impulse applied: " + impulse + ", Resulting velocity: " + rb.velocity);

                timeSinceLaunch = 0f;
                launched = true;
                isActivePuck = false;

            }

        }
    }

    bool IsOutOfBounds()
    {
        Vector3 pos = transform.position;

        if (isHeld)
        {
            return pos.z < leftLimit || pos.z > rightLimit || pos.x < frontLimit || pos.x > backLimit;
        }
        else
        {
            return pos.z < leftOuter || pos.z > rightOuter || pos.x < frontOuter || pos.x > backOuter;
        }
    }

    void FallOffTable()
    {
        hasFallen = true;

        rb.constraints = RigidbodyConstraints.FreezePositionX | RigidbodyConstraints.FreezePositionZ | RigidbodyConstraints.FreezeRotation;
        rb.useGravity = true;

        StartCoroutine(WaitToReachBottom());
    }

    IEnumerator WaitToReachBottom()
    {
        while (transform.position.y > bottom)
        {
            yield return null;
        }

        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeAll;
        transform.position = new Vector3(transform.position.x, bottom, transform.position.z);

    }

    void StopAtBottom()
    {
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeAll;
        transform.position = new Vector3(transform.position.x, bottom, transform.position.z);
        this.enabled = false;

    }

    public bool HasFallen()
    {
        return hasFallen;
    }

    public bool HasLaunched()
    {
        return launched;
    }



}


