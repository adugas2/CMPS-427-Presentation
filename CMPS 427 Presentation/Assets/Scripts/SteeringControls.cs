using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UIElements;

public class SteeringControls : MonoBehaviour
{
    public enum SteeringType { Direct, Seek, Flee, Arrive, Pursue, Evade, Hide, Wander }

    public Transform target;

    [Header("Physics")]
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;
    public Vector3 desiredVelocity;
    public Vector3 steeringForce;

    [Header("Maxes")]
    public float maxSpeed = 3;
    public float maxForce = 1;
    public float maxObstacleForce = 5;

    [Header("Arrive")]
    public float arrivalDistance = 4;

    [Header("Pursuit")]
    public float pursuitAheadTime = 1;

    [Header("Wander")]
    public float wanderDistance = 5;
    public float wanderRadius = 2;
    public float wanderVariance = 90;

    [Header("Obstacles")]
    public List<Transform> obstacles;
    public float hideDistance;
    public bool avoidObstacles;
    public float sightDistance = 5;


    public SteeringType steeringType;
    public Dictionary<SteeringType, Action> steeringUpdates;

    // Start is called before the first frame update
    void Start()
    {
        steeringUpdates = new Dictionary<SteeringType, Action>();
        steeringUpdates.Add(SteeringType.Seek, SeekUpdate);
        steeringUpdates.Add(SteeringType.Flee, FleeUpdate);
        steeringUpdates.Add(SteeringType.Direct, DirectUpdate);
        steeringUpdates.Add(SteeringType.Arrive, ArriveUpdate);
        steeringUpdates.Add(SteeringType.Pursue, PursueUpdate);
        steeringUpdates.Add(SteeringType.Evade, EvadeUpdate);
        steeringUpdates.Add(SteeringType.Hide, HideUpdate);
        steeringUpdates.Add(SteeringType.Wander, WanderUpdate);

        position = transform.position;
        velocity = Vector3.zero;
        acceleration = Vector3.zero;

        target = GameObject.FindGameObjectWithTag("Target").transform;
    }

    // Update is called once per frame
    void Update()
    {
        acceleration = Vector3.zero;

        steeringUpdates[steeringType]();

        if (avoidObstacles) AvoidObstacles();

        acceleration = Vector3.ProjectOnPlane(acceleration, Vector3.up);
        velocity = Vector3.ClampMagnitude(velocity + acceleration * Time.deltaTime, maxSpeed);
        position = position + velocity * Time.deltaTime;
        transform.position = position;


        if (velocity != Vector3.zero) transform.forward = velocity.normalized;
    }

    private void DirectUpdate()
    {
        // Acceleration just brings it straight to the target
        acceleration += Vector3.ClampMagnitude(target.position - transform.position, maxForce);
    }

    private void SeekUpdate()
    {
        // Desired velocity is the direction from me to the target
        desiredVelocity = (target.position - transform.position).normalized * maxSpeed;

        steeringForce = Vector3.ClampMagnitude(desiredVelocity - velocity, maxForce);

        acceleration += steeringForce;
    }
    private void FleeUpdate()
    {
        desiredVelocity = (transform.position - target.position).normalized * maxSpeed;

        steeringForce = Vector3.ClampMagnitude(desiredVelocity - velocity, maxForce);

        acceleration += steeringForce;
    }

    private void PursueUpdate()
    {
        desiredVelocity = (target.position - transform.position).normalized + target.GetComponent<Rigidbody>().velocity * pursuitAheadTime;

        steeringForce = Vector3.ClampMagnitude(desiredVelocity - velocity, maxForce);

        acceleration += steeringForce;
    }
    private void EvadeUpdate()
    {
        desiredVelocity = (transform.position - (target.position + target.GetComponent<Rigidbody>().velocity * pursuitAheadTime)).normalized * maxSpeed;
        //Vector3 desiredVelocity = (transform.position - target.posiation).normalized + target.GetComponent<Rigidbody>().velocity * pursuitAheadTime;

        steeringForce = Vector3.ClampMagnitude(desiredVelocity - velocity, maxForce);

        acceleration += steeringForce;
    }

    private void ArriveUpdate()
    {
        steeringForce = ArriveAtTargetPos(target.position);

        acceleration += steeringForce;
    }

    private Vector3 ArriveAtTargetPos(Vector3 targetPos)
    {
        desiredVelocity = (targetPos - transform.position).normalized * maxSpeed;
        Debug.DrawRay(transform.position, (targetPos - transform.position).normalized * maxSpeed);

        float dist = Vector3.Distance(targetPos, transform.position);
        if (dist < arrivalDistance) desiredVelocity *= (dist / arrivalDistance);

        Vector3 sf = Vector3.ClampMagnitude(desiredVelocity - velocity, maxForce);

        return sf;
    }

    Vector3[] hidePoses = new Vector3[0];
    float[] distances = new float[0];
    Vector3 hidePose = Vector3.zero;
    private void HideUpdate()
    {
        hidePoses = new Vector3[obstacles.Count];
        distances = new float[obstacles.Count];
        // Find the hiding position for each obstacle and the distance to each
        for (int i = 0; i < obstacles.Count; i++)
        {
            // Hiding position is the obstacle position plus direction from the target to the obstacle, multiplied by hiding distance
            Transform ob = obstacles[i];
            hidePoses[i] = ob.position + (ob.position - target.position).normalized * hideDistance;
            hidePoses[i].y = transform.position.y;

            distances[i] = Vector3.Distance(transform.position, hidePoses[i]);
        }

        // Find the smallest distance
        int minIndex = 0;
        float minDistance = float.MaxValue;
        for (int i = 0; i < hidePoses.Length; i++)
        {
            if (distances[i] < minDistance)
            {
                minDistance = distances[i];
                minIndex = i;
            }
        }

        hidePose = hidePoses[minIndex];

        if (minDistance < 0.1f) { acceleration = Vector3.zero; velocity = Vector3.zero; return; } // Just stop if we're super close

        // Then "arrive" to closest hiding position
        steeringForce = ArriveAtTargetPos(hidePose);

        acceleration += steeringForce;
    }

    float wanderAngle;
    Vector3 wanderTarget;
    private void WanderUpdate()
    {
        // Project a circle out front of the player, calculate a position on it using wanderAngle
        Vector3 circleCenter = transform.position + transform.forward * wanderDistance;
        Vector3 posOnCircle = new Vector3(wanderRadius * Mathf.Sin(wanderAngle), 0, wanderRadius * Mathf.Cos(wanderAngle));
        wanderTarget = circleCenter + posOnCircle;

        // Adjust the angle by some random amount to vary the motion
        wanderAngle += (UnityEngine.Random.Range(-wanderVariance + 2, wanderVariance) * Time.deltaTime);

        steeringForce = ArriveAtTargetPos(wanderTarget);

        acceleration += steeringForce;
    }

    private void AvoidObstacles()
    {
        // Creates some force that steers us away from any obstacles we see
        List<Transform> seenObstacles = new List<Transform>();
        foreach (var ob in obstacles)
        {
            float dotProd = Vector3.Dot(ob.position - transform.position, velocity);
            if (dotProd > 0) seenObstacles.Add(ob);
        }

        // Exted out 2 rays from sides
        Ray leftRay = new Ray(transform.position + transform.right * (-transform.localScale.x / 2), velocity.normalized);
        Ray rightRay = new Ray(transform.position + transform.right * (transform.localScale.x / 2), velocity.normalized);
        RaycastHit hit;

        // If the left one hits, steer away from the left
        if (Physics.Raycast(leftRay, out hit, sightDistance))
        {
            if (obstacles.Contains(hit.transform))
            {
                // Steer moves away from the area we hit
                Vector3 dirToTarget = hit.point - transform.position;// origin;
                Vector3 obstacleForce = (leftRay.direction * sightDistance - dirToTarget) * maxObstacleForce;
                Debug.DrawRay(hit.point, obstacleForce, Color.cyan);
                steeringForce += obstacleForce;
                acceleration += obstacleForce;
            }
        }
        else if (Physics.Raycast(rightRay, out hit, sightDistance))
        {
            if (obstacles.Contains(hit.transform))
            {
                // Steer moves away from the area we hit
                Vector3 dirToTarget = hit.point - transform.position; // rightRay.origin;
                Vector3 obstacleForce = (rightRay.direction * sightDistance - dirToTarget) * maxObstacleForce;
                Debug.DrawRay(hit.point, obstacleForce, Color.cyan);
                steeringForce += obstacleForce;
                acceleration += obstacleForce;
            }
        }

    }
}
