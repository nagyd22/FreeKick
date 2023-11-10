using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;

public class GoalShootingAgent : Agent
{
    public GameObject ball;
    public GameObject boundaryBehind;

    [SerializeField] private Transform targetTransform;
    private Transform ballTransform;
    private Rigidbody ballRigidbody;

    private float initialDistance;

    private bool kicked;

    public override void OnEpisodeBegin()
    {
        ballTransform = ball.transform;
        ballRigidbody = ball.GetComponent<Rigidbody>();
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.velocity = new Vector3(0, 0, 0);

        // 3 fixed place where the agent will start to help the training
        Vector3[] possibleStartPositions = new Vector3[]
        {
        new Vector3(10, -3, -22),
        new Vector3(-10, -3, -22),
        new Vector3(0, -3, -22)
        };

        int choosenStartingPoint = UnityEngine.Random.Range(0, possibleStartPositions.Length);
        Vector3 startPosition = possibleStartPositions[choosenStartingPoint];

        // Assign the shooter and the ball to the random position
        transform.localPosition = startPosition;
        ballTransform.localPosition = new Vector3(startPosition.x, 0.65f, startPosition.z - 0.7f);

        // Assign the boundary behind the shooter
        boundaryBehind.transform.localPosition = new Vector3(boundaryBehind.transform.localPosition.x, boundaryBehind.transform.localPosition.y, startPosition.z + 3f);

        initialDistance = Vector3.Distance(ballTransform.localPosition, transform.localPosition);
        kicked = false;
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // The position of the ball
        sensor.AddObservation(ballTransform.localPosition.x);
        sensor.AddObservation(ballTransform.localPosition.y);

        // The position of the football gate
        sensor.AddObservation(targetTransform.localPosition.x);
        sensor.AddObservation(targetTransform.localPosition.y);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float actionXRotate = actionTaken[0];
        float actionYRotate = actionTaken[1];
        float actionPower = actionTaken[2];

        float angleX = actionXRotate * 40f;
        float angleY = actionYRotate * 60f;
        float strength = actionPower * 10f;
        KickBall(angleX, angleY, strength);

        bool isCloseToGoal = IsCloseToGoal();
        bool isStopped = IsStopped();

        if(isCloseToGoal && kicked && isStopped)
        {
            AddReward(0.5f);
        }
        else if (kicked && isStopped)
        {
            AddReward(-0.1f);
        }
        AddReward(-0.1f / MaxStep);
    }

    private bool IsStopped()
    {
        if (kicked && ballRigidbody.IsSleeping())
        {
            return true;
        }
        else { return false; }
    }

    private void KickBall(float angleX, float angleY, float strength)
    {
        float actualDistance = Vector3.Distance(ballTransform.localPosition, transform.localPosition);

        if (initialDistance + 0.01 >= actualDistance)
        {
            kicked = true;
            // Apply rotations around both X and Y axes to the ballTransform
            ballTransform.localRotation = Quaternion.Euler(-angleX, angleY, 0);

            // Calculate the direction and force
            Vector3 kickDirection = ballTransform.forward;
            Vector3 kickForce = kickDirection * strength;

            ballRigidbody.AddForce(kickForce, ForceMode.Impulse);
        }
        else return;
    }

    private bool IsCloseToGoal()
    {
        bool isCloseToTarget = Vector3.Distance(ballTransform.localPosition, targetTransform.localPosition) < 3.5f;

        return isCloseToTarget;
    }

    public void GoalTriggered()
    {
        // The ball hits the gameobject which has goal tag on it
        AddReward(1f);
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        EndEpisode();
    }

    public void BoundaryTriggered()
    {
        // The ball hits one of the gameobjects which has boundary tag on it
        AddReward(-1f);
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        EndEpisode();
    }

}
