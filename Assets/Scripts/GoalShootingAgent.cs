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

    private const float maxDistance = 22f;

    private bool kicked;
    private bool episodeHasBegun;

    public override void OnEpisodeBegin()
    {
        episodeHasBegun = true;
        ballTransform = ball.transform;
        ballRigidbody = ball.GetComponent<Rigidbody>();

        // 3 fixed place where the agent will start to help the training
        Vector3[] possibleStartPositions = new Vector3[]
        {
        new Vector3(10, -3, -20),
        new Vector3(-10, -3, -20),
        new Vector3(0, -3, -20)
        };

        int choosenStartingPoint = UnityEngine.Random.Range(0, possibleStartPositions.Length);
        Vector3 startPosition = possibleStartPositions[choosenStartingPoint];

        // Assign the shooter and the ball to the random position
        transform.localPosition = startPosition;
        ballTransform.localPosition = new Vector3(startPosition.x, -3.36f, startPosition.z - 0.7f);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.constraints = RigidbodyConstraints.FreezeRotation;

        // Assign the boundary behind the shooter
        boundaryBehind.transform.localPosition = new Vector3(boundaryBehind.transform.localPosition.x, boundaryBehind.transform.localPosition.y, startPosition.z + 2f);

        kicked = false;
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // The position of the football gate
        sensor.AddObservation(targetTransform.localPosition.x);
        sensor.AddObservation(targetTransform.localPosition.z);

        // The position of the player
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float actionXRotate = Math.Clamp(actionTaken[0], 0, 1);
        float actionYRotate = actionTaken[1];
        float actionPower = Math.Clamp(actionTaken[2], 0, 1);

        float angleX = actionXRotate * 30f;
        float angleY = actionYRotate * 50f;
        float strength = actionPower * 12f;
        KickBall(angleX, angleY, strength);

    }

    private void FixedUpdate()
    {
        if(episodeHasBegun)
        {
            bool isStopped = IsStopped();
            bool isCloseToGoal = IsCloseToGoal();
            bool isReallyCloseToGoal = IsReallyCloseToGoal();
            bool isCloseToPlayer = IsCloseToPlayer();

            float distance_scaled = Vector3.Distance(targetTransform.localPosition, ballTransform.localPosition) / maxDistance;

            if (isReallyCloseToGoal && kicked && isStopped)
            {
                AddReward(0.6f);
                OnEndEpisode();
            }
            else if (kicked && isStopped && isCloseToGoal) {
                AddReward(0.3f);
                OnEndEpisode();
            }
            else if (kicked && isStopped)
            {
                if (isCloseToPlayer)
                {
                    AddReward((-distance_scaled / 10) * 2);
                }
                else
                {
                    AddReward(-distance_scaled / 10);
                }
                IsGoodAngle();
                OnEndEpisode();
            }

            if (ballTransform.localPosition.y < -5)
            {
                AddReward(-1);
                OnEndEpisode();
            }
        }


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
        if (!kicked)
        {
            kicked = true;
            ballRigidbody.constraints = RigidbodyConstraints.None;
            // Apply rotations around both X and Y axes to the ballTransform
            ballTransform.localRotation = Quaternion.Euler(-angleX, angleY, 0);

            // Calculate the direction and force
            Vector3 kickDirection = ballTransform.forward;
            Vector3 kickForce = kickDirection * strength;

            ballRigidbody.AddForce(kickForce, ForceMode.Impulse);
        }
        else return;
    }

    private void IsGoodAngle()
    {
        float angleToGoal = Vector3.SignedAngle(ballTransform.localPosition - transform.localPosition, targetTransform.localPosition - transform.localPosition, Vector3.up);
        float angleReward = 1f - Mathf.Abs(angleToGoal) / 75f;

        AddReward(angleReward);
        OnEndEpisode();
    }

    private bool IsCloseToPlayer()
    {
        bool isCloseToPlayer = Vector3.Distance(ballTransform.localPosition, transform.localPosition) < 3f;

        return isCloseToPlayer;
    }

    private bool IsCloseToGoal()
    {
        bool isCloseToTarget = Vector3.Distance(ballTransform.localPosition, targetTransform.localPosition) < 8f;

        return isCloseToTarget;
    }

    private bool IsReallyCloseToGoal()
    {
        bool isReallyCloseToGoal = Vector3.Distance(ballTransform.localPosition, targetTransform.localPosition) < 4f;

        return isReallyCloseToGoal;
    }

    public void GoalTriggered()
    {
        // The ball hits the gameobject which has goal tag on it
        AddReward(1f);
        OnEndEpisode();
    }

    public void BoundaryTriggered()
    {
        // The ball hits one of the gameobjects which has boundary tag on it
        AddReward(-0.3f);
        OnEndEpisode();
    }

    public void OnEndEpisode()
    {
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        episodeHasBegun = false;
        EndEpisode();
    }

}
