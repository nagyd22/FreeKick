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

    public float maxDistance;
    public float timeToEndEpisode;
    public float timeSinceDecision;


    [SerializeField] private Transform targetTransform;
    private Transform ballTransform;
    private Rigidbody ballRigidbody;

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

        // Choose a place for the player randomly
        int choosenStartingPoint = UnityEngine.Random.Range(0, possibleStartPositions.Length);
        Vector3 startPosition = possibleStartPositions[choosenStartingPoint];

        // Assign the shooter and the ball to the random position
        transform.localPosition = startPosition;
        ballTransform.localPosition = new Vector3(startPosition.x, -3.36f, startPosition.z - 0.7f);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.constraints = RigidbodyConstraints.FreezeRotation;

        maxDistance = Vector3.Distance(ballTransform.localPosition, targetTransform.localPosition);

        // Assign the boundary behind the shooter
        boundaryBehind.transform.localPosition = new Vector3(boundaryBehind.transform.localPosition.x, boundaryBehind.transform.localPosition.y, startPosition.z + 2f);

        RequestDecision();
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

        float actionXRotate = (actionTaken[0] + 1) / 2;
        float actionYRotate = actionTaken[1];
        float actionPower = (actionTaken[2] + 1) / 2;

        float angleX = actionXRotate * 20;
        float angleY = actionYRotate * 40;
        float strength = actionPower * 12;

        KickBall(angleX, angleY, strength);


    }

    private void WaitTimeUntillEndOfEpisode()
    {
        if(timeSinceDecision >= timeToEndEpisode)
        {
            timeSinceDecision = 0;
            AddReward(-1f);
            OnEndEpisode();
        } else
        {
            timeSinceDecision += Time.fixedDeltaTime;
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        RequestDecision();
        actions[0] = 0;
        actions[1] = 0f;
        actions[2] = 1f;

        if (Input.GetKeyDown(KeyCode.Space))
        {
            RequestDecision();
            actions[2] = 1;
        }
    }

    private void FixedUpdate()
    {
        //FixedUpdate is called before OnEpisodeBegin too. In this way there won't be null reference
        if(episodeHasBegun)
        {
            WaitTimeUntillEndOfEpisode();
            bool isStopped = IsStopped();
            bool isCloseToPlayer = IsCloseToPlayer();

            if (isStopped)
            {
                if (isCloseToPlayer)
                {
                    AddReward(-1);
                }
                else
                {
                    float distanceToGoal = Vector3.Distance(ballTransform.localPosition, targetTransform.localPosition);
                    float distanceReward = Mathf.Clamp01(maxDistance / distanceToGoal);
                    Debug.Log(distanceToGoal);
                    Debug.Log(distanceReward);
                    AddReward(distanceReward);
                }
                OnEndEpisode();
            }

            //If the ball leaves the playable area without touching the boundarys give negative reward and end the episode
            if (ballTransform.localPosition.y < -5)
            {
                AddReward(-1);
                OnEndEpisode();
            }
        }
    }

    /// <summary>
    /// Returns a true/false value whether the ball stopped or not after the kick
    /// </summary>
    /// <returns>True if the ball is stopped and False if the ball is still moving or rotating</returns>
    private bool IsStopped()
    {
        if (kicked && ballRigidbody.IsSleeping())
        {
            return true;
        }
        else { return false; }
    }

    /// <summary>
    /// Kick the ball with the rigidbody's AddForce method with two rotation and the strength when it wasn't already kicked in this episode
    /// </summary>
    /// <param name="angleX">The angle of rotation along the X axis</param>
    /// <param name="angleY">The angle of rotation along the Y axis</param>
    /// <param name="strength">The force we kick the ball with</param>
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
            Vector3 kickForce = -kickDirection * strength;

            ballRigidbody.AddForce(kickForce, ForceMode.Impulse);
        }
        else return;
    }

    //The agent won't just barely touch the ball and go to the next episode
    private bool IsCloseToPlayer()
    {
        bool isCloseToPlayer = Vector3.Distance(ballTransform.localPosition, transform.localPosition) < 3f;

        return isCloseToPlayer;
    }

    /// <summary>
    /// The ball hits the gameobject which has goal tag on it
    /// </summary>
    public void GoalTriggered()
    {
        AddReward(1f);
        OnEndEpisode();
    }

    /// <summary>
    /// The ball hits one of the gameobjects which has boundary tag on it
    /// </summary>
    public void BoundaryTriggered()
    {
        AddReward(-0.5f);
        OnEndEpisode();
    }

    /// <summary>
    /// Set the rotation and velocity of the ball to 0
    /// </summary>
    public void OnEndEpisode()
    {
        ballRigidbody.rotation = Quaternion.Euler(0, 0, 0);
        ballRigidbody.velocity = new Vector3(0, 0, 0);
        episodeHasBegun = false;
        EndEpisode();
    }
}
