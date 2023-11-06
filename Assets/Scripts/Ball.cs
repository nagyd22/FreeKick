using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ball : MonoBehaviour
{
    public GoalShootingAgent goalShootingAgent;

    public void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            goalShootingAgent.GoalTriggered();
        }
        else if (other.CompareTag("Boundary"))
        {
            goalShootingAgent.BoundaryTriggered();
        }
    }
}
