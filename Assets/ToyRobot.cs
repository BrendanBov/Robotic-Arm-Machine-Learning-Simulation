//Toy Robot
//Brendan Bovenschen
//Causes a test to terminate once an object has hit the ground
//or to succeed once the robot has reached the goal

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToyRobot : MonoBehaviour
{
    [SerializeField] RobotArmAgent trialManager;

    private void OnCollisionEnter(Collision col)
    {
        if(col.collider.gameObject.tag == "Ground")
        {
            trialManager.EpisodeFailed(-0.2f);
        }
    }

    private void OnTriggerEnter(Collider col)
    {
        if(col.tag == "Goal")
        {
            trialManager.EpisodeWon();
        }
    }
}
