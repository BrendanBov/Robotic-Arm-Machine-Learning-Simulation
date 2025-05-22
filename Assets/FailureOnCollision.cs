//DEPRICATED, see Toy Robot

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FailureOnCollision : MonoBehaviour
{
    public string colName;
    public float reward;

    [SerializeField] private RobotArmAgent trainingManager;

    void OnCollisionEnter(Collision col)
    {
        if(col.gameObject.name == colName)
        {
            trainingManager.EpisodeFailed(reward);
        }
    }
}
