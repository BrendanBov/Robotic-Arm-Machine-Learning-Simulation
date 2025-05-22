//DEPRICATED, see "HControlButton"

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HControlUI : MonoBehaviour
{
    [SerializeField] private RobotArmAgent trainingManager;

    public void ControlJoint(string cmd)
    {
        if (cmd.Length < 3)
        {
            Debug.LogError("Invalid control input");
            return;
        }
        string[] splitCmd = cmd.Split(' ');
        int jointID = int.Parse(splitCmd[0]);
        float vel = float.Parse(splitCmd[1]);

        trainingManager.HControlJoint(jointID, vel);

    }
}
