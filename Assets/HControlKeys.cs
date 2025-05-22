//Heuristic Controls Keyboard
//Brendan Bovenschen
//Handles keyboard input to control the robot arm
//Used for collection sample trials for imitation learning

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class HControlKeys: MonoBehaviour
{
    public List<KeyInput> keys = new();

    [SerializeField] private RobotArmAgent trialManager;

    [Serializable]
    public struct KeyInput
    {
        public bool isGripper;
        public string key;
        public int jointID;
        public float vel;
    }

    void Update()
    {
        foreach(var key in keys)
        {
            if(Input.GetKeyDown(key.key))
                trialManager.HControlJoint(key.jointID, key.vel);
            else if (Input.GetKeyUp(key.key))
            {
                if (key.isGripper) return; // Gripper should continue to grip

                trialManager.HControlJoint(key.jointID, 0f);
            }
        }
    }
}
