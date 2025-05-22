//Heuristic Control Button
//Brendan Bovenschen
//UI element for controlling the robotic arm

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class HControlButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler 
{
    public int jointID;
    public float vel;

    public bool isGripper = false;

    [SerializeField] private RobotArmAgent trialManager;

    public void OnPointerDown(PointerEventData eventData)
    {
        trialManager.HControlJoint(jointID, vel);
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        if (isGripper) return; // Gripper should continue to grip

        trialManager.HControlJoint(jointID, 0f);
    }
}
