//Robot Arm Agent
//Brendan Bovenschen
//Handles environmnet parameters
//and controls the agent

using System.Collections.Generic;
using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotArmAgent : Agent
{
    [Header("Configurations")]
    public TrainingMode trainingMode = TrainingMode.Reinforcement;
    public int maxStepImitation = 5000;

    [Header("Agent Properties")]

    public List<RobotJoint> joints = new();
    public ClawJoint clawJoint;
    public GameObject toyRobot;
    public Transform goal;


    [Header("Goal Properties")]

    public float rewardDistRobot = 5f;
    public float rewardDistPickup = 0.5f;
    public float rewardDistGoal = 6f;
    public int maxStepGrabbing = 1000;
    public int maxStepPickingUp = 1500;
    public int maxStepPlacing = 2000;
    public CurriculumState currentState = CurriculumState.Grabbing;

    [SerializeField] private Transform toyRobotSpawnMin;
    [SerializeField] private Transform toyRobotSpawnMax;
    [SerializeField] private Transform goalSpawnMin;
    [SerializeField] private Transform goalSpawnMax;
    [SerializeField] private Transform handOrientation;
    [SerializeField] private Transform goalTransform;


    [Header("Visual Feedback")]

    [SerializeField] private Renderer groundRend;
    [SerializeField] private Material successMat;
    [SerializeField] private Material failureMat;

    private float[] heuristicControls = { 0f, 0f, 0f, 0f, 0f, 0f };  // Values for Heuristic controls
    private Vector3 toyRobotStartPos = Vector3.zero;
    private bool allowWin = false;


    [Serializable]
    public class RobotJoint
    {
        public HingeJoint joint;
        public float speed;
        public PosRot ps;

        RobotJoint()
        {
            ps = new PosRot();
        }
    }

    [Serializable]
    public class ClawJoint
    {
        public ConfigurableJoint clawL;
        public ConfigurableJoint clawR;
        public PosRot clawLPS;
        public PosRot clawRPS;
        public float speed;

        ClawJoint()
        {
            clawLPS = new PosRot();
            clawRPS = new PosRot();
        }
    }

    public class PosRot
    {
        public Vector3 pos;
        public Quaternion rot;

        public void Set(Transform t)
        {
            pos = t.localPosition;
            rot = t.localRotation;
        }

        public void Get(Transform t)
        {
            t.localPosition = pos;
            t.localRotation = rot;
        }
    }

    [Serializable]
    public enum CurriculumState
    {
        Grabbing,
        PickingUp,
        Placing
    }

    [Serializable]
    public enum TrainingMode
    { 
        Reinforcement,
        Imitation,
        Curriculum
    }


    public void HControlJoint(int jointID, float vel)
    {
        heuristicControls[jointID] = vel;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        for(int i = 0; i < heuristicControls.Length; i++)
        {
            // Do axis control if button is pressed
            continuousActions[i] = heuristicControls[i];
        }
    }

    public void HandleRewards()
    {
        Vector3 initPos = Vector3.zero;
        Vector3 finalPos = Vector3.zero;
        float rewardDist = 0f;

        switch (currentState)
        {
            case CurriculumState.Grabbing:
                initPos = handOrientation.position;
                finalPos = toyRobot.transform.position;
                rewardDist = rewardDistRobot;
                break;
            case CurriculumState.PickingUp:
                initPos = toyRobot.transform.position.y * Vector3.up;
                finalPos = (toyRobotStartPos.y + 2.5f) * Vector3.up;
                rewardDist = rewardDistPickup;
                break;
            case CurriculumState.Placing:
                initPos = toyRobot.transform.position;
                finalPos = goalTransform.position;
                rewardDist = rewardDistGoal;
                break;
        }

        // Allow reward once in range of distance
        float distance = Vector3.Distance(initPos, finalPos);
        float reward = (rewardDist - distance);
        reward = reward / rewardDist;
        reward = Mathf.Clamp(reward, -1f, 1f);

        if(MaxStep != 0)
            AddReward(reward / MaxStep);

        if(distance < 0.7f)
        {
            SetReward(1f);
            EndEpisode();
        }

    }

    public void Start()
    {
        // Save joint positions to be set at the beginning of each episode
        foreach (var j in joints)       
        {
            j.ps = new PosRot();
            j.ps.Set(j.joint.transform);
        }
        clawJoint.clawLPS.Set(clawJoint.clawL.transform);
        clawJoint.clawRPS.Set(clawJoint.clawR.transform);
    }

    public override void OnEpisodeBegin()
    {
        if(trainingMode == TrainingMode.Curriculum)
        {
            float param = Academy.Instance.EnvironmentParameters.GetWithDefault("train_grabbing", -1f);

            switch (param)
            {
                case 0f:
                    currentState = CurriculumState.Grabbing;
                    MaxStep = maxStepGrabbing;
                    break;
                case 1f:
                    currentState = CurriculumState.PickingUp;
                    MaxStep = maxStepPickingUp;
                    break;
                case 2f:
                    currentState = CurriculumState.Placing;
                    MaxStep = maxStepPlacing;
                    break;
                default: Debug.LogError("could not find parameter"); break;
            }
        }
        else
        {
            currentState = CurriculumState.Placing;
        }

        if (trainingMode == TrainingMode.Imitation) MaxStep = maxStepImitation;

        allowWin = currentState == CurriculumState.Placing;

        foreach (var j in joints)   // Set joints
        {
            j.ps.Get(j.joint.transform);
        }
        clawJoint.clawLPS.Get(clawJoint.clawL.transform);
        clawJoint.clawRPS.Get(clawJoint.clawR.transform);

        // Randomize toy robot spawn
        Vector3 toyRobotLocalPos = RandomVect(toyRobotSpawnMin.localPosition, toyRobotSpawnMax.localPosition);
        toyRobotStartPos = toyRobot.transform.localToWorldMatrix * toyRobotLocalPos;
        toyRobot.transform.localPosition = toyRobotLocalPos;
        toyRobot.transform.localRotation = Quaternion.identity;
        Rigidbody toyRobotRb = toyRobot.GetComponent<Rigidbody>();
        toyRobotRb.velocity = Vector3.zero;
        toyRobotRb.angularVelocity = Vector3.zero;

        // Randomize goal position
        goal.localPosition = RandomVect(goalSpawnMin.localPosition, goalSpawnMax.localPosition);
    }

    // Total Observation Vectors: 
    public override void CollectObservations(VectorSensor sensor)
    {
        Quaternion handLocalRot = handOrientation.transform.rotation * transform.rotation;

        // Hand orientaiton, position local to work with multiple training sets
        sensor.AddObservation(handOrientation.position - transform.position);       // 3 observation vectors
        sensor.AddObservation(handLocalRot);                                        // 4 observation vectors

        // Toy robot position and rotation
        sensor.AddObservation(toyRobot.transform.localPosition);                    // 3 observation vectors
        sensor.AddObservation(toyRobot.transform.localRotation);                    // 4 observation vectors

        // Goal position
        sensor.AddObservation(goal.transform.localPosition);                        // 3 observation vectors

        foreach(var j in joints)                                                    // 5 x 7 observation vectors
        {
            // operations performed to get position and rotation local to test environment
            sensor.AddObservation(j.joint.transform.position - transform.position); // 3 observation vectors
            sensor.AddObservation(j.joint.transform.rotation * transform.rotation); // 4 observation vectors
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
       
        int ContID = 0; // Action ID for Continuous Actions

        foreach(var j in joints)
        {
            var joint = j.joint;        // Get joint reference and speed from list
            var speed = j.speed;

            var motor = joint.motor;    // Set target velocity of motor (copy of motor)
            motor.targetVelocity = actions.ContinuousActions[ContID] * speed;
            joint.motor = motor;

            ContID++;
        }

        Vector3 targetSpeed = clawJoint.speed * actions.ContinuousActions[ContID] * Vector3.up;
        clawJoint.clawL.targetVelocity = targetSpeed;
        clawJoint.clawR.targetVelocity = targetSpeed;

        HandleRewards();
        
    }

    public void EpisodeFailed(float reward)
    {
        AddReward(reward);
        groundRend.material = failureMat;
        EndEpisode();
    }

    public void EpisodeWon()
    {
        if (!allowWin) return;
        SetReward(1f);
        groundRend.material = successMat;
        EndEpisode();
    }

    private Vector3 RandomVect(Vector3 min, Vector3 max)
    {
        return new Vector3(UnityEngine.Random.Range(min.x,max.x), UnityEngine.Random.Range(min.y,max.y), UnityEngine.Random.Range(min.z,max.z));
    }
}
