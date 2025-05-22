using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class TestAgent : Agent
{
    public float moveSpeed = 1f;
    public Material successMat;
    public Material failMat;
    public Material startMat;
    [SerializeField] private Transform targetTrans;
    [SerializeField] private Renderer rend;
    


    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(Random.Range(-3.5f,-1f),0f, Random.Range(-3.5f, 3.5f));
        targetTrans.localPosition = new Vector3(Random.Range(3.5f, 1f), 0f, Random.Range(-3.5f, 3.5f));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTrans.localPosition);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        transform.localPosition += new Vector3(moveX, 0f, moveZ) * Time.deltaTime * moveSpeed;
    }
    private void OnTriggerEnter(Collider col)
    {
        if (col.gameObject.name == "TestGoal")
        {
            SetReward(1f);
            rend.material = successMat;
        }
        if(col.gameObject.name == "Wall")
        {
            SetReward(-1f);
            rend.material = failMat;
        }
        EndEpisode();
    }
}
