// using System.Collections;
// using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class ToGoalAgent : Agent
{
    [SerializeField] private Transform targetTransform;

    public override void OnEpisodeBegin() {
        transform.position = new Vector3(0.5f, 0.5f, 0.5f);
    }


    public override void CollectObservations(VectorSensor sensor) {
        sensor.AddObservation(transform.position);
        sensor.AddObservation(targetTransform.position);

    }

    public override void OnActionReceived(ActionBuffers actionBuffers) {
        // Debug.Log("0:"+actionBuffers.ContinuousActions[0]);
        // Debug.Log("1:"+actionBuffers.ContinuousActions[1]);
        // Debug.Log("hello");
        float moveX = actionBuffers.ContinuousActions[0];
        // float moveX = 0.4f;
        float moveZ = actionBuffers.ContinuousActions[1];
        // float moveZ = 0.4f;

        float moveSpeed = 2f;

        transform.position += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    }

    public override void Heuristic(in ActionBuffers actionsOut) {
        // base.Heuristic(actionsOut);
        ActionSegment<float> continuousActionSegment = actionsOut.ContinuousActions;
        continuousActionSegment[0] = Input.GetAxisRaw("Horizontal");
        continuousActionSegment[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other) {
        if (other.TryGetComponent<GoalScript>(out GoalScript goal)) {
            // Could AddReward
            SetReward(1f);
        }
        if (other.TryGetComponent<WallScript>(out WallScript wall)){
            // Could AddReward
            SetReward(-1f);
        }
        EndEpisode();
    }
    
}
