using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RollerRexAgent : Agent
{
    Rigidbody rBody;
    GameObject gObject;
    void Start()
    {
        gObject = GameObject.Find("but");
        rBody = gObject.GetComponent<Rigidbody>();
        // rBody = GetComponent<Rigidbody>();
        print("Rigid body is " + rBody);
    }

    public Transform Target;

    public override void OnEpisodeBegin()
    {
        print("restarting episode");
        // If the Agent fell, zero its momentum
        if (gObject.transform.position.y < 0)
        {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            gObject.transform.position = new Vector3(0, 4, 0);
        }

        // Move the target to a new spot
        Target.position = new Vector3(Random.value * 8 - 4,
                                           0.5f,
                                           Random.value * 8 - 4);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(Target.position);
        sensor.AddObservation(gObject.transform.position);

        // Agent velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);
    }

    public float movementSpeed = 1;

    public override void OnActionReceived(ActionBuffers action)
    {
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = action.ContinuousActions[0];
        controlSignal.z = action.ContinuousActions[1];
        // controlSignal.y = .1;

        print("controlSignal: " + controlSignal);
        gObject.transform.position += controlSignal * Time.deltaTime * movementSpeed;
        print("setting new position to " + gObject.transform.position);
        // rBody.AddForce(controlSignal * forceMultiplier);

        float distanceToTarget = Vector3.Distance(gObject.transform.position, Target.position);
        print("distance:"+distanceToTarget);
        // Reached target
        if (distanceToTarget < 5.0f)
        {
            SetReward(1.0f);
            EndEpisode();
        }
        // Fell off platform
        if (gObject.transform.position.y < 0)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        print("moving manually");
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}