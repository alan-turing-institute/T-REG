using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class DinoBehaviourScript : Agent
{
    private float left_action;
    private float right_action;
    private GameObject neck;
    public Transform Target;

    [Header("Body Parts")]
    public Transform thighL;
    public Transform thighR;
    public Transform shinL;
    public Transform shinR;
    public Transform but;


    [Header("Walk Speed")]
    [Range(0.1f, 10)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 10;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    const float m_maxWalkingSpeed = 10; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    public bool randomizeWalkSpeedEachEpisode;

    //The direction an agent will walk during training.
    private Vector3 m_WorldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        print("controller:" + m_JdController);
        m_JdController.SetupBodyPart(but);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(shinL);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        neck = GameObject.Find("Neck");

        SetResetParameters();
        left_action = 0;
        right_action = 0;
    }
    public override void OnEpisodeBegin()
    {
       // If the Agent fell, zero its momentum
        // if (this.transform.localPosition.y < 0)
        // {
            // this.rBody.angularVelocity = Vector3.zero;
            // this.rBody.velocity = Vector3.zero;
            // this.transform.localPosition = new Vector3( 0, 0.5f, 0);
        // }

        // Move the target to a new spot
        // Target.position = new Vector3(Random.value * 8 - 4,
                                        //    5f,
                                        //    Random.value * 8 - 4);
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }

    public void SetTorsoMass()
    {
        m_JdController.bodyPartsDict[but].rb.mass = m_ResetParams.GetWithDefault("chest_mass", 8);
        print("Chest mass is " + m_JdController.bodyPartsDict[but].rb.mass);
    }

    //     void Start(){
    //         print("DinoBehaviour started");

    //     }

    public override void CollectObservations(VectorSensor sensor)
    {
        // print("in CollectObservations");
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
        sensor.AddObservation(0);
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // print("in onactionreceived");
        var bpDict = m_JdController.bodyPartsDict;
        // var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;
        // bpDict[thighR].SetJointTargetRotation(90f, 90f, 90f);
        // bpDict[thighL].SetJointTargetRotation(90f, 90f, 90f);
        // print("Setting target rotation to" + continuousActions[0]);
        bpDict[thighR].SetJointTargetRotation(continuousActions[0], continuousActions[0], continuousActions[0]);
        bpDict[thighL].SetJointTargetRotation(continuousActions[1], continuousActions[1], continuousActions[1]);
        // bpDict[thighL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        //update joint strength settings
        // bpDict[thighR].SetJointStrength(continuousActions[++i]);
        // bpDict[thighL].SetJointStrength(continuousActions[++i]);
        bpDict[thighR].SetJointStrength(1f);
        bpDict[thighL].SetJointStrength(1f);
        // bpDict[thighL].SetJointStrength(500000f);
        
        float distanceToTarget = Vector3.Distance(neck.transform.position, Target.position);
        print("distanceToTarget "+distanceToTarget);
        // Reached target
        if (distanceToTarget < 10f)
        {
            // print("ending episode");
            SetReward(1.0f);
            EndEpisode();
        }
        else if (distanceToTarget > 30f)
        {
            SetReward(-1f);
            EndEpisode();
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        left_action += 0.1f * Input.GetAxis("Horizontal");
        right_action += 0.1f * Input.GetAxis("Vertical");
        continuousActionsOut[0] = left_action;
        continuousActionsOut[1] = right_action;
    }
    private void OnTriggerEnter(Collider other) {
        print("triggered");
        // if (other.TryGetComponent<GoalScript>(out GoalScript goal)) {
        //     // Could AddReward
        //     SetReward(1f);
        // }
        // if (other.TryGetComponent<WallScript>(out WallScript wall)){
        //     // Could AddReward
        //     SetReward(-1f);
        // }
        // EndEpisode();
    }
    private void OnCollisionEnter(Collision collision) {
        print("collided");
        EndEpisode();

    }
}
