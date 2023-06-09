
using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class DinoTwo : Agent
{
    [Header("Body Parts")]
    public Transform thighL;
    public Transform thighR;
    public Transform shinL;
    public Transform shinR;
    public Transform but;

    // [Header("Walk Speed")]
    // [Range(0.1f, 10)]
    // [SerializeField]
    //The walking speed to try and achieve
    // private float m_TargetWalkingSpeed = 10;

    // public float MTargetWalkingSpeed // property
    // {
    //     get { return m_TargetWalkingSpeed; }
    //     set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    // }

    // const float m_maxWalkingSpeed = 10; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    // public bool randomizeWalkSpeedEachEpisode;

    //The direction an agent will walk during training.
    // private Vector3 m_WorldDirToWalk = Vector3.right;

    // [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    // //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    // EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

    //     //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        print(thighR);
        m_JdController.SetupBodyPart(thighR);
    //     m_JdController.SetupBodyPart(thighL);
    //     m_JdController.SetupBodyPart(shinR);
    //     m_JdController.SetupBodyPart(shinL);
    //     m_JdController.SetupBodyPart(but);

    //     m_ResetParams = Academy.Instance.EnvironmentParameters;

    //     SetResetParameters();
    }

    // public void SetResetParameters()
    // {
    //     SetTorsoMass();
    // }

    // public void SetTorsoMass()
    // {
    //     m_JdController.bodyPartsDict[but].rb.mass = m_ResetParams.GetWithDefault("chest_mass", 8);
    //     print("Chest mass is " + m_JdController.bodyPartsDict[but].rb.mass);
    // }

    void Start(){
        print("DinoBehaviour started");

    }

    public override void CollectObservations(VectorSensor sensor){
        print("in CollectObservations");
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
       print("in onactionreceived");
    }

}
