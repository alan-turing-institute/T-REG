using System;
using System.Collections;
using System.Collections.Generic;
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
    // public Transform target;

    [Header("Body Parts")]
    public Transform thighL;
    public Transform thighR;
    public Transform shinL;
    public Transform shinR;
    public Transform but;
    public Transform tail_1;


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

    // position and distance to target
    private Vector3 lastPosition;
    private float lastDistanceToTarget;

    // to keep track of number of steps agent has taken in an episode
    private int episode_steps = 0;
    private int max_episode_steps = 1000;

    private List<Rigidbody> all_rigid_bodies = new List<Rigidbody>();

    public override void Initialize()
    {
        FindAllRigidBodies find_rigid_bodies = GetComponent<FindAllRigidBodies>();
        all_rigid_bodies = find_rigid_bodies.CountBodies();
        print("DinoAgent.Initialize: list of rigid bodies: " + all_rigid_bodies.Count);

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
        m_JdController.SetupBodyPart(tail_1);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        neck = GameObject.Find("Neck");

        SetResetParameters();
        left_action = 0;
        right_action = 0;
    }
    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        print("DinoAgent.OnEpisodeBegin. My caller: " + (new System.Diagnostics.StackTrace()).GetFrame(1).GetMethod().Name);

        foreach (var rb in all_rigid_bodies) {
            // rb.transform.position = bp.startingPos;
            // rb.transform.rotation = bp.startingRot;
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        // Fixed starting position on episode reset.
        // this.transform.localPosition = new Vector3(22, 25, -12); 

         // Randomized starting position on episode reset. Best to keep Y tight. 
        float randomX = UnityEngine.Random.Range(20f, 25f); 
        float randomY = UnityEngine.Random.Range(20f, 22f); 
        float randomZ = UnityEngine.Random.Range(-15f, -10f); 
        this.transform.localPosition = new Vector3(randomX, randomY, randomZ); 

        // if (this.transform.localPosition.y < 0)
        // {
        //     // this.rBody.angularVelocity = Vector3.zero;
        //     // this.rBody.velocity = Vector3.zero;
        //     this.transform.localPosition = new Vector3( 0, 0.5f, 0);
        // }

        // Move the target to a new spot
        // Target.position = new Vector3(Random.value * 8 - 4,
        //                                 5f,
        //                                 Random.value * 8 - 4);
        target.transform.localPosition = new Vector3(22, 3, 40);
        SetResetParameters();

        // get position and distance to target
        lastPosition = neck.transform.position;
        lastDistanceToTarget = Vector3.Distance(neck.transform.position, target.position);
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }

    public void SetTorsoMass()
    {
        m_JdController.bodyPartsDict[but].rb.mass = m_ResetParams.GetWithDefault("chest_mass", 8);
        // print("Chest mass is " + m_JdController.bodyPartsDict[but].rb.mass);
    }

    //     void Start(){
    //         print("DinoBehaviour started");

    //     }


    // Copied from WalkerAgent.cs
    //Returns the average velocity of all of the body parts
    //Using the velocity of the hips only has shown to result in more erratic movement from the limbs, so...
    //...using the average helps prevent this erratic movement
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// Note: this method was copied from WalkerAgent.cs equivalent with minor
    /// modifications.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to `but` in the context of our orientation cube's space
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - but.position));

        if (bp.rb.transform != but && bp.rb.transform != thighL && bp.rb.transform != thighR)
        {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// Note: this method was copied from WalkerAgent.cs equivalent with minor
    /// modifications.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // print("in CollectObservations");
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);
        // sensor.AddObservation(0);

        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * MTargetWalkingSpeed;
        //Dino's avg vel
        var avgVel = GetAvgVelocity();

        //current Dino velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas
        sensor.AddObservation(Quaternion.FromToRotation(but.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(but.forward, cubeForward));

        //Position of target position relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // let's move the legs (rotation between -90 and 90 degrees)
        var bpDict = m_JdController.bodyPartsDict;
        var continuousActions = actionBuffers.ContinuousActions;

        float thighR_target_rotation = Mathf.Clamp(continuousActions[0] * 90, -90, 90);
        float thighL_target_rotation = Mathf.Clamp(continuousActions[1] * 90, -90, 90);

        bpDict[thighR].SetJointTargetRotation(thighR_target_rotation, thighR_target_rotation, thighR_target_rotation);
        bpDict[thighL].SetJointTargetRotation(thighL_target_rotation, thighL_target_rotation, thighL_target_rotation);

        bpDict[thighR].SetJointStrength(0.1f);
        bpDict[thighL].SetJointStrength(0.1f);
        
        // tail movement
        float tail1_target_rotation = Mathf.Clamp(continuousActions[2] * 90, -90, 90);
        bpDict[tail_1].SetJointTargetRotation(tail1_target_rotation, tail1_target_rotation, tail1_target_rotation);
        bpDict[tail_1].SetJointStrength(0.1f);

        // Add a reward for balance.
        // Calculate dot product between up vector for 'but' and the world's up vector.
        // This will be 1 when 'but' is exactly upright, and less than 1 as 'but' tilts.
        float balance = Vector3.Dot(but.up, Vector3.up);
        // Scaling 
        AddReward(balance * 0.1f);

        float distanceToTarget = Vector3.Distance(neck.transform.position, target.position);
        // print("distanceToTarget "+distanceToTarget);
        // Reached target
        if (distanceToTarget < 1f) {
            SetReward(1.0f);
            EndEpisode();

            // reset steps counter
            this.episode_steps = 0;
        }
        // else if (distanceToTarget > 80f) {
        else if (distanceToTarget > 150f) {
            print("Distance to target: " + distanceToTarget);
            SetReward(-1f);
            EndEpisode();

            // reset steps counter
            this.episode_steps = 0;
        }

        // calculate reward
        float currentDistanceToTarget = Vector3.Distance(neck.transform.position, target.position);
        float distanceDifference = lastDistanceToTarget - currentDistanceToTarget;

        // only reward the agent if it got closer to the target
        if (distanceDifference > 0)
        {
            AddReward(distanceDifference * 1f);  // positive reward when getting closer
        } else {
            AddReward(distanceDifference * 0.1f);  // negative reward when getting further away
        }

        // standing upright bonus
        // var uprightBonus = but.up.y;
        // AddReward(uprightBonus * 0.05f);

        // penalize too much movement
        // var continuousActionsOut = actionsOut.ContinuousActions;
        // float jointMovementCost = continuousActionsOut[0] * continuousActionsOut[1] * -0.01f;
        // AddReward(jointMovementCost);

        lastDistanceToTarget = currentDistanceToTarget;

        this.episode_steps += 1;
        if (this.episode_steps >= this.max_episode_steps){
            print("DinoAgent: Maximum episode steps reached. End episode and reset.");
            SetReward(-1f);
            EndEpisode();

            // reset steps counter
            this.episode_steps = 0;
        }
    }

    /// <summary>
    /// This method is repeatedly called when the TRex simulator is run in Unity
    /// without any RL. In essence, the method is called when the TRex is not being
    /// controlled by RL or a pre-trained model
    /// </summary>
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
        //EndEpisode();

    }
}
