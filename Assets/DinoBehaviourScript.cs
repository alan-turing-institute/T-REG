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
    private GameObject neckGameObject;

    private Vector3 startingPosition; 

    [Header("Body Parts")]
    public Transform mainBody;

    public Transform footL;
    public Transform footR;

    public Transform but;

    public Transform hipL;
    public Transform thighL;
    public Transform shinL; // this is the left calf consider renmaing to leftCalf
    public Transform hipR;
    public Transform thighR;
    public Transform shinR; // this is the right calf consider renmaing to rightCalf

    public Transform tail1;
    public Transform tail2;
    public Transform tail3;

    public Transform spineLower;
    // public Transform upperSpine;
    public Transform neck;

    public Transform jawTop;
    public Transform jawBottom;

    public Transform shoulderL;
    public Transform shoulderR;

    public Transform armL;
    public Transform armR;

    public Transform forArmL;
    public Transform forArmR;


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
    private double episode_reward_tracker = 0;
    private int episode_counter = 0;

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
        m_JdController.SetupBodyPart(mainBody);

        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(footR);

        m_JdController.SetupBodyPart(but);

        m_JdController.SetupBodyPart(hipL);
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(hipR);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);

        m_JdController.SetupBodyPart(tail1);
        m_JdController.SetupBodyPart(tail2);
        m_JdController.SetupBodyPart(tail3);

        m_JdController.SetupBodyPart(spineLower);
        m_JdController.SetupBodyPart(neck);

        m_JdController.SetupBodyPart(jawTop);
        m_JdController.SetupBodyPart(jawBottom);

        m_JdController.SetupBodyPart(shoulderL);
        m_JdController.SetupBodyPart(shoulderR);

        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(armR);

        m_JdController.SetupBodyPart(forArmL);
        m_JdController.SetupBodyPart(forArmR);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        neckGameObject = GameObject.Find("Neck");

        startingPosition = mainBody.position;

        SetResetParameters();
    }

    // Copied from WalkerAgent.cs
    //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - but.position;
        m_OrientationCube.UpdateOrientation(but, target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    public override void OnEpisodeBegin()
    {
        // print the total episode reward every 50 episodes
        if (this.episode_counter % 10 == 0) {
            print("episode " + this.episode_counter + " reward: " + episode_reward_tracker);
        }

        // increment episode counter
        this.episode_counter += 1;

        // reset other trackers
        this.episode_steps = 0;
        this.episode_reward_tracker = 0;

        // If the Agent fell, zero its momentum
        // print("DinoAgent.OnEpisodeBegin. My caller: " + (new System.Diagnostics.StackTrace()).GetFrame(1).GetMethod().Name);

        foreach (var rb in all_rigid_bodies) {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        //Reset all of the body parts
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }
        mainBody.rotation = Quaternion.Euler(-90, 0, 0);
        mainBody.position = startingPosition;

        // Radomly start the agent with small variation on the y-axis. For fixed position, use the comment line below
        //this.transform.localPosition = new Vector3(22f, Random.Range(20f, 25f), -12f);
        // this.transform.localPosition = new Vector3(22, 25, -12); // fixed position
        // this.transform.localPosition = new Vector3(22, 22f, -12); // fixed position
        UpdateOrientationObjects();


        // if (this.transform.localPosition.y < 0)
        // {
        //     // this.rBody.angularVelocity = Vector3.zero;
        //     // this.rBody.velocity = Vector3.zero;
        //     this.transform.localPosition = new Vector3( 0, 0.5f, 0);
        // }

        // Move the target to a new spot
        // target.transform.localPosition = new Vector3(Random.value * 8 - 4, 5f, Random.value * 8 - 4);
        // or keep target in a fixed position (simpler problem, but agent cannot generalize to different positions)
        // target.transform.localPosition = new Vector3(22, 3, 40);
        // print("DinoAgent:Setting new target position");
        // print(target.transform.localPosition);

        SetResetParameters();

        // get position and distance to target
        lastPosition = neckGameObject.transform.position;
        lastDistanceToTarget = Vector3.Distance(neckGameObject.transform.position, target.position);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // take an action.
        // involves rotating the legs (via the thigs) and the tail
        // note, the rotation is clamped between -90 and 90 degrees
        var bpDict = m_JdController.bodyPartsDict;
        var continuousActions = actionBuffers.ContinuousActions;

        float thighL_target_rotation = Mathf.Clamp(continuousActions[0], -90, 90);
        float thighR_target_rotation = Mathf.Clamp(continuousActions[1], -90, 90);
        float tail1_target_rotation = Mathf.Clamp(continuousActions[2], -90, 90);
        float spineLower_x_target_rotation = Mathf.Clamp(continuousActions[3], -30, 30);
        float spineLower_z_target_rotation = Mathf.Clamp(continuousActions[4], -30, 30);

        bpDict[thighL].SetJointTargetRotation(0.0f, 0.0f, thighL_target_rotation);
        bpDict[thighL].SetJointStrength(50.1f);

        bpDict[thighR].SetJointTargetRotation(0.0f, 0.0f, thighR_target_rotation);
        bpDict[thighR].SetJointStrength(50.1f);

        bpDict[tail1].SetJointTargetRotation(0.0f, 0.0f, tail1_target_rotation);
        bpDict[tail1].SetJointStrength(50.1f);

        bpDict[spineLower].SetJointTargetRotation(spineLower_x_target_rotation, 0.0f, spineLower_z_target_rotation);
        bpDict[spineLower].SetJointStrength(50.1f);

        // update episode tracker
        this.episode_steps += 1;

        // check for terminal state and corresponding reward
        float distanceToTarget = Vector3.Distance(neckGameObject.transform.position, target.position);

        // Reached target
        if (distanceToTarget < 1f) {
            // agent has reached target, positvely reward agent it and exit episode.
            SetReward(1.0f);
            this.episode_reward_tracker += 1.0f;
            EndEpisode();
        }
        else if (distanceToTarget > 150f) {
            // agent is too far from target, neegatively reward it and exit episode.
            SetReward(-1f);
            this.episode_reward_tracker += -1.0f;
            EndEpisode();
        }
        else {
            // Add a reward for balance.

            // balance reward
            // Calculate dot product between up vector for 'but' and the world's up vector.
            // This will be 1 when 'but' is exactly upright, and less than 1 as 'but' tilts.
            float balance = Vector3.Dot(but.up, Vector3.up);
            AddReward(balance * 0.1f); // scale the reward
            this.episode_reward_tracker += balance * 0.1f;

            // distance reward
            float distanceDifference = lastDistanceToTarget - distanceToTarget;
            if (distanceDifference > 0)
            {
                AddReward(distanceDifference * 0.1f);  // positive reward when getting closer
                this.episode_reward_tracker += distanceDifference * 0.1f;
            }
            else {
                AddReward(distanceDifference * 0.01f);  // negative reward when getting further away
                this.episode_reward_tracker += distanceDifference * 0.1f;
            }

            lastDistanceToTarget = distanceToTarget;
        }
    }

    /// <summary>
    /// This method is repeatedly called when the TRex simulator is run in Unity
    /// without any RL. In essence, the method is called when the TRex is not being
    /// controlled by RL or a pre-trained model
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // clear  buffer
        actionsOut.Clear();

        var continuousActionsOut = actionsOut.ContinuousActions;
        if (this.episode_steps == 0) {

        }
        else if (this.episode_steps % 1 == 0) {
            print("take an action // step ------> " + this.episode_steps);
            continuousActionsOut[0] = 0.0f; // left thigh rotation (along z-axis)
            continuousActionsOut[1] = 0.0f; // right thight rotation (along z-axis)
            continuousActionsOut[2] = 0.0f; // tail rotation (along z-axis)
            continuousActionsOut[3] = 0.0f; // lower spine rotation (along x-axis)
            continuousActionsOut[4] = 0.0f; // lower spine rotation (along z-axis)
        }
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
}
