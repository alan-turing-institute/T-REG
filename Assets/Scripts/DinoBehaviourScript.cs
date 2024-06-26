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
    private float targetWalkingSpeed = 10f;

   // private float m_arenaSize = 100f;

    public float TargetWalkingSpeed // property
    {
        get { return targetWalkingSpeed; }
        set { targetWalkingSpeed = Mathf.Clamp(value, .1f, maxWalkingSpeed); }
    }

    const float maxWalkingSpeed = 10; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    public bool randomizeWalkSpeedEachEpisode;

    //The direction an agent will walk during training.
    private Vector3 worldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController orientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator directionIndicator;
    JointDriveController jdController;
    EnvironmentParameters resetParams;

    // position and distance to target
    private Vector3 lastPosition;
    private float lastDistanceToTarget;

    // to keep track of number of steps agent has taken in an episode
    private int episodeSteps = 0;
    private double episodeRewardTracker = 0;
    private int episodeCounter = 0;

    private List<Rigidbody> allRigidBodies = new List<Rigidbody>();

    public override void Initialize()
    {
        FindAllRigidBodies findRigidBodies = GetComponent<FindAllRigidBodies>();
        allRigidBodies = findRigidBodies.CountBodies();
        print("DinoAgent.Initialize: list of rigid bodies: " + allRigidBodies.Count);

        orientationCube = GetComponentInChildren<OrientationCubeController>();
        directionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        jdController = GetComponent<JointDriveController>();
        jdController.SetupBodyPart(mainBody);

        jdController.SetupBodyPart(footL);
        jdController.SetupBodyPart(footR);

        jdController.SetupBodyPart(but);

        jdController.SetupBodyPart(hipL);
        jdController.SetupBodyPart(thighL);
        jdController.SetupBodyPart(shinL);
        jdController.SetupBodyPart(hipR);
        jdController.SetupBodyPart(thighR);
        jdController.SetupBodyPart(shinR);

        jdController.SetupBodyPart(tail1);
        jdController.SetupBodyPart(tail2);
        jdController.SetupBodyPart(tail3);

        jdController.SetupBodyPart(spineLower);
        jdController.SetupBodyPart(neck);

        jdController.SetupBodyPart(jawTop);
        jdController.SetupBodyPart(jawBottom);

        jdController.SetupBodyPart(shoulderL);
        jdController.SetupBodyPart(shoulderR);

        jdController.SetupBodyPart(armL);
        jdController.SetupBodyPart(armR);

        jdController.SetupBodyPart(forArmL);
        jdController.SetupBodyPart(forArmR);

        resetParams = Academy.Instance.EnvironmentParameters;

        neckGameObject = GameObject.Find("Neck");

        startingPosition = mainBody.position;

        SetResetParameters();
    }

    // Copied from WalkerAgent.cs
    //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects()
    {
        worldDirToWalk = target.position - but.position;
        orientationCube.UpdateOrientation(but, target);
        if (directionIndicator)
        {
            directionIndicator.MatchOrientation(orientationCube.transform);
        }
    }

    public override void OnEpisodeBegin()
    {
        // print the total episode reward every 50 episodes
       // if (this.episode_counter % 10 == 0) {
        //    print("episode " + this.episode_counter + " reward: " + episode_reward_tracker);
       // }

        // increment episode counter
        this.episodeCounter += 1;

        // reset other trackers
        this.episodeSteps = 0;
        this.episodeRewardTracker = 0;

        // If the Agent fell, zero its momentum
        // print("DinoAgent.OnEpisodeBegin. My caller: " + (new System.Diagnostics.StackTrace()).GetFrame(1).GetMethod().Name);

        foreach (var rb in allRigidBodies) {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        //Reset all of the body parts
        foreach (var bodyPart in jdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }
        mainBody.rotation = Quaternion.Euler(-90, 0, 0);
        mainBody.position = startingPosition;

        // Randomly start the agent with small variation on the y-axis. For fixed position, use the comment line below
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
        var bpDict = jdController.bodyPartsDict;
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
        this.episodeSteps += 1;

        // check for terminal state and corresponding reward
        float distanceToTarget = Vector3.Distance(neckGameObject.transform.position, target.position);

        // Reached target
        if (distanceToTarget < 1f) {
            // agent has reached target, positvely reward agent it and exit episode.
            SetReward(1.0f);
            this.episodeRewardTracker += 1.0f;
            EndEpisode();
        }
        else if (distanceToTarget > 150f) {
            // agent is too far from target, negatively reward it and exit episode.
            SetReward(-0.5f);
            this.episodeRewardTracker += -0.5f;
            EndEpisode();
        }
        else {
            // Add a reward for balance.

            // balance reward
            // Calculate dot product between up vector for 'but' and the world's up vector.
            // This will be 1 when 'but' is exactly upright, and less than 1 as 'but' tilts.
            float balance = Vector3.Dot(but.up, Vector3.up);
            AddReward(balance * 0.005f); // scale the reward
            this.episodeRewardTracker += balance * 0.005f;

            // distance reward
            float distanceDifference = lastDistanceToTarget - distanceToTarget;
            if (distanceDifference > 0)
            {
                AddReward(distanceDifference * 0.05f);  // positive reward when getting closer
                this.episodeRewardTracker += distanceDifference * 0.05f;
            }
            else {
                AddReward(distanceDifference * 0.05f);  // negative reward when getting further away
                this.episodeRewardTracker += distanceDifference * 0.05f;
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

        // suspend t-rex in mid-air (or not)
        Rigidbody mainRigidBody = mainBody.gameObject.GetComponent<Rigidbody>();
        if (Input.GetKey(KeyCode.T)) {
            mainRigidBody.isKinematic = !mainRigidBody.isKinematic;           
        }

        var continuousActionsOut = actionsOut.ContinuousActions;
        // left thigh rotation (along z-axis)
        if (Input.GetKey(KeyCode.Q)) {
            continuousActionsOut[0] = 1f; 
        } else if  (Input.GetKey(KeyCode.W)) {
            continuousActionsOut[0] = -1f; 
        }
        // right thight rotation (along z-axis)
        if (Input.GetKey(KeyCode.O)) {
            continuousActionsOut[1] = 1f; 
        } else if  (Input.GetKey(KeyCode.P)) {
            continuousActionsOut[1] = -1f;
        }
        // tail rotation (along z-axis)
        if (Input.GetKey(KeyCode.A)) {
            continuousActionsOut[2] = 1f;
        } else if  (Input.GetKey(KeyCode.S)) {
            continuousActionsOut[2] = -1f;
        }
        // lower spine rotation (along x-axis)
        if (Input.GetKey(KeyCode.D)) {
            continuousActionsOut[3] = 1f;
        }  else if  (Input.GetKey(KeyCode.F)) {
            continuousActionsOut[3] = -1f;
        }
        // lower spine rotation (along z-axis)
        if (Input.GetKey(KeyCode.G)) {
            continuousActionsOut[4] = 1f;
        }  else if  (Input.GetKey(KeyCode.H)) {
            continuousActionsOut[4] = -1f;
        }
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }

    public void SetTorsoMass()
    {
        jdController.bodyPartsDict[but].rb.mass = resetParams.GetWithDefault("chest_mass", 8);
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
        foreach (var item in jdController.bodyPartsList)
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
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to `but` in the context of our orientation cube's space
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.position - but.position));

        if (bp.rb.transform != but && bp.rb.transform != thighL && bp.rb.transform != thighR)
        {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / jdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// Note: this method was copied from WalkerAgent.cs equivalent with minor
    /// modifications.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = orientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //Dino's avg vel
        var avgVel = GetAvgVelocity();

        //current Dino velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas
        sensor.AddObservation(Quaternion.FromToRotation(but.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(but.forward, cubeForward));

        //Position of target position relative to cube
        sensor.AddObservation(orientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in jdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }
}
