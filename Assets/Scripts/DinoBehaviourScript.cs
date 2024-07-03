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

    private List<Vector3> targetPositions;

    private int targetPositionIndex  = 0;

    [Header("Body Parts")]
    public Transform mainBody;
    public Transform footL;
    public Transform footR;
    public Transform butt;
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

    private List<Rigidbody> allRigidBodies = new List<Rigidbody>();

    public override void Initialize() {

        targetPositions = new List<Vector3> {
            new Vector3(0f, 10f, 30f),
            new Vector3(10f, 10f, 50f),
            new Vector3(0f, 10f, 70f),
            new Vector3(-10f, 10f, 90f),
            new Vector3(0f, 10f, 110f)
        };
        target.transform.position = targetPositions[0];
        FindAllRigidBodies findRigidBodies = GetComponent<FindAllRigidBodies>();
        allRigidBodies = findRigidBodies.CountBodies();
        print("DinoAgent.Initialize: list of rigid bodies: " + allRigidBodies.Count);

        orientationCube = GetComponentInChildren<OrientationCubeController>();
        directionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        jdController = GetComponent<JointDriveController>();
     //   jdController.SetupBodyPart(mainBody);

     //   jdController.SetupBodyPart(footL);
     //   jdController.SetupBodyPart(footR);

        jdController.SetupBodyPart(butt);

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

      //  jdController.SetupBodyPart(jawTop);
      //  jdController.SetupBodyPart(jawBottom);

      //  jdController.SetupBodyPart(shoulderL);
      //  jdController.SetupBodyPart(shoulderR);

      //  jdController.SetupBodyPart(armL);
      //  jdController.SetupBodyPart(armR);

      //  jdController.SetupBodyPart(forArmL);
      //  jdController.SetupBodyPart(forArmR);

        resetParams = Academy.Instance.EnvironmentParameters;

        neckGameObject = GameObject.Find("Neck");

        startingPosition = mainBody.position;

        SetResetParameters();
    }

    // Copied from WalkerAgent.cs
    //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects() {
        worldDirToWalk = target.position - butt.position;
        orientationCube.UpdateOrientation(butt, target);
        if (directionIndicator)
        {
            directionIndicator.MatchOrientation(orientationCube.transform);
        }
    }

    public override void OnEpisodeBegin() {

        // If the Agent fell, zero its momentum

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
        targetPositionIndex = 0;
        target.transform.position = targetPositions[targetPositionIndex];
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

    public override void OnActionReceived(ActionBuffers actionBuffers) {
        // take an action.
        // involves rotating the legs (via the thighs) and the tail
        // note, the rotation is clamped between -90 and 90 degrees
        var bpDict = jdController.bodyPartsDict;
        var continuousActions = actionBuffers.ContinuousActions;

        float thighL_target_rotation = Mathf.Clamp(continuousActions[0], -90, 90);
        float thighR_target_rotation = Mathf.Clamp(continuousActions[1], -90, 90);
        float tail1_target_rotation = Mathf.Clamp(continuousActions[2], -90, 90);
        float spineLower_x_target_rotation = Mathf.Clamp(continuousActions[3], -30, 30);
        float spineLower_z_target_rotation = Mathf.Clamp(continuousActions[4], -30, 30);
        float calfL_target_rotation = Mathf.Clamp(continuousActions[5],-70, 70 );
        float calfR_target_rotation = Mathf.Clamp(continuousActions[6],-70, 70 );

        bpDict[thighL].SetJointTargetRotation(0.0f, 0.0f, thighL_target_rotation);
        bpDict[thighL].SetJointStrength(50.1f);

        bpDict[thighR].SetJointTargetRotation(0.0f, 0.0f, thighR_target_rotation);
        bpDict[thighR].SetJointStrength(50.1f);

        bpDict[shinL].SetJointTargetRotation(calfL_target_rotation,0.0f, 0.0f );
        bpDict[shinL].SetJointStrength(50.1f);

        bpDict[shinR].SetJointTargetRotation(calfR_target_rotation,0.0f, 0.0f );
        bpDict[shinR].SetJointStrength(50.1f);

        bpDict[tail1].SetJointTargetRotation(0.0f, 0.0f, tail1_target_rotation);
        bpDict[tail1].SetJointStrength(50.1f);

        bpDict[spineLower].SetJointTargetRotation(spineLower_x_target_rotation, 0.0f, spineLower_z_target_rotation);
        bpDict[spineLower].SetJointStrength(50.1f);

        // Add a reward for balance.
        SetBalanceReward();

        // Reward for coming close to target velocity
        SetVelocityReward();

        // distance reward, including terminating episode if too far away
        SetDistanceReward();
            
    }
    

    private void SetDistanceReward() {
        // check for terminal state and corresponding reward
        float distanceToTarget = Vector3.Distance(neckGameObject.transform.position, target.position);
        // Reached target
        if (distanceToTarget < 2f) {
            // agent has reached target, positively reward it.
            SetReward(1.0f);
            MoveTargetToNextPosition();
        } else if (distanceToTarget < 150f) {
            // reward for getting closer to the target than previous frame.
            float distanceDifference = lastDistanceToTarget - distanceToTarget;
            if (distanceDifference > 0) {
                AddReward(distanceDifference * 0.04f);  // positive reward when getting closer
            } else {
                AddReward(distanceDifference * 0.04f);  // negative reward when getting further away
            }
            lastDistanceToTarget = distanceToTarget;
        } else  {
            // agent is too far from target, negatively reward it and exit episode.
            SetReward(-0.5f);
            EndEpisode();
        }
    }

    private void SetBalanceReward() {
        // Calculate dot product between up vector for 'butt' and the world's up vector.
        // This will be 1 when 'butt' is exactly upright, and less than 1 as 'butt' tilts.
        float balance = Vector3.Dot(butt.up, Vector3.up);
        AddReward(balance * 0.003f); // scale the reward

    }

    private void SetVelocityReward() {
        var cubeForward = orientationCube.transform.forward;
        var velocityGoal = cubeForward * TargetWalkingSpeed;
        var actualVelocity = GetAvgVelocity();
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);
        float reward = 0.1f*Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
        AddReward(reward);
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

        // left calf rotation (along z-axis)
        if (Input.GetKey(KeyCode.Z)) {
            continuousActionsOut[5] = 1f;
        }  else if  (Input.GetKey(KeyCode.X)) {
            continuousActionsOut[5] = -1f;
        }

        // right calf rotation (along z-axis)
        if (Input.GetKey(KeyCode.C)) {
            continuousActionsOut[6] = 1f;
        }  else if  (Input.GetKey(KeyCode.V)) {
            continuousActionsOut[6] = -1f;
        }

        if (Input.GetKey(KeyCode.N)) {
            MoveTargetToNextPosition();
        }
    }

    private void MoveTargetToNextPosition() {
        targetPositionIndex += 1;
        if (targetPositionIndex == targetPositions.Count) {
            EndEpisode();
        } else {
            target.transform.position = targetPositions[targetPositionIndex];
        }
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }

    public void SetTorsoMass()
    {
        jdController.bodyPartsDict[butt].rb.mass = resetParams.GetWithDefault("chest_mass", 8);
        // print("Chest mass is " + m_JdController.bodyPartsDict[butt].rb.mass);
    }

    // Copied from WalkerAgent.cs
    //Returns the average velocity of all of the body parts
    //Using the velocity of the hips only has shown to result in more erratic movement from the limbs, so...
    //...using the average helps prevent this erratic movement
    Vector3 GetAvgVelocity() {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in jdController.bodyPartsList) {
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
    /// Six observations per body part.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK - 1 observation
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        //Get velocities in the context of our orientation cube's space - 6 observations
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to `butt` in the context of our orientation cube's space - 3 observations
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(bp.rb.position - butt.position));

        //if (bp.rb.transform != butt && bp.rb.transform != thighL && bp.rb.transform != thighR)
        //{
        // rotations and forces - 5 observations.
        sensor.AddObservation(bp.rb.transform.localRotation);
        sensor.AddObservation(bp.currentStrength / jdController.maxJointForceLimit);
        //}
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// Note: this method was copied from WalkerAgent.cs equivalent with minor
    /// modifications.
    /// Total number of observations collected is: 
    /// 20 plus nBodyPart * 15, plus any ray sensors (currently 8 rays).
    /// Currently have 12 body parts in jointDriveController.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = orientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //Dino's avg vel
        var avgVel = GetAvgVelocity();

        //current Dino velocity. normalized - 3 observations
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube - 3 observations
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube - 3 observations
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(velGoal));

        //rotation deltas - 8 observations
        sensor.AddObservation(Quaternion.FromToRotation(butt.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(butt.forward, cubeForward));

        //Position of target position relative to cube - 3 observations
        sensor.AddObservation(orientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in jdController.bodyPartsList) {
            CollectObservationBodyPart(bodyPart, sensor);
        }
       // print("Observation size: " + sensor.ObservationSize());
    }

    private void Update() {
        /// check that both feet are above ground,
        /// or if not, end the episode.
        if (footL.position.y < 0f)  {
            print("Left foot got stuck under the ground :( )");
            AddReward(-0.2f);
            footL.position = new Vector3(footL.position.x,footL.position.y+0.02f,footL.position.z+0.002f);
            EndEpisode();
        } else if (footR.position.y < 0f)  {
            print("Right foot got stuck under the ground :( )");
            AddReward(-0.2f);
            footR.position = new Vector3(footR.position.x,footR.position.y+0.002f,footR.position.z+0.002f);;
            EndEpisode();
        }
    }
}
