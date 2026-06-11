using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public GameObject agent;
    private DinoBehaviourScript dinoScript;
    private Vector3 startingPosition;
    private Vector3 agentStartingPosition;
    private Vector3 offset;
    // Start is called before the first frame update
    void Start()
    {
        startingPosition = transform.position;

        // get the agent's position
        dinoScript = agent.GetComponent<DinoBehaviourScript>();
        agentStartingPosition = dinoScript.GetAvgPosition();
        offset = startingPosition - agentStartingPosition;
    }

    // Update is called once per frame
    void Update()
    {   
        Vector3 dinoPosition = dinoScript.GetAvgPosition();
        transform.position = dinoPosition + offset;
    }
}
