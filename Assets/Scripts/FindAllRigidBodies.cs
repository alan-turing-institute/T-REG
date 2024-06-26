using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using UnityEngine.PhysicsModule;

public class FindAllRigidBodies : MonoBehaviour
{
    // Start is called before the first frame update
    public List<Rigidbody> CountBodies() {
        List<Rigidbody> rigidBodies = new List<Rigidbody>();
        Rigidbody rb = GetComponent<Rigidbody>();

        if (rb != null) rigidBodies.Add(rb);
        TraverseHierarchy(transform, rigidBodies);
        // print("FindAllRB: The rigid bodies: " + rigidBodies.Count);
        return rigidBodies;
    }

    private void TraverseHierarchy(Transform transform, List<Rigidbody> rigidBodies) {
        foreach (Transform child in transform) {
            GameObject go = child.gameObject;
            Rigidbody rb = go.GetComponent<Rigidbody>();
            if (rb != null) rigidBodies.Add(rb);
            TraverseHierarchy(child, rigidBodies);
        }
    }    

}
